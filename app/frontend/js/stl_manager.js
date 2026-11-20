/**
 * STL Importer, Multi-Model Robot Arm Library & Workcell CAD Manager (PAROL6-python-API/app)
 */
class STLManager {
  constructor(robot, scene, appController) {
    this.robot = robot;
    this.scene = scene;
    this.app = appController;
    this.loader = new THREE.STLLoader();
    this.workcellObjects = [];
    this.savedStlInfo = {};
    this.activeModelId = "parol6_default";
    this.activeModelName = "PAROL6 Mặc Định (Chuẩn)";
    this.pendingFiles = [null, null, null, null, null, null, null];
    this.targetLinkForFilePicker = 0;

    this.setupListeners();
  }

  setupListeners() {
    // Single Part STL File Picker (triggered from Part Tree or 3D Part Inspector or File Menu)
    const singleStlInput = document.getElementById('input-single-part-stl');
    if (singleStlInput) {
      singleStlInput.onchange = async (e) => {
        const file = e.target.files?.[0];
        if (!file) return;
        const linkIdx = this.targetLinkForFilePicker;
        await this.importSTLForSinglePart(linkIdx, file);
        singleStlInput.value = '';
      };
    }

    // Header Robot Model Select Dropdown
    const modelSelect = document.getElementById('robot-model-select');
    if (modelSelect) {
      modelSelect.onchange = async (e) => {
        const selectedId = e.target.value;
        if (selectedId) {
          await this.activateRobotModel(selectedId);
        }
      };
    }

    // Workcell CAD Modal
    const modalCAD = document.getElementById('modal-workcell-cad');
    const btnOpenCad = document.getElementById('btn-open-cad-modal');
    if (btnOpenCad) {
      btnOpenCad.onclick = () => { modalCAD.style.display = 'flex'; };
    }
    const menuImportWorkcell = document.getElementById('menu-import-workcell');
    if (menuImportWorkcell) {
      menuImportWorkcell.onclick = () => { modalCAD.style.display = 'flex'; };
    }
    const btnCloseCad = document.getElementById('btn-close-cad-modal');
    if (btnCloseCad) {
      btnCloseCad.onclick = () => { modalCAD.style.display = 'none'; };
    }
    const btnAddCadConfirm = document.getElementById('btn-add-cad-confirm');
    if (btnAddCadConfirm) {
      btnAddCadConfirm.onclick = () => {
        this.loadWorkcellCADFromInput();
        modalCAD.style.display = 'none';
      };
    }
  }

  promptImportSTLForPart(linkIndex) {
    this.targetLinkForFilePicker = linkIndex;
    const inp = document.getElementById('input-single-part-stl');
    if (inp) inp.click();
  }

  async importSTLForSinglePart(linkIndex, file) {
    try {
      const buf = await file.arrayBuffer();
      const geom = this.loader.parse(buf);
      this.pendingFiles[linkIndex] = file;

      // Giữ nguyên 100% gốc tọa độ của file STL khi xuất từ CAD (align_mode = 'cad')
      if (this.robot.linkOffsets[linkIndex]) {
        this.robot.linkOffsets[linkIndex].coord_mode = "world_abs";
        this.robot.linkOffsets[linkIndex].align_mode = "cad";
        this.robot.linkOffsets[linkIndex].dx = 0;
        this.robot.linkOffsets[linkIndex].dy = 0;
        this.robot.linkOffsets[linkIndex].dz = 0;
        this.robot.linkOffsets[linkIndex].rx = 0;
        this.robot.linkOffsets[linkIndex].ry = 0;
        this.robot.linkOffsets[linkIndex].rz = 0;
        this.robot.linkOffsets[linkIndex].pivot_dx = 0;
        this.robot.linkOffsets[linkIndex].pivot_dy = 0;
        this.robot.linkOffsets[linkIndex].pivot_dz = 0;
        this.robot.linkOffsets[linkIndex].pivot_rx = 0;
        this.robot.linkOffsets[linkIndex].pivot_ry = 0;
        this.robot.linkOffsets[linkIndex].pivot_rz = 0;
      }
      this.robot.replaceLinkMesh(linkIndex, geom, file.name);

      // Immediately upload & save to active model on backend so it's persisted
      const upRes = await fetch(
        `/api/models/${encodeURIComponent(this.activeModelId)}/stl/${linkIndex}?filename=${encodeURIComponent(file.name)}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/octet-stream' },
          body: buf
        }
      );
      const upData = await upRes.json();
      if (upData.success) {
        this.savedStlInfo[String(linkIndex)] = { filename: upData.filename, url: upData.url };
        this.robot.linkOffsets[linkIndex].stl_file = upData.filename;
        this.pendingFiles[linkIndex] = null;
      }

      await this.refreshModelListDropdown();
      if (this.app) {
        this.app.selectRobotPart(linkIndex);
        this.app.renderPartTreeList();
        this.app.updateHUDStatus(`Đã gắn STL "${file.name}" vào Part ${linkIndex}`, 'normal');
      }
    } catch (err) {
      alert(`Lỗi khi nạp file STL (${file.name}): ` + err.message);
    }
  }

  async createNewRobotModel() {
    const newName = prompt('Nhập tên cho Model Cánh Tay Robot mới:', 'Robot Cánh Tay Mới');
    if (!newName || !newName.trim()) return;

    this.robot.resetToDefaultMeshes();
    this.savedStlInfo = {};
    this.pendingFiles = [null, null, null, null, null, null, null];

    const res = await fetch('/api/models/save', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        model_id: null,
        model_name: newName.trim(),
        dh_table: this.robot.dhTable,
        link_offsets: this.robot.linkOffsets,
        tcp_offset: this.robot.tcpOffset,
        copy_stls_from_active: false
      })
    });
    const data = await res.json();
    if (data.success) {
      await this.applyLoadedProfileToScene(data);
      await this.refreshModelListDropdown();
      if (this.app) {
        this.app.updateHUDStatus(`Đã tạo Model mới: ${data.model_name}`, 'normal');
      }
    }
  }

  async saveFullRobotModel(modelName, asNewModel = false) {
    try {
      const targetName = (modelName && modelName.trim()) ? modelName.trim() : this.activeModelName;
      const saveRes = await fetch('/api/models/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          model_id: asNewModel ? null : this.activeModelId,
          model_name: targetName,
          dh_table: this.robot.dhTable,
          link_offsets: this.robot.linkOffsets,
          tcp_offset: this.robot.tcpOffset,
          copy_stls_from_active: true,
          workcell_objects: this.getWorkcellPayload()
        })
      });
      const saveData = await saveRes.json();
      if (!saveData.success) throw new Error('Không thể lưu Model');

      const targetModelId = saveData.model_id;
      this.activeModelId = targetModelId;
      this.activeModelName = saveData.model_name;

      // Upload any pending STL files
      for (let i = 0; i <= 6; i++) {
        const file = this.pendingFiles[i];
        if (file) {
          const buf = await file.arrayBuffer();
          const upRes = await fetch(
            `/api/models/${encodeURIComponent(targetModelId)}/stl/${i}?filename=${encodeURIComponent(file.name)}`,
            {
              method: 'POST',
              headers: { 'Content-Type': 'application/octet-stream' },
              body: buf
            }
          );
          const upData = await upRes.json();
          if (upData.success) {
            this.savedStlInfo[String(i)] = { filename: upData.filename, url: upData.url };
            this.robot.linkOffsets[i].stl_file = upData.filename;
          }
          this.pendingFiles[i] = null;
        }
      }

      await this.refreshModelListDropdown();
      if (this.app) {
        this.app.renderPartTreeList();
        this.app.updateHUDStatus(`Đã lưu Model: ${this.activeModelName}`, 'normal');
      }
      return true;
    } catch (err) {
      alert('Lỗi khi lưu Model: ' + err.message);
      return false;
    }
  }

  async refreshModelListDropdown() {
    try {
      const res = await fetch('/api/models');
      if (!res.ok) return;
      const data = await res.json();
      this.activeModelId = data.active_model_id || 'parol6_default';

      const sel = document.getElementById('robot-model-select');
      if (sel && Array.isArray(data.models)) {
        sel.innerHTML = data.models.map(m => `
          <option value="${m.id}" ${m.id === this.activeModelId ? 'selected' : ''}>
            ${m.name} (${m.stl_count} STL)
          </option>
        `).join('');
        const activeObj = data.models.find(m => m.id === this.activeModelId);
        if (activeObj) {
          this.activeModelName = activeObj.name;
          const nameInput = document.getElementById('assembly-model-name');
          if (nameInput) nameInput.value = activeObj.name;
        }
      }
    } catch (err) {
      console.warn('Failed to refresh model list:', err);
    }
  }

  async activateRobotModel(modelId) {
    try {
      const res = await fetch(`/api/models/${encodeURIComponent(modelId)}`);
      if (!res.ok) return;
      const profile = await res.json();
      await this.applyLoadedProfileToScene(profile);
      await this.refreshModelListDropdown();
      if (this.app) {
        this.app.updateHUDStatus(`Đã nạp Model: ${profile.model_name}`, 'normal');
      }
    } catch (err) {
      alert('Lỗi khi nạp Model Cánh Tay: ' + err.message);
    }
  }

  async applyLoadedProfileToScene(profile) {
    this.activeModelId = profile.model_id || 'parol6_default';
    this.activeModelName = profile.model_name || 'PAROL6 Mặc Định';
    this.pendingFiles = [null, null, null, null, null, null, null];

    this.robot.resetToDefaultMeshes();

    if (profile.dh_table && profile.dh_table.length === 6) {
      this.robot.updateDHTable(profile.dh_table);
    }
    if (profile.link_offsets && profile.link_offsets.length === 7) {
      this.robot.updateLinkOffsets(profile.link_offsets);
    }

    if (Array.isArray(profile.workcell_objects)) {
      this.restoreWorkcellObjects(profile.workcell_objects);
    } else {
      this.restoreWorkcellObjects([]);
    }
    if (profile.tcp_offset) {
      this.robot.setTCPOffset(profile.tcp_offset);
    }

    this.savedStlInfo = profile.saved_stls || {};
    const indices = Object.keys(this.savedStlInfo);
    for (const idxStr of indices) {
      const i = parseInt(idxStr, 10);
      const info = this.savedStlInfo[idxStr];
      if (info && info.url) {
        try {
          const stlRes = await fetch(info.url);
          if (stlRes.ok) {
            const buf = await stlRes.arrayBuffer();
            const geometry = this.loader.parse(buf);
            this.robot.replaceLinkMesh(i, geometry, info.filename);
          }
        } catch (e) {
          console.warn(`Failed to load STL for link ${i}:`, e);
        }
      }
    }

    if (profile.link_offsets && profile.link_offsets.length === 7) {
      this.robot.updateLinkOffsets(profile.link_offsets);
    }

    if (Array.isArray(profile.workcell_objects)) {
      this.restoreWorkcellObjects(profile.workcell_objects);
    } else {
      this.restoreWorkcellObjects([]);
    }

    const nameInput = document.getElementById('assembly-model-name');
    if (nameInput) nameInput.value = this.activeModelName;

    if (this.app) {
      this.app.renderPartTreeList();
      this.app.renderDHEditorTable();
      this.app.syncTCPInputsFromModel();
      this.app.setupFKSliders();
      this.app.syncFromJoints(this.robot.joints);
      this.app.updatePickableObjects();
      if (this.app.selectedPartIndex >= 0) {
        this.app.populateInspectorForPart(this.app.selectedPartIndex);
      }
    }
  }

  async loadSavedProfileFromServer() {
    try {
      await this.refreshModelListDropdown();
      const res = await fetch('/api/robot_profile');
      if (!res.ok) return;
      const profile = await res.json();
      await this.applyLoadedProfileToScene(profile);
    } catch (err) {
      console.warn('Could not load saved profile:', err);
    }
  }

  async deleteSinglePart(linkIndex) {
    try {
      this.robot.deleteSingleLinkMesh(linkIndex);
      delete this.savedStlInfo[String(linkIndex)];
      this.pendingFiles[linkIndex] = null;

      await fetch(`/api/models/${encodeURIComponent(this.activeModelId)}/stl/${linkIndex}`, {
        method: 'DELETE'
      });

      await this.refreshModelListDropdown();
      if (this.app) {
        if (this.app.selectedTargetType === 'robot_link' && this.app.selectedPartIndex === linkIndex) {
          this.app.deselectPart();
        }
        this.app.renderPartTreeList();
        this.app.updatePickableObjects();
        this.app.updateHUDStatus(`🗑 Đã xóa Part ${linkIndex} khỏi màn hình 3D`, 'normal');
      }
    } catch (err) {
      alert('Lỗi khi xóa Part: ' + err.message);
    }
  }

  async clearAllModelParts() {
    try {
      // Remove all workcell objects too
      this.workcellObjects.forEach(o => {
        if (o.mesh) this.scene.remove(o.mesh);
      });
      this.workcellObjects = [];
      this.updateWorkcellUI();

      // Remove all robot 3D meshes
      this.robot.clearAllLinkMeshes();
      this.savedStlInfo = {};
      this.pendingFiles = [null, null, null, null, null, null, null];

      await fetch(`/api/models/${encodeURIComponent(this.activeModelId)}/clear_all`, {
        method: 'POST'
      });

      await this.refreshModelListDropdown();
      if (this.app) {
        this.app.deselectPart();
        this.app.renderPartTreeList();
        this.app.updatePickableObjects();
        this.app.updateHUDStatus('🧹 Đã xóa trắng toàn bộ các khối có sẵn & STL trên màn hình 3D', 'normal');
      }
    } catch (err) {
      alert('Lỗi khi xóa trắng 3D: ' + err.message);
    }
  }

  async restoreDefaultModelBlocks() {
    try {
      const res = await fetch(`/api/models/${encodeURIComponent(this.activeModelId)}/restore_defaults`, {
        method: 'POST'
      });
      const profile = await res.json();
      if (profile.success) {
        await this.applyLoadedProfileToScene(profile);
        await this.refreshModelListDropdown();
        if (this.app) {
          this.app.updateHUDStatus('🔄 Đã hiện lại 7 khối 3D mẫu PAROL6', 'normal');
        }
      }
    } catch (err) {
      alert('Lỗi khi khôi phục khối mẫu: ' + err.message);
    }
  }

  async deleteOrResetModel(modelId) {
    try {
      const res = await fetch(`/api/models/${encodeURIComponent(modelId)}`, { method: 'DELETE' });
      const profile = await res.json();
      if (profile.success) {
        await this.applyLoadedProfileToScene(profile);
        await this.refreshModelListDropdown();
        if (this.app) {
          this.app.deselectPart();
          this.app.updateHUDStatus('🗑 Đã xóa Model thành công', 'normal');
        }
      }
    } catch (err) {
      alert('Lỗi khi xóa/reset model: ' + err.message);
    }
  }

  async exportCurrentModelBundle() {
    try {
      await this.saveFullRobotModel(this.activeModelName, false);
      const res = await fetch(`/api/models/${encodeURIComponent(this.activeModelId)}/export_bundle`);
      if (!res.ok) throw new Error('Không thể xuất gói dữ liệu model');
      const bundle = await res.json();
      const blob = new Blob([JSON.stringify(bundle, null, 2)], { type: 'application/json' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = `${this.activeModelId}.robot.json`;
      a.click();
    } catch (err) {
      alert('Lỗi xuất file cánh tay: ' + err.message);
    }
  }

  async importModelBundleFile(file) {
    try {
      const text = await file.text();
      const data = JSON.parse(text);
      const res = await fetch('/api/models/import_bundle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(data)
      });
      const profile = await res.json();
      if (profile.success) {
        await this.applyLoadedProfileToScene(profile);
        await this.refreshModelListDropdown();
        alert(`✅ Đã nạp thành công toàn bộ cơ cấu cánh tay "${profile.model_name}"!`);
      }
    } catch (err) {
      alert('Lỗi khi nạp file cánh tay (.robot.json): ' + err.message);
    }
  }

  /* --------- Workcell 3D Objects --------- */
  _arrayBufferToBase64(buffer) {
    const bytes = new Uint8Array(buffer);
    let binary = '';
    const chunkSize = 0x8000;
    for (let i = 0; i < bytes.length; i += chunkSize) {
      binary += String.fromCharCode.apply(null, bytes.subarray(i, i + chunkSize));
    }
    return btoa(binary);
  }

  _base64ToArrayBuffer(b64) {
    const binary = atob(b64);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) {
      bytes[i] = binary.charCodeAt(i);
    }
    return bytes.buffer;
  }

  getWorkcellPayload() {
    const d = (rad) => Math.round(THREE.MathUtils.radToDeg(rad) * 100) / 100;
    const r1 = (v) => Math.round(v * 100) / 100;
    return this.workcellObjects
      .filter(o => o.mesh && o.stl_b64)
      .map(o => ({
        name: o.name,
        stl_b64: o.stl_b64,
        auto_center: !!o.auto_center,
        pos: [r1(o.mesh.position.x), r1(-o.mesh.position.z), r1(o.mesh.position.y)],
        rot_deg: [d(o.mesh.rotation.x), d(-o.mesh.rotation.z), d(o.mesh.rotation.y)],
        scale: r1(o.mesh.scale.x || 1.0)
      }));
  }

  restoreWorkcellObjects(items) {
    this.workcellObjects.forEach(o => {
      if (o.mesh) this.scene.remove(o.mesh);
    });
    this.workcellObjects = [];

    if (Array.isArray(items)) {
      const r = (deg) => THREE.MathUtils.degToRad(deg || 0);
      items.forEach((item) => {
        if (!item || !item.stl_b64) return;
        try {
          const buf = this._base64ToArrayBuffer(item.stl_b64);
          const geometry = this.loader.parse(buf);
          geometry.rotateX(-Math.PI / 2);
          geometry.computeVertexNormals();
          if (item.auto_center) geometry.center();

          const material = new THREE.MeshStandardMaterial({
            color: 0x94a3b8,
            roughness: 0.6,
            metalness: 0.2
          });
          const mesh = new THREE.Mesh(geometry, material);
          const pos = item.pos || [0, 0, 0];
          const rot = item.rot_deg || [0, 0, 0];
          const sc = item.scale || 1.0;
          mesh.position.set(pos[0] || 0, pos[2] || 0, -(pos[1] || 0));
          mesh.rotation.set(r(rot[0]), r(rot[2]), r(-(rot[1] || 0)));
          mesh.scale.set(sc, sc, sc);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          mesh.name = `CAD_${item.name || 'object.stl'}`;

          const idx = this.workcellObjects.length;
          mesh.userData = { workcellIndex: idx };
          this.scene.add(mesh);
          this.workcellObjects.push({
            name: item.name || `object_${idx + 1}.stl`,
            mesh: mesh,
            stl_b64: item.stl_b64,
            auto_center: !!item.auto_center
          });
        } catch (err) {
          console.warn('Failed to restore workcell object:', item.name, err);
        }
      });
    }

    this.updateWorkcellUI();
    if (this.app) this.app.updatePickableObjects();
  }

  loadWorkcellCADFromInput() {
    const fileInput = document.getElementById('cad-object-file');
    if (!fileInput || !fileInput.files || !fileInput.files[0]) {
      alert("Vui lòng chọn 1 file STL!");
      return;
    }

    const file = fileInput.files[0];
    const posX = parseFloat(document.getElementById('cad-pos-x')?.value) || 0;
    const posY = parseFloat(document.getElementById('cad-pos-y')?.value) || 0;
    const posZ = parseFloat(document.getElementById('cad-pos-z')?.value) || 0;
    const rotX = parseFloat(document.getElementById('cad-rot-x')?.value) || 0;
    const rotY = parseFloat(document.getElementById('cad-rot-y')?.value) || 0;
    const rotZ = parseFloat(document.getElementById('cad-rot-z')?.value) || 0;
    const scale = parseFloat(document.getElementById('cad-scale')?.value) || 1.0;
    const autoCenter = document.getElementById('cad-auto-center')?.checked || false;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const stlB64 = this._arrayBufferToBase64(e.target.result);
        const geometry = this.loader.parse(e.target.result);
        geometry.rotateX(-Math.PI / 2);
        geometry.computeVertexNormals();
        if (autoCenter) {
          geometry.center();
        }

        const material = new THREE.MeshStandardMaterial({
          color: 0x94a3b8,
          roughness: 0.6,
          metalness: 0.2
        });

        const mesh = new THREE.Mesh(geometry, material);
        const r = (deg) => THREE.MathUtils.degToRad(deg);
        mesh.position.set(posX, posZ, -posY);
        mesh.rotation.set(r(rotX), r(rotZ), r(-rotY));
        mesh.scale.set(scale, scale, scale);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh.name = `CAD_${file.name}`;

        const idx = this.workcellObjects.length;
        mesh.userData = { workcellIndex: idx };

        this.scene.add(mesh);
        this.workcellObjects.push({ name: file.name, mesh: mesh, stl_b64: stlB64, auto_center: autoCenter });

        this.updateWorkcellUI();
        if (this.app) this.app.updatePickableObjects();
      } catch (err) {
        alert("Lỗi đọc file STL: " + err.message);
      }
    };
    reader.readAsArrayBuffer(file);
  }

  updateWorkcellUI() {
    const list = document.getElementById('workcell-objects-list');
    if (!list) return;
    if (this.workcellObjects.length === 0) {
      list.innerHTML = "Chưa có vật cản nào được nạp.";
      return;
    }

    list.innerHTML = this.workcellObjects.map((obj, idx) => `
      <div style="display: flex; justify-content: space-between; align-items: center; padding: 3px 0; border-bottom: 1px solid var(--border);">
        <span style="overflow: hidden; text-overflow: ellipsis; white-space: nowrap; max-width: 170px;">🔹 ${obj.name}</span>
        <div style="display: flex; gap: 4px;">
          <button class="btn" style="padding: 1px 5px; font-size: 10px;" onclick="window.app.selectWorkcellObject(${idx})">🕹 Chỉnh 3D</button>
          <button class="btn btn-danger" style="padding: 1px 5px; font-size: 10px;" onclick="window.stlManager.removeWorkcellObject(${idx})">Xóa</button>
        </div>
      </div>
    `).join('');
  }

  removeWorkcellObject(index) {
    if (this.workcellObjects[index]) {
      this.scene.remove(this.workcellObjects[index].mesh);
      this.workcellObjects.splice(index, 1);
      this.workcellObjects.forEach((o, i) => { o.mesh.userData.workcellIndex = i; });
      this.updateWorkcellUI();
      if (this.app) {
        this.app.updatePickableObjects();
        this.app.deselectPart();
      }
    }
  }

  /**
   * Scans ALL Workcell objects and extracts fine world-space obstacle boxes (35mm cells from actual mesh triangles)
   * in Robotics Z-up (mm) so multi-part, tilted, or multiple separate obstacles are all accurately avoided.
   */
  getWorkcellObstaclesAABB() {
    const obstacles = [];
    const cellSize = 35.0;

    this.workcellObjects.forEach((obj) => {
      if (!obj.mesh) return;
      obj.mesh.updateMatrixWorld(true);
      const matWorld = obj.mesh.matrixWorld;
      const geom = obj.mesh.geometry;

      if (geom && geom.attributes && geom.attributes.position) {
        const posAttr = geom.attributes.position;
        const v = new THREE.Vector3();
        const totalVerts = posAttr.count;
        const stride = totalVerts > 4000 ? Math.floor(totalVerts / 2000) : 1;

        const buildCells = (stepMm) => {
          const cellMap = new Map();
          for (let idx = 0; idx < totalVerts; idx += stride) {
            v.fromBufferAttribute(posAttr, idx).applyMatrix4(matWorld);
            // Convert Three.js (X, Y_up, Z) -> Robotics (X, -Z, Y_up)
            const rx = v.x;
            const ry = -v.z;
            const rz = v.y;
            const cx = Math.floor(rx / stepMm);
            const cy = Math.floor(ry / stepMm);
            const cz = Math.floor(rz / stepMm);
            const key = `${cx},${cy},${cz}`;
            if (!cellMap.has(key)) {
              cellMap.set(key, {
                min: [cx * stepMm, cy * stepMm, cz * stepMm],
                max: [(cx + 1) * stepMm, (cy + 1) * stepMm, (cz + 1) * stepMm]
              });
            }
          }
          return Array.from(cellMap.values());
        };

        let subBoxes = buildCells(40.0);
        if (subBoxes.length > 250) {
          subBoxes = buildCells(65.0);
        }
        const box = new THREE.Box3().setFromObject(obj.mesh);
        obstacles.push({
          name: obj.name,
          min: [
            Math.round(box.min.x * 10) / 10,
            Math.round(-box.max.z * 10) / 10,
            Math.round(box.min.y * 10) / 10
          ],
          max: [
            Math.round(box.max.x * 10) / 10,
            Math.round(-box.min.z * 10) / 10,
            Math.round(box.max.y * 10) / 10
          ],
          sub_boxes: subBoxes
        });
      } else {
        const box = new THREE.Box3().setFromObject(obj.mesh);
        obstacles.push({
          name: obj.name,
          min: [
            Math.round(box.min.x * 10) / 10,
            Math.round(-box.max.z * 10) / 10,
            Math.round(box.min.y * 10) / 10
          ],
          max: [
            Math.round(box.max.x * 10) / 10,
            Math.round(-box.min.z * 10) / 10,
            Math.round(box.max.y * 10) / 10
          ]
        });
      }
    });
    return obstacles;
  }
}
