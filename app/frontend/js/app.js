/**
 * Main Controller (PAROL6-python-API/app)
 * Features:
 * 1. 3 Workspace Environments:
 *    - ws-assembly: Gắn STL & Dựng Model Robot
 *    - ws-dh: Bảng Thông Số Động Học DH & TCP
 *    - ws-control: Điều Khiển FK/IK & Lập Trình Quỹ Đạo OLP
 * 2. 3D Click-to-Select Part Inspector:
 *    - Click any Part in 3D View -> Shows that Part's Own Origin Frame (Gốc 2) + Floating XYZ/Rotation Inspector
 *    - Fixed Reference Origin (Gốc 1) always visible at (0,0,0)
 * 3. Standard Desktop File Menu & Toolbar (New, Open, Save, Save As, Import STL, Export G-code)
 */
class AppController {
  constructor() {
    this.viewport = new Viewport3D('viewport-canvas');
    this.robot = new RobotModel(this.viewport.scene);

    this.activeWorkspace = 'ws-assembly';
    this.selectedTargetType = null; // 'robot_link' | 'workcell' | null
    this.selectedPartIndex = -1;

    this.stlManager = new STLManager(this.robot, this.viewport.scene, this);
    window.stlManager = this.stlManager;

    this.serialUI = new SerialUI();
    window.serialUI = this.serialUI;

    this.trajectoryManager = new TrajectoryManager(this.robot, this);
    window.trajectoryManager = this.trajectoryManager;

    // 3D Raycaster Click Callback
    this.viewport.onPartSelected3D = (selection) => this.handle3DSelection(selection);
    this.viewport.onGizmoChange = (object) => this.onGizmoManipulated(object);
    this.updatePickableObjects();

    this.setupWebSocket();
    this.setupWorkspaceSwitcher();
    this.setupTopMenuAndToolbar();
    this.setupPartInspectorEvents();
    this.setupResizablePanels();
    this.setupCollapsibleCards();
    this.setupFKSliders();
    this.setupIKInputs();
    this.setupDHEditorControls();
    this.setupTCPControls();
    this.setupViewportTools();

    this.renderPartTreeList();
    this.renderDHEditorTable();
    this.syncFromJoints([0, 0, 0, 0, 0, 0]);

    // Auto-load active Robot Arm Model from backend
    this.stlManager.loadSavedProfileFromServer();
  }

  updatePickableObjects() {
    const objs = [...this.robot.meshHolders];
    if (this.stlManager && this.stlManager.workcellObjects) {
      this.stlManager.workcellObjects.forEach(w => {
        if (w.mesh) objs.push(w.mesh);
      });
    }
    this.viewport.setPickableObjects(objs);
  }

  /* --------- 1. Workspace Environment Switcher --------- */
  setupWorkspaceSwitcher() {
    const wsTabs = document.querySelectorAll('.ws-tab');
    wsTabs.forEach(tab => {
      tab.onclick = () => {
        const wsId = tab.getAttribute('data-ws');
        this.switchWorkspace(wsId);
      };
    });
  }

  switchWorkspace(wsId) {
    this.activeWorkspace = wsId;
    document.querySelectorAll('.ws-tab').forEach(t => {
      t.classList.toggle('active', t.getAttribute('data-ws') === wsId);
    });

    document.getElementById('ws-pane-assembly').style.display = (wsId === 'ws-assembly') ? 'block' : 'none';
    document.getElementById('ws-pane-dh').style.display = (wsId === 'ws-dh') ? 'block' : 'none';
    document.getElementById('ws-pane-control').style.display = (wsId === 'ws-control') ? 'block' : 'none';

    const leftPanel = document.getElementById('left-panel');
    const rightPanel = document.getElementById('right-panel');
    const resizerRight = document.getElementById('resizer-right');
    const btnToggleRight = document.getElementById('btn-toggle-right');

    if (wsId === 'ws-assembly') {
      leftPanel.style.width = '330px';
      rightPanel.style.display = 'none';
      resizerRight.style.display = 'none';
      if (btnToggleRight) btnToggleRight.style.display = 'none';
      this.robot.setDHJointAxesVisible(false);
      if (this.selectedPartIndex < 0) {
        this.viewport.detachGizmo();
      }
      this.updateHUDStatus('Môi trường Gắn STL: Nhấp vào Part bất kỳ trên 3D để hiện Gốc riêng & chỉnh X,Y,Z', 'normal');
    } else if (wsId === 'ws-dh') {
      leftPanel.style.width = '430px';
      rightPanel.style.display = 'none';
      resizerRight.style.display = 'none';
      if (btnToggleRight) btnToggleRight.style.display = 'none';
      this.deselectPart();
      this.viewport.detachGizmo();
      this.updateHUDStatus('Môi trường Bảng DH & TCP: Chỉnh sửa thông số động học các khâu', 'normal');
    } else if (wsId === 'ws-control') {
      leftPanel.style.width = '320px';
      rightPanel.style.display = 'flex';
      resizerRight.style.display = 'flex';
      if (btnToggleRight) btnToggleRight.style.display = 'flex';
      this.robot.setDHJointAxesVisible(false);
      this.deselectPart();
      this.viewport.attachGizmoTo(this.robot.tcpTarget);
      this.updateHUDStatus('Môi trường Điều Khiển & Lập Trình: Kéo mũi TCP hoặc trượt góc khớp J1-J6', 'normal');
    }

    setTimeout(() => this.viewport.onResize(), 20);
  }

  /* --------- 2. Standard Desktop Menu Bar & Quick Toolbar --------- */
  setupTopMenuAndToolbar() {
    const inpProfile = document.getElementById('input-import-profile');
    if (inpProfile) {
      inpProfile.onchange = async (e) => {
        const file = e.target.files?.[0];
        if (!file) return;
        await this.stlManager.importModelBundleFile(file);
        inpProfile.value = '';
      };
    }

    const triggerOpenFile = () => inpProfile?.click();
    const triggerSaveCurrent = async () => {
      const nameVal = document.getElementById('assembly-model-name')?.value?.trim() || this.stlManager.activeModelName;
      const ok = await this.stlManager.saveFullRobotModel(nameVal, false);
      if (ok) {
        this.updateHUDStatus(`💾 Đã lưu thành công Model "${this.stlManager.activeModelName}"`, 'normal');
      }
    };
    const triggerSaveAs = async () => {
      const defaultName = (document.getElementById('assembly-model-name')?.value?.trim() || this.stlManager.activeModelName) + ' (Bản sao)';
      const newName = prompt('Nhập tên cho Model Cánh Tay mới:', defaultName);
      if (!newName || !newName.trim()) return;
      const ok = await this.stlManager.saveFullRobotModel(newName.trim(), true);
      if (ok) {
        alert(`✅ Đã lưu thành công Model Cánh Tay mới: "${newName.trim()}"!`);
      }
    };

    // Menu items
    document.getElementById('menu-new-model').onclick = () => this.stlManager.createNewRobotModel();
    document.getElementById('menu-open-bundle').onclick = triggerOpenFile;
    document.getElementById('menu-save-model').onclick = triggerSaveCurrent;
    document.getElementById('menu-save-as-model').onclick = triggerSaveAs;
    document.getElementById('menu-export-bundle').onclick = () => this.stlManager.exportCurrentModelBundle();
    document.getElementById('menu-import-part-stl').onclick = () => {
      const targetIdx = (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) ? this.selectedPartIndex : 0;
      this.stlManager.promptImportSTLForPart(targetIdx);
    };
    document.getElementById('menu-export-gcode').onclick = () => {
      document.getElementById('btn-export-gcode')?.click();
    };

    // Quick toolbar buttons
    document.getElementById('btn-quick-open').onclick = triggerOpenFile;
    document.getElementById('btn-quick-save').onclick = triggerSaveCurrent;
    document.getElementById('btn-quick-save-as').onclick = triggerSaveAs;

    // Assembly sidebar buttons
    document.getElementById('btn-assembly-save').onclick = triggerSaveCurrent;
    document.getElementById('btn-assembly-save-as').onclick = triggerSaveAs;
    document.getElementById('btn-assembly-export').onclick = () => this.stlManager.exportCurrentModelBundle();
    document.getElementById('btn-assembly-delete').onclick = async () => {
      const msg = this.stlManager.activeModelId === 'parol6_default'
        ? 'Khôi phục PAROL6 Mặc Định về hình khối cơ bản và xóa các STL đã gắn?'
        : `Xóa Model Cánh Tay "${this.stlManager.activeModelName}"?`;
      if (!confirm(msg)) return;
      await this.stlManager.deleteOrResetModel(this.stlManager.activeModelId);
    };

    // Keyboard shortcut Ctrl+S to save model
    window.addEventListener('keydown', (e) => {
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 's') {
        e.preventDefault();
        triggerSaveCurrent();
      } else if (e.key === 'Escape') {
        this.deselectPart();
      }
    });
  }

  /* --------- 3. 3D Part Selection, Part Tree & Floating Part Inspector --------- */
  renderPartTreeList() {
    const container = document.getElementById('part-tree-list');
    if (!container) return;
    container.innerHTML = '';

    for (let i = 0; i <= 6; i++) {
      const off = this.robot.linkOffsets[i] || {};
      const stlName = off.stl_file;
      const isSelected = (this.selectedTargetType === 'robot_link' && this.selectedPartIndex === i);

      const item = document.createElement('div');
      item.className = `part-tree-item ${isSelected ? 'selected' : ''}`;
      item.innerHTML = `
        <div class="part-tree-info">
          <span class="part-tree-name">${off.name || 'Part ' + i}</span>
          <span class="part-tree-stl ${stlName ? 'has-stl' : ''}">
            ${stlName ? '✅ STL: ' + stlName : '▫ Khối 3D mặc định'}
          </span>
        </div>
        <button class="btn" style="padding: 3px 7px; font-size: 10px;" title="Chọn file .stl gắn vào Part này">
          📥 Gắn STL
        </button>
      `;

      // Clicking the row selects this Part in 3D and opens the Inspector
      item.onclick = (e) => {
        if (e.target.tagName === 'BUTTON') return;
        this.selectRobotPart(i);
      };

      // Clicking "📥 Gắn STL" opens file dialog for this Part
      const btnImport = item.querySelector('button');
      btnImport.onclick = (e) => {
        e.stopPropagation();
        this.selectRobotPart(i);
        this.stlManager.promptImportSTLForPart(i);
      };

      container.appendChild(item);
    }
  }

  handle3DSelection(selection) {
    if (!selection) {
      this.deselectPart();
      return;
    }
    if (selection.type === 'robot_link') {
      this.selectRobotPart(selection.index);
    } else if (selection.type === 'workcell') {
      this.selectWorkcellObject(selection.index);
    }
  }

  selectRobotPart(linkIndex) {
    this.selectedTargetType = 'robot_link';
    this.selectedPartIndex = linkIndex;

    // 1. Highlight Part & Show ONLY this Part's Own Origin Frame (Gốc 2)
    this.robot.selectPart(linkIndex);

    // 2. Attach 3D Gizmo directly to this Part's holder so user can drag/rotate it
    const holder = this.robot.meshHolders[linkIndex];
    if (holder) {
      this.viewport.attachGizmoTo(holder);
    }

    // 3. Open & populate Floating Part Inspector
    this.populateInspectorForPart(linkIndex);
    this.renderPartTreeList();
  }

  selectWorkcellObject(wcIndex) {
    const obj = this.stlManager.workcellObjects[wcIndex];
    if (!obj || !obj.mesh) return;

    this.selectedTargetType = 'workcell';
    this.selectedPartIndex = wcIndex;
    this.robot.selectPart(-1);
    this.viewport.attachGizmoTo(obj.mesh);

    const box = document.getElementById('part-inspector-box');
    if (box) box.style.display = 'block';

    document.getElementById('insp-part-badge').innerText = 'VẬT THỂ WORKCELL';
    document.getElementById('insp-part-title').innerText = obj.name;
    document.getElementById('insp-stl-filename').innerText = `📦 File: ${obj.name}`;

    const deg = (r) => Math.round(THREE.MathUtils.radToDeg(r) * 10) / 10;
    const rnd = (v) => Math.round(v * 10) / 10;

    document.getElementById('insp-pos-x').value = rnd(obj.mesh.position.x);
    document.getElementById('insp-pos-y').value = rnd(obj.mesh.position.z);
    document.getElementById('insp-pos-z').value = rnd(obj.mesh.position.y);
    document.getElementById('insp-rot-x').value = deg(obj.mesh.rotation.x);
    document.getElementById('insp-rot-y').value = deg(obj.mesh.rotation.z);
    document.getElementById('insp-rot-z').value = deg(obj.mesh.rotation.y);
    document.getElementById('insp-scale').value = rnd(obj.mesh.scale.x);

    this.renderPartTreeList();
  }

  deselectPart() {
    this.selectedTargetType = null;
    this.selectedPartIndex = -1;
    this.robot.selectPart(-1); // Hides all Parts' Own Origin Frames (Gốc 2)

    const box = document.getElementById('part-inspector-box');
    if (box) box.style.display = 'none';

    if (this.activeWorkspace === 'ws-control') {
      this.viewport.attachGizmoTo(this.robot.tcpTarget);
    } else {
      this.viewport.detachGizmo();
    }
    this.renderPartTreeList();
  }

  populateInspectorForPart(linkIndex) {
    const off = this.robot.linkOffsets[linkIndex];
    if (!off) return;

    const box = document.getElementById('part-inspector-box');
    if (box) box.style.display = 'block';

    document.getElementById('insp-part-badge').innerText = '🟡 GỐC RIÊNG CỦA PART (GỐC 2)';
    document.getElementById('insp-part-title').innerText = off.name || `Part ${linkIndex}`;

    const bounds = this.robot.stlBounds[linkIndex];
    const dimStr = bounds ? ` (${bounds.width}×${bounds.depth}×${bounds.height} mm)` : '';
    const fileLabel = document.getElementById('insp-stl-filename');
    if (off.stl_file) {
      fileLabel.innerText = `✅ ${off.stl_file}${dimStr}`;
      fileLabel.style.color = 'var(--success)';
    } else {
      fileLabel.innerText = '▫ Đang dùng hình khối 3D mặc định';
      fileLabel.style.color = 'var(--text-dim)';
    }

    // Luôn lấy TỌA ĐỘ THỰC (X, Y, Z, Rx, Ry, Rz) tính trực tiếp từ Gốc Cố Định (0,0,0)!
    const truePose = this.robot.getPartTrueWorldPose(linkIndex);
    document.getElementById('insp-pos-x').value = truePose.x;
    document.getElementById('insp-pos-y').value = truePose.y;
    document.getElementById('insp-pos-z').value = truePose.z;
    document.getElementById('insp-rot-x').value = truePose.rx;
    document.getElementById('insp-rot-y').value = truePose.ry;
    document.getElementById('insp-rot-z').value = truePose.rz;
    document.getElementById('insp-scale').value = truePose.scale;

    const alignSelect = document.getElementById('insp-align-mode');
    if (alignSelect) {
      alignSelect.value = off.align_mode || 'bottom';
    }
  }

  setupPartInspectorEvents() {
    document.getElementById('btn-close-inspector').onclick = () => this.deselectPart();

    document.getElementById('btn-insp-import-stl').onclick = () => {
      if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
        this.stlManager.promptImportSTLForPart(this.selectedPartIndex);
      }
    };

    document.getElementById('btn-insp-reset-part').onclick = async () => {
      if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
        const i = this.selectedPartIndex;
        if (!confirm(`Gỡ file STL của Part ${i} và quay về khối mặc định?`)) return;
        this.robot.resetSingleLinkToDefault(i);
        this.populateInspectorForPart(i);
        this.renderPartTreeList();
        await this.saveOffsetsToServer(false);
      }
    };

    const applyInspectorInputsLive = () => {
      const px = parseFloat(document.getElementById('insp-pos-x').value) || 0;
      const py = parseFloat(document.getElementById('insp-pos-y').value) || 0;
      const pz = parseFloat(document.getElementById('insp-pos-z').value) || 0;
      const rx = parseFloat(document.getElementById('insp-rot-x').value) || 0;
      const ry = parseFloat(document.getElementById('insp-rot-y').value) || 0;
      const rz = parseFloat(document.getElementById('insp-rot-z').value) || 0;
      const sc = parseFloat(document.getElementById('insp-scale').value) || 1.0;

      if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
        const i = this.selectedPartIndex;
        this.robot.setPartTrueWorldPose(i, {
          x: px, y: py, z: pz,
          rx: rx, ry: ry, rz: rz,
          scale: sc
        });
      } else if (this.selectedTargetType === 'workcell' && this.selectedPartIndex >= 0) {
        const obj = this.stlManager.workcellObjects[this.selectedPartIndex];
        if (obj && obj.mesh) {
          const r = (d) => THREE.MathUtils.degToRad(d);
          obj.mesh.position.set(px, pz, py);
          obj.mesh.rotation.set(r(rx), r(rz), r(ry));
          obj.mesh.scale.set(sc, sc, sc);
        }
      }
    };

    ['insp-pos-x', 'insp-pos-y', 'insp-pos-z', 'insp-rot-x', 'insp-rot-y', 'insp-rot-z', 'insp-scale'].forEach(id => {
      const el = document.getElementById(id);
      if (el) {
        el.addEventListener('input', applyInspectorInputsLive);
      }
    });

    const alignSelect = document.getElementById('insp-align-mode');
    if (alignSelect) {
      alignSelect.addEventListener('change', (e) => {
        if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
          this.robot.setPartAlignMode(this.selectedPartIndex, e.target.value);
        }
      });
    }

    document.getElementById('btn-insp-snap-origin').onclick = () => {
      document.getElementById('insp-pos-x').value = 0;
      document.getElementById('insp-pos-y').value = 0;
      document.getElementById('insp-pos-z').value = 0;
      document.getElementById('insp-rot-x').value = 0;
      document.getElementById('insp-rot-y').value = 0;
      document.getElementById('insp-rot-z').value = 0;
      applyInspectorInputsLive();
    };

    const btnSnapDh = document.getElementById('btn-insp-snap-dh');
    if (btnSnapDh) {
      btnSnapDh.onclick = () => {
        if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
          this.robot.snapPartToDHJoint(this.selectedPartIndex);
          this.populateInspectorForPart(this.selectedPartIndex);
        }
      };
    }

    document.getElementById('btn-insp-save-part').onclick = async () => {
      applyInspectorInputsLive();
      await this.saveOffsetsToServer(true);
    };
  }

  async saveOffsetsToServer(showNotice = true) {
    try {
      await fetch('/api/dh_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          dh_table: this.robot.dhTable,
          link_offsets: this.robot.linkOffsets,
          tcp_offset: this.robot.tcpOffset,
          save_to_disk: true
        })
      });
      if (showNotice) {
        this.updateHUDStatus('💾 Đã lưu vị trí & góc quay của Part vào Model hiện tại', 'normal');
      }
    } catch (err) {
      console.error('Failed to save offsets:', err);
    }
  }

  /* --------- WebSocket & Kinematics --------- */
  setupWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      this.updateHUDStatus("Đã kết nối Python API - Nhấp vào Part bất kỳ trên 3D để chỉnh X,Y,Z", "normal");
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'ik_result') {
          this.handleIKResult(msg.data);
        } else if (msg.type === 'fk_result') {
          this.handleFKResult(msg.data);
        } else if (msg.type === 'serial_rx') {
          this.serialUI.logMessage('RX', msg.data);
        }
      } catch (err) {
        console.error("WS parse error:", err);
      }
    };

    this.ws.onclose = () => {
      setTimeout(() => this.setupWebSocket(), 3000);
    };
  }

  /* --------- Resizable Splitters & Collapsible Panels --------- */
  setupResizablePanels() {
    const leftPanel = document.getElementById('left-panel');
    const rightPanel = document.getElementById('right-panel');
    const resizerLeft = document.getElementById('resizer-left');
    const resizerRight = document.getElementById('resizer-right');
    const btnToggleLeft = document.getElementById('btn-toggle-left');
    const btnToggleRight = document.getElementById('btn-toggle-right');

    const updateToggleIcons = () => {
      if (btnToggleLeft) {
        btnToggleLeft.innerText = leftPanel.classList.contains('collapsed') ? '▶' : '◀';
      }
      if (btnToggleRight) {
        btnToggleRight.innerText = rightPanel.classList.contains('collapsed') ? '◀' : '▶';
      }
      this.viewport.onResize();
    };

    const toggleLeft = () => {
      leftPanel.classList.toggle('collapsed');
      updateToggleIcons();
    };

    const toggleRight = () => {
      rightPanel.classList.toggle('collapsed');
      updateToggleIcons();
    };

    if (btnToggleLeft) btnToggleLeft.onclick = toggleLeft;
    if (btnToggleRight) btnToggleRight.onclick = toggleRight;
    if (resizerLeft) resizerLeft.ondblclick = toggleLeft;
    if (resizerRight) resizerRight.ondblclick = toggleRight;

    if (resizerLeft) {
      resizerLeft.addEventListener('mousedown', (e) => {
        e.preventDefault();
        resizerLeft.classList.add('dragging');
        const onMove = (ev) => {
          const newW = ev.clientX;
          if (newW < 90) {
            leftPanel.classList.add('collapsed');
          } else {
            leftPanel.classList.remove('collapsed');
            const clamped = Math.min(Math.max(newW, 180), 680);
            leftPanel.style.width = `${clamped}px`;
          }
          updateToggleIcons();
        };
        const onUp = () => {
          resizerLeft.classList.remove('dragging');
          window.removeEventListener('mousemove', onMove);
          window.removeEventListener('mouseup', onUp);
          this.viewport.onResize();
        };
        window.addEventListener('mousemove', onMove);
        window.addEventListener('mouseup', onUp);
      });
    }

    if (resizerRight) {
      resizerRight.addEventListener('mousedown', (e) => {
        e.preventDefault();
        resizerRight.classList.add('dragging');
        const onMove = (ev) => {
          const newW = window.innerWidth - ev.clientX;
          if (newW < 90) {
            rightPanel.classList.add('collapsed');
          } else {
            rightPanel.classList.remove('collapsed');
            const clamped = Math.min(Math.max(newW, 200), 680);
            rightPanel.style.width = `${clamped}px`;
          }
          updateToggleIcons();
        };
        const onUp = () => {
          resizerRight.classList.remove('dragging');
          window.removeEventListener('mousemove', onMove);
          window.removeEventListener('mouseup', onUp);
          this.viewport.onResize();
        };
        window.addEventListener('mousemove', onMove);
        window.addEventListener('mouseup', onUp);
      });
    }
  }

  setupCollapsibleCards() {
    document.querySelectorAll('.collapsible-header').forEach(header => {
      header.addEventListener('click', (e) => {
        if (e.target.tagName === 'BUTTON' || e.target.tagName === 'INPUT' || e.target.tagName === 'LABEL') return;
        const card = header.closest('.section-card');
        if (!card) return;
        card.classList.toggle('card-collapsed');
        const span = header.querySelector('span:first-child');
        if (span) {
          const txt = span.innerText.replace(/^[▼▶]\s*/, '');
          span.innerText = `${card.classList.contains('card-collapsed') ? '▶' : '▼'} ${txt}`;
        }
      });
    });
  }

  /* --------- Workspace 2: DH Table & TCP Editor --------- */
  renderDHEditorTable() {
    const tbody = document.getElementById('dh-table-body');
    if (!tbody) return;
    tbody.innerHTML = '';

    this.robot.dhTable.forEach((row, i) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td style="font-weight: 700; color: #38bdf8;">J${i + 1}</td>
        <td><input type="number" class="dh-input" id="dh-a-${i}" value="${row.a}" step="1"></td>
        <td><input type="number" class="dh-input" id="dh-alpha-${i}" value="${row.alpha_deg}" step="1"></td>
        <td><input type="number" class="dh-input" id="dh-d-${i}" value="${row.d}" step="1"></td>
        <td><input type="number" class="dh-input" id="dh-off-${i}" value="${row.offset_deg}" step="1"></td>
        <td><input type="number" class="dh-input" id="dh-min-${i}" value="${row.min_deg}" step="5"></td>
        <td><input type="number" class="dh-input" id="dh-max-${i}" value="${row.max_deg}" step="5"></td>
      `;
      tbody.appendChild(tr);

      tr.querySelectorAll('input').forEach(inp => {
        inp.addEventListener('input', () => {
          this.applyDHFromTableUI(false);
        });
      });
    });
  }

  collectDHFromTableUI() {
    return this.robot.dhTable.map((row, i) => ({
      ...row,
      a: parseFloat(document.getElementById(`dh-a-${i}`)?.value) || 0,
      alpha_deg: parseFloat(document.getElementById(`dh-alpha-${i}`)?.value) || 0,
      d: parseFloat(document.getElementById(`dh-d-${i}`)?.value) || 0,
      offset_deg: parseFloat(document.getElementById(`dh-off-${i}`)?.value) || 0,
      min_deg: parseFloat(document.getElementById(`dh-min-${i}`)?.value ?? row.min_deg),
      max_deg: parseFloat(document.getElementById(`dh-max-${i}`)?.value ?? row.max_deg)
    }));
  }

  async applyDHFromTableUI(saveToDisk = true) {
    const newDh = this.collectDHFromTableUI();
    this.robot.updateDHTable(newDh);
    this.setupFKSliders();
    this.syncFromJoints(this.robot.joints);

    try {
      await fetch('/api/dh_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          dh_table: this.robot.dhTable,
          link_offsets: this.robot.linkOffsets,
          tcp_offset: this.robot.tcpOffset,
          save_to_disk: saveToDisk
        })
      });
      if (saveToDisk) {
        this.updateHUDStatus('💾 Đã lưu Bảng thông số DH', 'normal');
      }
    } catch (err) {
      console.error('Failed to update DH config:', err);
    }
  }

  setupDHEditorControls() {
    const chkDhAxes = document.getElementById('chk-show-dh-axes');
    if (chkDhAxes) {
      chkDhAxes.onchange = (e) => {
        this.robot.setDHJointAxesVisible(e.target.checked);
      };
    }

    const btnSaveDh = document.getElementById('btn-save-dh');
    if (btnSaveDh) {
      btnSaveDh.onclick = async () => {
        await this.applyDHFromTableUI(true);
        alert('✅ Đã lưu Bảng thông số DH vào Model hiện tại!');
      };
    }

    const btnResetDh = document.getElementById('btn-reset-dh');
    if (btnResetDh) {
      btnResetDh.onclick = async () => {
        if (!confirm('Khôi phục bảng DH về thông số chuẩn của PAROL6?')) return;
        const res = await fetch('/api/dh_reset', { method: 'POST' });
        const data = await res.json();
        if (data.success) {
          this.robot.updateDHTable(data.dh_table);
          this.renderDHEditorTable();
          this.setupFKSliders();
          this.syncFromJoints(this.robot.joints);
          this.updateHUDStatus('Đã khôi phục DH mặc định', 'normal');
        }
      };
    }
  }

  /* --------- Workspace 3: FK / IK / TCP Controls --------- */
  setupFKSliders() {
    const container = document.getElementById('fk-sliders-container');
    if (!container) return;
    container.innerHTML = '';

    for (let i = 0; i < 6; i++) {
      const limits = this.robot.jointLimits[i] || [-180, 180];
      const curVal = this.robot.joints[i] || 0;
      const row = document.createElement('div');
      row.className = 'joint-row';
      row.innerHTML = `
        <div class="joint-header">
          <span>Khớp J${i + 1}</span>
          <span style="color: var(--text-dim); font-size: 11px;">[${limits[0]}° .. ${limits[1]}°]</span>
        </div>
        <div class="slider-container">
          <input type="range" id="slider-j${i}" min="${limits[0]}" max="${limits[1]}" step="0.5" value="${curVal}">
          <input type="number" id="num-j${i}" class="num-input" min="${limits[0]}" max="${limits[1]}" step="0.5" value="${curVal}">
          <span style="font-size: 11px; color: var(--text-dim);">°</span>
        </div>
      `;
      container.appendChild(row);

      const slider = row.querySelector(`#slider-j${i}`);
      const num = row.querySelector(`#num-j${i}`);

      const updateJoint = (val) => {
        const floatVal = parseFloat(val) || 0;
        slider.value = floatVal;
        num.value = floatVal;
        const newJoints = [...this.robot.joints];
        newJoints[i] = floatVal;
        this.syncFromJoints(newJoints);
      };

      slider.addEventListener('input', (e) => updateJoint(e.target.value));
      num.addEventListener('change', (e) => updateJoint(e.target.value));
    }

    document.getElementById('btn-home-robot').onclick = () => {
      this.setJointsFromExternal([0, 0, 0, 0, 0, 0]);
    };
  }

  setupIKInputs() {
    document.getElementById('btn-solve-ik').onclick = () => {
      const x = parseFloat(document.getElementById('ik-x').value) || 0;
      const y = parseFloat(document.getElementById('ik-y').value) || 0;
      const z = parseFloat(document.getElementById('ik-z').value) || 0;
      const roll = parseFloat(document.getElementById('ik-roll').value) || 0;
      const pitch = parseFloat(document.getElementById('ik-pitch').value) || 0;
      const yaw = parseFloat(document.getElementById('ik-yaw').value) || 0;

      this.requestIK({ x, y, z, roll, pitch, yaw });
    };
  }

  syncTCPInputsFromModel() {
    const t = this.robot.tcpOffset;
    if (!t) return;
    document.getElementById('tcp-off-x').value = t.x ?? 0;
    document.getElementById('tcp-off-y').value = t.y ?? 0;
    document.getElementById('tcp-off-z').value = t.z ?? 65;
    document.getElementById('tcp-off-roll').value = t.roll ?? 0;
    document.getElementById('tcp-off-pitch').value = t.pitch ?? 0;
    document.getElementById('tcp-off-yaw').value = t.yaw ?? 0;
  }

  setupTCPControls() {
    document.getElementById('btn-apply-tcp').onclick = async () => {
      const offset = {
        x: parseFloat(document.getElementById('tcp-off-x').value) || 0,
        y: parseFloat(document.getElementById('tcp-off-y').value) || 0,
        z: parseFloat(document.getElementById('tcp-off-z').value) || 0,
        roll: parseFloat(document.getElementById('tcp-off-roll').value) || 0,
        pitch: parseFloat(document.getElementById('tcp-off-pitch').value) || 0,
        yaw: parseFloat(document.getElementById('tcp-off-yaw').value) || 0
      };
      this.robot.setTCPOffset(offset);
      this.syncFromJoints(this.robot.joints);
      await this.applyDHFromTableUI(true);
      alert('✅ Đã lưu cấu hình TCP Offset!');
    };
  }

  setupViewportTools() {
    const btnTrans = document.getElementById('btn-gizmo-translate');
    const btnRot = document.getElementById('btn-gizmo-rotate');
    const btnHide = document.getElementById('btn-gizmo-hide');

    btnTrans.onclick = () => {
      this.viewport.setGizmoMode('translate');
      btnTrans.classList.add('active');
      btnRot.classList.remove('active');
      btnHide.classList.remove('active');
    };

    btnRot.onclick = () => {
      this.viewport.setGizmoMode('rotate');
      btnRot.classList.add('active');
      btnTrans.classList.remove('active');
      btnHide.classList.remove('active');
    };

    btnHide.onclick = () => {
      this.viewport.setGizmoMode('hide');
      btnHide.classList.add('active');
      btnTrans.classList.remove('active');
      btnRot.classList.remove('active');
    };

    document.getElementById('btn-cam-iso').onclick = () => this.viewport.setCameraView('iso');
    document.getElementById('btn-cam-top').onclick = () => this.viewport.setCameraView('top');
    document.getElementById('btn-cam-front').onclick = () => this.viewport.setCameraView('front');
    const btnRight = document.getElementById('btn-cam-right');
    if (btnRight) btnRight.onclick = () => this.viewport.setCameraView('right');
  }

  syncFromJoints(newJoints) {
    this.robot.setJointAngles(newJoints);

    for (let i = 0; i < 6; i++) {
      const slider = document.getElementById(`slider-j${i}`);
      const num = document.getElementById(`num-j${i}`);
      if (slider) slider.value = newJoints[i];
      if (num) num.value = newJoints[i];
    }

    const pose = this.robot.getTCPWorldPose();
    document.getElementById('ik-x').value = pose.x;
    document.getElementById('ik-y').value = pose.y;
    document.getElementById('ik-z').value = pose.z;
    document.getElementById('ik-roll').value = pose.roll;
    document.getElementById('ik-pitch').value = pose.pitch;
    document.getElementById('ik-yaw').value = pose.yaw;

    document.getElementById('hud-tcp-coords').innerText = `TCP: X: ${pose.x} Y: ${pose.y} Z: ${pose.z}`;
  }

  setJointsFromExternal(joints) {
    this.syncFromJoints(joints);
  }

  onGizmoManipulated(targetObject) {
    // If a Robot Part is selected and dragged with the 3D Gizmo, update its X,Y,Z,Rx,Ry,Rz inputs live!
    if (this.selectedTargetType === 'robot_link' && this.selectedPartIndex >= 0) {
      const updated = this.robot.readBackOffsetFromHolder(this.selectedPartIndex);
      if (updated) {
        document.getElementById('insp-pos-x').value = updated.x ?? updated.dx;
        document.getElementById('insp-pos-y').value = updated.y ?? updated.dy;
        document.getElementById('insp-pos-z').value = updated.z ?? updated.dz;
        document.getElementById('insp-rot-x').value = updated.rx;
        document.getElementById('insp-rot-y').value = updated.ry;
        document.getElementById('insp-rot-z').value = updated.rz;
      }
      return;
    }

    // If a Workcell Object is selected and dragged with the 3D Gizmo, update its inputs live!
    if (this.selectedTargetType === 'workcell' && this.selectedPartIndex >= 0) {
      const obj = this.stlManager.workcellObjects[this.selectedPartIndex];
      if (obj && obj.mesh) {
        const deg = (r) => Math.round(THREE.MathUtils.radToDeg(r) * 10) / 10;
        const rnd = (v) => Math.round(v * 10) / 10;
        document.getElementById('insp-pos-x').value = rnd(obj.mesh.position.x);
        document.getElementById('insp-pos-y').value = rnd(obj.mesh.position.z);
        document.getElementById('insp-pos-z').value = rnd(obj.mesh.position.y);
        document.getElementById('insp-rot-x').value = deg(obj.mesh.rotation.x);
        document.getElementById('insp-rot-y').value = deg(obj.mesh.rotation.z);
        document.getElementById('insp-rot-z').value = deg(obj.mesh.rotation.y);
      }
      return;
    }

    // Otherwise, TCP IK manipulation
    const pose = this.robot.getTCPWorldPose();
    this.requestIK(pose);
  }

  requestIK(pose) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const tcpOff = [
        this.robot.tcpOffset.x, this.robot.tcpOffset.y, this.robot.tcpOffset.z,
        this.robot.tcpOffset.roll, this.robot.tcpOffset.pitch, this.robot.tcpOffset.yaw
      ];
      this.ws.send(JSON.stringify({
        type: 'ik',
        pose: pose,
        current_joints: this.robot.joints,
        tcp_offset: tcpOff
      }));
    }
  }

  handleIKResult(result) {
    const alertEl = document.getElementById('singularity-alert');
    if (result.success) {
      this.syncFromJoints(result.joint_angles);
      if (result.singularities && result.singularities.has_singularity) {
        let msg = "⚠️ Cảnh báo kỳ dị: ";
        if (result.singularities.wrist_singularity) msg += "Cổ tay (J5 ≈ 0°) ";
        if (result.singularities.shoulder_singularity) msg += "Trục vai ";
        alertEl.style.color = "var(--warning)";
        alertEl.innerText = msg;
        this.updateHUDStatus(msg, "warning");
      } else {
        alertEl.style.color = "var(--success)";
        alertEl.innerText = "✅ Trạng thái động học bình thường";
        this.updateHUDStatus("PAROL6 Sẵn Sàng", "normal");
      }
    } else {
      alertEl.style.color = "var(--danger)";
      alertEl.innerText = "⛔ Vượt tầm với hoặc vi phạm giới hạn khớp!";
      this.updateHUDStatus("Vượt Tầm Với (Out of Reach)", "danger");
    }
  }

  handleFKResult(result) {}

  updateHUDStatus(text, statusType) {
    document.getElementById('hud-status-text').innerText = text;
    const dot = document.getElementById('hud-status-dot');
    dot.className = 'status-dot';
    if (statusType === 'danger') dot.classList.add('danger');
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.app = new AppController();
});
