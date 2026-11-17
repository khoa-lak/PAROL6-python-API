/**
 * PAROL6 Robot 3D Model & Dynamic DH Kinematic Hierarchy (PAROL6-python-API/app)
 * Features:
 * - Gốc 1 (Fixed Reference Origin 0,0,0): Cố định tại tâm bàn máy (0,0,0)
 * - Gốc 2 (Part's Own Origin - Tâm Vàng): Gắn trực tiếp vào từng Part, chỉ hiện khi click chọn Part đó.
 * - Tọa độ (X, Y, Z) và Góc quay (Rx, Ry, Rz) của mọi Part hiển thị và điều chỉnh theo TỌA ĐỘ THỰC so với Gốc Cố Định (0,0,0):
 *   Khi Z = 0 thì Part nằm đúng tại mặt phẳng Z = 0 của Gốc Cố Định!
 */
class RobotModel {
  constructor(scene) {
    this.scene = scene;
    this.joints = [0, 0, 0, 0, 0, 0];
    this.tcpOffset = { x: 0, y: 0, z: 65, roll: 0, pitch: 0, yaw: 0 };
    this.selectedLinkIndex = -1;
    this.showDHJointFrames = false;

    this.dhTable = [
      { joint: 1, name: "J1 (Shoulder Base)", a: 0.0,   alpha_deg: 90.0,  d: 118.0, offset_deg: 0.0,  min_deg: -170.0, max_deg: 170.0 },
      { joint: 2, name: "J2 (Upper Arm)",     a: 145.0, alpha_deg: 0.0,   d: 0.0,   offset_deg: 90.0, min_deg: -100.0, max_deg: 90.0 },
      { joint: 3, name: "J3 (Elbow)",         a: 0.0,   alpha_deg: 90.0,  d: 0.0,   offset_deg: 0.0,  min_deg: -150.0, max_deg: 130.0 },
      { joint: 4, name: "J4 (Forearm)",       a: 0.0,   alpha_deg: -90.0, d: 175.0, offset_deg: 0.0,  min_deg: -180.0, max_deg: 180.0 },
      { joint: 5, name: "J5 (Wrist)",         a: 0.0,   alpha_deg: 90.0,  d: 0.0,   offset_deg: 0.0,  min_deg: -105.0, max_deg: 105.0 },
      { joint: 6, name: "J6 (Flange)",        a: 0.0,   alpha_deg: 0.0,   d: 70.0,  offset_deg: 0.0,  min_deg: -360.0, max_deg: 360.0 }
    ];

    // dx, dy, dz, rx, ry, rz are stored as the relative offset from the DH Joint frame ONLY if coord_mode === 'dh_joint',
    // OR as exact World Coordinates relative to Fixed Origin (0,0,0) when coord_mode === 'world_abs'.
    // Importantly, the UI Inspector ALWAYS displays and accepts TRUE World Coordinates (X, Y, Z, Rx, Ry, Rz) relative to Fixed Origin (0,0,0)!
    this.linkOffsets = [
      { link: 0, name: "Part 0: Đế Robot (Base)",        dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 1, name: "Part 1: Cụm Vai (Khâu J1)",      dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 2, name: "Part 2: Cánh Tay Trên (Khâu J2)", dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 3, name: "Part 3: Khuỷu Tay (Khâu J3)",    dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 4, name: "Part 4: Cẳng Tay (Khâu J4)",     dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 5, name: "Part 5: Cổ Tay (Khâu J5)",       dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null },
      { link: 6, name: "Part 6: Mặt Bích (Khâu J6)",     dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "bottom", auto_center: true, stl_file: null }
    ];

    this.jointLimits = this.dhTable.map(r => [r.min_deg, r.max_deg]);

    this.linkMaterials = [];
    for (let i = 0; i <= 6; i++) {
      const baseColor = i === 0 ? 0x1e293b : (i % 2 === 1 ? 0xcbd5e1 : 0x0284c7);
      this.linkMaterials[i] = new THREE.MeshStandardMaterial({
        color: baseColor,
        roughness: 0.4,
        metalness: 0.35
      });
    }
    this.matTool = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.3, metalness: 0.4 });

    this.parentNodes = [];
    this.linkMeshes = [];
    this.meshHolders = [];
    this.partOriginFrames = [];
    this.dhJointAxes = [];
    this.rawGeometries = [null, null, null, null, null, null, null];
    this.stlBounds = [null, null, null, null, null, null, null];
    this.hasCustomSTL = [false, false, false, false, false, false, false];

    this.buildHierarchy();
  }

  createPartOriginFrame() {
    const group = new THREE.Group();
    const len = 95;
    const headLen = 14;
    const headW = 6;

    const axX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), len, 0xff5252, headLen, headW);
    const axY = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), len, 0x69f0ae, headLen, headW);
    const axZ = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), len, 0x40c4ff, headLen, headW);

    const centerDot = new THREE.Mesh(
      new THREE.SphereGeometry(5.5, 14, 14),
      new THREE.MeshBasicMaterial({ color: 0xffd740 })
    );

    group.add(axX, axY, axZ, centerDot);
    group.visible = false;
    return group;
  }

  buildHierarchy() {
    this.root = new THREE.Group();
    this.root.name = "PAROL6_Robot";

    this.baseGroup = new THREE.Group();
    this.root.add(this.baseGroup);

    this.j1Node = new THREE.Group();
    this.baseGroup.add(this.j1Node);

    this.j2Node = new THREE.Group();
    this.j1Node.add(this.j2Node);

    this.j3Node = new THREE.Group();
    this.j2Node.add(this.j3Node);

    this.j4Node = new THREE.Group();
    this.j3Node.add(this.j4Node);

    this.j5Node = new THREE.Group();
    this.j4Node.add(this.j5Node);

    this.j6Node = new THREE.Group();
    this.j5Node.add(this.j6Node);

    this.flangeNode = new THREE.Group();
    this.j6Node.add(this.flangeNode);

    this.parentNodes = [
      this.baseGroup,
      this.j1Node,
      this.j2Node,
      this.j3Node,
      this.j4Node,
      this.j5Node,
      this.j6Node
    ];

    for (let i = 0; i <= 6; i++) {
      const dhAxes = new THREE.AxesHelper(50);
      dhAxes.visible = false;
      this.parentNodes[i].add(dhAxes);
      this.dhJointAxes[i] = dhAxes;

      const holder = new THREE.Group();
      holder.name = `MeshHolder_Link_${i}`;
      holder.userData = { linkIndex: i };
      this.parentNodes[i].add(holder);
      this.meshHolders[i] = holder;

      const partOrigin = this.createPartOriginFrame();
      holder.add(partOrigin);
      this.partOriginFrames[i] = partOrigin;

      const defaultMesh = this.createDefaultMeshForLink(i);
      this.tagMeshHierarchy(defaultMesh, i);
      holder.add(defaultMesh);
      this.linkMeshes[i] = defaultMesh;
    }

    this.tcpTarget = new THREE.Group();
    this.tcpTarget.name = "TCP_Target";
    this.flangeNode.add(this.tcpTarget);

    this.tcpAxes = new THREE.AxesHelper(55);
    this.tcpTarget.add(this.tcpAxes);

    this.toolMesh = this.createDefaultToolMesh();
    this.tcpTarget.add(this.toolMesh);

    this.scene.add(this.root);
    this.applyDHGeometry();
    this.applyLinkOffsets();
    this.updateTCPTransform();
    this.setJointAngles(this.joints);
  }

  tagMeshHierarchy(obj, linkIndex) {
    obj.userData = { ...obj.userData, linkIndex: linkIndex };
    if (obj.children) {
      obj.children.forEach(c => this.tagMeshHierarchy(c, linkIndex));
    }
  }

  selectPart(linkIndex) {
    this.selectedLinkIndex = linkIndex;
    for (let i = 0; i <= 6; i++) {
      const isSelected = (i === linkIndex);
      if (this.linkMaterials[i]) {
        this.linkMaterials[i].emissive.setHex(isSelected ? 0x004d66 : 0x000000);
      }
      if (this.partOriginFrames[i]) {
        this.partOriginFrames[i].visible = isSelected;
      }
    }
  }

  setDHJointAxesVisible(visible) {
    this.showDHJointFrames = !!visible;
    this.dhJointAxes.forEach(ax => {
      if (ax) ax.visible = this.showDHJointFrames;
    });
  }

  getParentHomeWorldMatrix(linkIndex) {
    const savedJoints = [...this.joints];
    const r = (deg) => THREE.MathUtils.degToRad(deg);
    const defaultOffsets = [0, 90, 0, 0, 0, 0];
    const effHome = (i) => {
      const dhOff = parseFloat(this.dhTable[i]?.offset_deg ?? defaultOffsets[i]);
      return 0 + (dhOff - defaultOffsets[i]);
    };

    this.j1Node.rotation.set(0, r(effHome(0)), 0);
    this.j2Node.rotation.set(0, 0, r(effHome(1)));
    this.j3Node.rotation.set(0, 0, r(effHome(2)));
    this.j4Node.rotation.set(0, r(effHome(3)), 0);
    this.j5Node.rotation.set(0, 0, r(effHome(4)));
    this.j6Node.rotation.set(0, r(effHome(5)), 0);
    this.root.updateMatrixWorld(true);

    const homeMat = this.parentNodes[linkIndex].matrixWorld.clone();

    this.j1Node.rotation.set(0, r(savedJoints[0] + (parseFloat(this.dhTable[0]?.offset_deg ?? 0) - 0)), 0);
    this.j2Node.rotation.set(0, 0, r(savedJoints[1] + (parseFloat(this.dhTable[1]?.offset_deg ?? 90) - 90)));
    this.j3Node.rotation.set(0, 0, r(savedJoints[2] + (parseFloat(this.dhTable[2]?.offset_deg ?? 0) - 0)));
    this.j4Node.rotation.set(0, r(savedJoints[3] + (parseFloat(this.dhTable[3]?.offset_deg ?? 0) - 0)), 0);
    this.j5Node.rotation.set(0, 0, r(savedJoints[4] + (parseFloat(this.dhTable[4]?.offset_deg ?? 0) - 0)));
    this.j6Node.rotation.set(0, r(savedJoints[5] + (parseFloat(this.dhTable[5]?.offset_deg ?? 0) - 0)), 0);
    this.root.updateMatrixWorld(true);

    return homeMat;
  }

  /**
   * Trả về tọa độ Mặc Định theo Bảng DH của Khớp i so với Gốc Cố Định (0,0,0) khi ở Home
   */
  getDHJointWorldPoseAtHome(linkIndex) {
    const mat = this.getParentHomeWorldMatrix(linkIndex);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const sc = new THREE.Vector3();
    mat.decompose(pos, quat, sc);
    const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
    const deg = (rad) => Math.round(THREE.MathUtils.radToDeg(rad) * 10) / 10;
    const rnd = (val) => Math.round(val * 10) / 10;
    return {
      x: rnd(pos.x),
      y: rnd(pos.z),
      z: rnd(pos.y),
      rx: deg(euler.x),
      ry: deg(euler.z),
      rz: deg(euler.y)
    };
  }

  /**
   * Trả về TỌA ĐỘ THỰC (X, Y, Z mm) và GÓC QUAY THỰC (Rx, Ry, Rz °) của Part i
   * tính trực tiếp từ GỐC CỐ ĐỊNH (0, 0, 0)!
   * Nếu Part nằm ở độ cao 118mm thì trả về Z = 118; nếu nằm ở mặt bàn (0,0,0) thì trả về Z = 0!
   */
  getPartTrueWorldPose(linkIndex) {
    const off = this.linkOffsets[linkIndex];
    if (!off) return { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0, scale: 1.0 };

    if (off.coord_mode === "world_abs") {
      return {
        x: off.dx || 0,
        y: off.dy || 0,
        z: off.dz || 0,
        rx: off.rx || 0,
        ry: off.ry || 0,
        rz: off.rz || 0,
        scale: off.scale || 1.0
      };
    }

    // Compute from current holder transform at Home
    const parentHomeMat = this.getParentHomeWorldMatrix(linkIndex);
    const holder = this.meshHolders[linkIndex];
    const localMat = new THREE.Matrix4().compose(holder.position, holder.quaternion, holder.scale);
    const worldMat = parentHomeMat.clone().multiply(localMat);

    const wPos = new THREE.Vector3();
    const wQuat = new THREE.Quaternion();
    const wSc = new THREE.Vector3();
    worldMat.decompose(wPos, wQuat, wSc);
    const wEuler = new THREE.Euler().setFromQuaternion(wQuat, 'XYZ');

    const deg = (rad) => Math.round(THREE.MathUtils.radToDeg(rad) * 10) / 10;
    const rnd = (val) => Math.round(val * 10) / 10;

    return {
      x: rnd(wPos.x),
      y: rnd(wPos.z),
      z: rnd(wPos.y),
      rx: deg(wEuler.x),
      ry: deg(wEuler.z),
      rz: deg(wEuler.y),
      scale: rnd(off.scale || 1.0)
    };
  }

  /**
   * Đặt TỌA ĐỘ THỰC (X, Y, Z mm) và GÓC QUAY THỰC (Rx, Ry, Rz °) của Part i
   * tính trực tiếp từ GỐC CỐ ĐỊNH (0, 0, 0)!
   * Khi nhập Z = 0 -> Part sẽ nằm đúng tại mặt phẳng Z = 0 của Gốc Cố Định!
   */
  setPartTrueWorldPose(linkIndex, { x, y, z, rx, ry, rz, scale }) {
    const off = this.linkOffsets[linkIndex];
    if (!off) return;
    off.coord_mode = "world_abs";
    off.dx = parseFloat(x) || 0;
    off.dy = parseFloat(y) || 0;
    off.dz = parseFloat(z) || 0;
    off.rx = parseFloat(rx) || 0;
    off.ry = parseFloat(ry) || 0;
    off.rz = parseFloat(rz) || 0;
    if (scale !== undefined) {
      off.scale = parseFloat(scale) > 0 ? parseFloat(scale) : 1.0;
    }
    this.applyLinkOffsets();
  }

  /**
   * Đặt Part i quay về đúng vị trí Khớp DH chuẩn của nó
   */
  snapPartToDHJoint(linkIndex) {
    const off = this.linkOffsets[linkIndex];
    if (!off) return;
    off.coord_mode = "dh_joint";
    off.dx = 0;
    off.dy = 0;
    off.dz = 0;
    off.rx = 0;
    off.ry = 0;
    off.rz = 0;
    this.applyLinkOffsets();
  }

  applyDHGeometry() {
    const d1 = parseFloat(this.dhTable[0]?.d) || 118;
    const a1 = parseFloat(this.dhTable[0]?.a) || 0;
    const a2 = parseFloat(this.dhTable[1]?.a) || 145;
    const d2 = parseFloat(this.dhTable[1]?.d) || 0;
    const a3 = parseFloat(this.dhTable[2]?.a) || 0;
    const d3 = parseFloat(this.dhTable[2]?.d) || 0;
    const d4 = parseFloat(this.dhTable[3]?.d) || 175;
    const a4 = parseFloat(this.dhTable[3]?.a) || 0;
    const d5 = parseFloat(this.dhTable[4]?.d) || 0;
    const a5 = parseFloat(this.dhTable[4]?.a) || 0;
    const d6 = parseFloat(this.dhTable[5]?.d) || 70;
    const a6 = parseFloat(this.dhTable[5]?.a) || 0;

    this.j1Node.position.set(a1, d1, 0);
    this.j2Node.position.set(0, 0, d2);
    this.j3Node.position.set(a3, a2, d3);
    this.j4Node.position.set(a4, 0, 0);
    this.j5Node.position.set(a5, d4, d5);
    this.j6Node.position.set(a6, d6, 0);
    this.flangeNode.position.set(0, 0, 0);

    this.jointLimits = this.dhTable.map(r => [
      parseFloat(r.min_deg ?? -180),
      parseFloat(r.max_deg ?? 180)
    ]);

    for (let i = 0; i <= 6; i++) {
      if (!this.hasCustomSTL[i] && this.meshHolders[i]) {
        if (this.linkMeshes[i]) {
          this.meshHolders[i].remove(this.linkMeshes[i]);
        }
        const newDefault = this.createDefaultMeshForLink(i);
        this.tagMeshHierarchy(newDefault, i);
        this.meshHolders[i].add(newDefault);
        this.linkMeshes[i] = newDefault;
      }
    }

    this.applyLinkOffsets();
    this.setJointAngles(this.joints);
  }

  updateDHTable(newTable) {
    if (!Array.isArray(newTable) || newTable.length !== 6) return;
    this.dhTable = newTable.map((r, i) => ({
      joint: i + 1,
      name: r.name || `J${i + 1}`,
      a: parseFloat(r.a) || 0,
      alpha_deg: parseFloat(r.alpha_deg) || 0,
      d: parseFloat(r.d) || 0,
      offset_deg: parseFloat(r.offset_deg) || 0,
      min_deg: parseFloat(r.min_deg ?? -180),
      max_deg: parseFloat(r.max_deg ?? 180)
    }));
    this.applyDHGeometry();
  }

  updateLinkOffsets(newOffsets) {
    if (!Array.isArray(newOffsets) || newOffsets.length !== 7) return;
    const prevAlign = this.linkOffsets.map(o => o.align_mode || 'bottom');

    this.linkOffsets = newOffsets.map((o, i) => {
      // Migrate legacy "local"/"world" where dx=0,dy=0,dz=0 was default joint pose
      let mode = o.coord_mode || "dh_joint";
      if (mode === "local" || (mode === "world" && !o.dx && !o.dy && !o.dz && !o.rx && !o.ry && !o.rz)) {
        mode = "dh_joint";
      } else if (mode === "world") {
        mode = "world_abs";
      }
      return {
        link: i,
        name: o.name || this.linkOffsets[i]?.name || `Part ${i}`,
        dx: parseFloat(o.dx) || 0,
        dy: parseFloat(o.dy) || 0,
        dz: parseFloat(o.dz) || 0,
        rx: parseFloat(o.rx) || 0,
        ry: parseFloat(o.ry) || 0,
        rz: parseFloat(o.rz) || 0,
        scale: parseFloat(o.scale) || 1.0,
        coord_mode: mode,
        align_mode: o.align_mode || this.linkOffsets[i]?.align_mode || "bottom",
        auto_center: o.auto_center !== undefined ? !!o.auto_center : true,
        stl_file: o.stl_file !== undefined ? o.stl_file : (this.linkOffsets[i]?.stl_file || null)
      };
    });

    for (let i = 0; i <= 6; i++) {
      if (this.hasCustomSTL[i] && this.rawGeometries[i] && prevAlign[i] !== this.linkOffsets[i].align_mode) {
        this._applyGeometryToHolder(i, this.rawGeometries[i]);
      }
    }

    this.applyLinkOffsets();
  }

  applyLinkOffsets() {
    const r = (deg) => THREE.MathUtils.degToRad(deg || 0);
    for (let i = 0; i <= 6; i++) {
      const holder = this.meshHolders[i];
      const off = this.linkOffsets[i];
      if (!holder || !off) continue;

      const s = off.scale && off.scale > 0 ? off.scale : 1.0;
      const targetScale = new THREE.Vector3(s, s, s);

      if (off.coord_mode === "world_abs") {
        // Đặt Gốc Riêng của Part i tại đúng tọa độ tuyệt đối (X=dx, Y=dy, Z=dz) so với Gốc Cố Định (0,0,0)!
        const targetPos = new THREE.Vector3(off.dx || 0, off.dz || 0, off.dy || 0);
        const targetEuler = new THREE.Euler(r(off.rx), r(off.rz), r(off.ry), 'XYZ');
        const targetQuat = new THREE.Quaternion().setFromEuler(targetEuler);

        const worldMat = new THREE.Matrix4().compose(targetPos, targetQuat, targetScale);
        const parentHomeMat = this.getParentHomeWorldMatrix(i);
        const parentHomeInv = parentHomeMat.clone().invert();
        const localMat = parentHomeInv.multiply(worldMat);

        const localPos = new THREE.Vector3();
        const localQuat = new THREE.Quaternion();
        const localSc = new THREE.Vector3();
        localMat.decompose(localPos, localQuat, localSc);

        holder.position.copy(localPos);
        holder.quaternion.copy(localQuat);
        holder.scale.copy(localSc);
      } else {
        // Mặc định: Đặt tại Khớp DH chuẩn
        holder.position.set(0, 0, 0);
        holder.quaternion.identity();
        holder.scale.copy(targetScale);
      }

      if (this.partOriginFrames[i]) {
        const invScale = 1.0 / (s || 1.0);
        this.partOriginFrames[i].scale.set(invScale, invScale, invScale);
      }
    }
    this.root.updateMatrixWorld(true);
  }

  /**
   * Khi người dùng cầm chuột kéo/xoay Part bằng Gizmo 3D -> Tính ra tọa độ thực (X, Y, Z, Rx, Ry, Rz) so với Gốc Cố Định (0,0,0)!
   */
  readBackOffsetFromHolder(linkIndex) {
    const holder = this.meshHolders[linkIndex];
    const off = this.linkOffsets[linkIndex];
    if (!holder || !off) return null;

    const deg = (rad) => Math.round(THREE.MathUtils.radToDeg(rad) * 10) / 10;
    const rnd = (val) => Math.round(val * 10) / 10;

    const parentHomeMat = this.getParentHomeWorldMatrix(linkIndex);
    const localMat = new THREE.Matrix4().compose(holder.position, holder.quaternion, holder.scale);
    const worldMatAtHome = parentHomeMat.clone().multiply(localMat);

    const wPos = new THREE.Vector3();
    const wQuat = new THREE.Quaternion();
    const wSc = new THREE.Vector3();
    worldMatAtHome.decompose(wPos, wQuat, wSc);
    const wEuler = new THREE.Euler().setFromQuaternion(wQuat, 'XYZ');

    off.coord_mode = "world_abs";
    off.dx = rnd(wPos.x);
    off.dy = rnd(wPos.z);
    off.dz = rnd(wPos.y);
    off.rx = deg(wEuler.x);
    off.ry = deg(wEuler.z);
    off.rz = deg(wEuler.y);

    return {
      x: off.dx,
      y: off.dy,
      z: off.dz,
      rx: off.rx,
      ry: off.ry,
      rz: off.rz,
      scale: off.scale
    };
  }

  createDefaultMeshForLink(linkIndex) {
    const d1 = Math.max(20, parseFloat(this.dhTable[0]?.d) || 118);
    const a2 = Math.max(20, parseFloat(this.dhTable[1]?.a) || 145);
    const d4 = Math.max(20, parseFloat(this.dhTable[3]?.d) || 175);
    const mat = this.linkMaterials[linkIndex];

    switch (linkIndex) {
      case 0: {
        const group = new THREE.Group();
        const baseH = Math.min(40, d1 * 0.35);
        const postH = Math.max(10, d1 - baseH);
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(70, 80, baseH, 32), mat);
        cyl.position.y = baseH / 2;
        cyl.castShadow = true;
        group.add(cyl);

        const lug = new THREE.Mesh(new THREE.BoxGeometry(30, baseH * 0.8, 35), mat);
        lug.position.set(70, baseH * 0.4, 0);
        group.add(lug);

        const post = new THREE.Mesh(new THREE.CylinderGeometry(48, 52, postH, 24), mat);
        post.position.y = baseH + postH / 2;
        post.castShadow = true;
        group.add(post);
        return group;
      }
      case 1: {
        const group = new THREE.Group();
        const body = new THREE.Mesh(new THREE.BoxGeometry(72, 68, 76), mat);
        body.position.y = 34;
        body.castShadow = true;
        group.add(body);
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(38, 38, 84, 24), mat);
        cyl.rotation.x = Math.PI / 2;
        cyl.position.y = 34;
        group.add(cyl);
        return group;
      }
      case 2: {
        const group = new THREE.Group();
        const arm = new THREE.Mesh(new THREE.BoxGeometry(46, a2, 46), mat);
        arm.position.y = a2 / 2;
        arm.castShadow = true;
        group.add(arm);
        return group;
      }
      case 3: {
        const group = new THREE.Group();
        const elbow = new THREE.Mesh(new THREE.BoxGeometry(58, 58, 64), mat);
        elbow.castShadow = true;
        group.add(elbow);
        return group;
      }
      case 4: {
        const group = new THREE.Group();
        const arm = new THREE.Mesh(new THREE.CylinderGeometry(28, 34, d4, 24), mat);
        arm.position.y = d4 / 2;
        arm.castShadow = true;
        group.add(arm);
        const fin = new THREE.Mesh(new THREE.BoxGeometry(44, d4 * 0.5, 16), mat);
        fin.position.y = d4 * 0.45;
        group.add(fin);
        return group;
      }
      case 5: {
        const group = new THREE.Group();
        const wrist = new THREE.Mesh(new THREE.BoxGeometry(46, 46, 52), mat);
        wrist.castShadow = true;
        group.add(wrist);
        return group;
      }
      case 6:
      default: {
        const group = new THREE.Group();
        const flange = new THREE.Mesh(new THREE.CylinderGeometry(30, 30, 16, 24), mat);
        flange.position.y = 8;
        flange.castShadow = true;
        group.add(flange);
        const keyway = new THREE.Mesh(new THREE.BoxGeometry(12, 16, 12), mat);
        keyway.position.set(24, 8, 0);
        group.add(keyway);
        return group;
      }
    }
  }

  createDefaultToolMesh() {
    const group = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(36, 30, 24), this.matTool);
    body.position.y = 15;
    group.add(body);
    const fingerGeo = new THREE.BoxGeometry(6, 35, 12);
    const f1 = new THREE.Mesh(fingerGeo, this.linkMaterials[0]);
    f1.position.set(-12, 45, 0);
    const f2 = new THREE.Mesh(fingerGeo, this.linkMaterials[0]);
    f2.position.set(12, 45, 0);
    group.add(f1, f2);
    return group;
  }

  setJointAngles(anglesDeg) {
    this.joints = [...anglesDeg];
    const r = (deg) => THREE.MathUtils.degToRad(deg);

    const defaultOffsets = [0, 90, 0, 0, 0, 0];
    const eff = (i) => {
      const dhOff = parseFloat(this.dhTable[i]?.offset_deg ?? defaultOffsets[i]);
      return anglesDeg[i] + (dhOff - defaultOffsets[i]);
    };

    this.j1Node.rotation.set(0, r(eff(0)), 0);
    this.j2Node.rotation.set(0, 0, r(eff(1)));
    this.j3Node.rotation.set(0, 0, r(eff(2)));
    this.j4Node.rotation.set(0, r(eff(3)), 0);
    this.j5Node.rotation.set(0, 0, r(eff(4)), 0);
    this.j6Node.rotation.set(0, r(eff(5)), 0);

    this.root.updateMatrixWorld(true);
  }

  setTCPOffset(offset) {
    this.tcpOffset = { ...this.tcpOffset, ...offset };
    this.updateTCPTransform();
  }

  updateTCPTransform() {
    this.tcpTarget.position.set(this.tcpOffset.x, this.tcpOffset.z, this.tcpOffset.y);
    this.tcpTarget.rotation.set(
      THREE.MathUtils.degToRad(this.tcpOffset.roll),
      THREE.MathUtils.degToRad(this.tcpOffset.yaw),
      THREE.MathUtils.degToRad(this.tcpOffset.pitch)
    );
    this.root.updateMatrixWorld(true);
  }

  getTCPWorldPose() {
    const worldPos = new THREE.Vector3();
    const worldQuat = new THREE.Quaternion();
    this.tcpTarget.getWorldPosition(worldPos);
    this.tcpTarget.getWorldQuaternion(worldQuat);

    const euler = new THREE.Euler().setFromQuaternion(worldQuat, 'XYZ');
    return {
      x: Math.round(worldPos.x * 100) / 100,
      y: Math.round(worldPos.z * 100) / 100,
      z: Math.round(worldPos.y * 100) / 100,
      roll: Math.round(THREE.MathUtils.radToDeg(euler.x) * 100) / 100,
      pitch: Math.round(THREE.MathUtils.radToDeg(euler.y) * 100) / 100,
      yaw: Math.round(THREE.MathUtils.radToDeg(euler.z) * 100) / 100
    };
  }

  /**
   * Khi nạp STL vào Part:
   * - align_mode = 'bottom' (Mặc định): Đưa tâm đáy (min Y trong Three.js = min Z độ cao) của STL về đúng Gốc Riêng Của Part (0,0,0),
   *   nhờ đó khi Z = 0 thì đáy của file STL nằm sát đúng mặt bàn Z = 0 của Gốc Cố Định!
   * - align_mode = 'center': Đưa tâm hình học 3 chiều của STL về Gốc Riêng Của Part.
   * - align_mode = 'cad': Giữ nguyên gốc tọa độ xuất từ phần mềm CAD.
   */
  _applyGeometryToHolder(linkIndex, rawGeom) {
    const geom = rawGeom.clone();
    geom.computeVertexNormals();
    geom.computeBoundingBox();

    const bb = geom.boundingBox;
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    bb.getSize(size);
    bb.getCenter(center);

    this.stlBounds[linkIndex] = {
      width: Math.round(size.x * 10) / 10,
      height: Math.round(size.y * 10) / 10,
      depth: Math.round(size.z * 10) / 10,
      maxDim: Math.max(size.x, size.y, size.z)
    };

    const alignMode = this.linkOffsets[linkIndex]?.align_mode || 'bottom';
    if (alignMode === 'bottom') {
      // Shift X and Z to center, and shift lowest height Y (bb.min.y) to 0 so bottom rests on Z=0!
      geom.translate(-center.x, -bb.min.y, -center.z);
    } else if (alignMode === 'center') {
      geom.center();
    }

    const mesh = new THREE.Mesh(geom, this.linkMaterials[linkIndex]);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.tagMeshHierarchy(mesh, linkIndex);

    const holder = this.meshHolders[linkIndex];
    if (holder) {
      if (this.linkMeshes[linkIndex]) {
        holder.remove(this.linkMeshes[linkIndex]);
      }
      holder.add(mesh);
      this.linkMeshes[linkIndex] = mesh;
      this.hasCustomSTL[linkIndex] = true;
    }
  }

  replaceLinkMesh(linkIndex, geometry, filename = null) {
    this.rawGeometries[linkIndex] = geometry.clone();
    this._applyGeometryToHolder(linkIndex, this.rawGeometries[linkIndex]);

    if (filename && this.linkOffsets[linkIndex]) {
      this.linkOffsets[linkIndex].stl_file = filename;
    }

    const bounds = this.stlBounds[linkIndex];
    if (bounds && bounds.maxDim > 0 && bounds.maxDim < 2.0 && this.linkOffsets[linkIndex]?.scale === 1.0) {
      this.linkOffsets[linkIndex].scale = 1000.0;
    }

    this.applyLinkOffsets();
  }

  setPartAlignMode(linkIndex, alignMode) {
    if (!this.linkOffsets[linkIndex]) return;
    this.linkOffsets[linkIndex].align_mode = alignMode;
    if (this.hasCustomSTL[linkIndex] && this.rawGeometries[linkIndex]) {
      this._applyGeometryToHolder(linkIndex, this.rawGeometries[linkIndex]);
    }
    this.applyLinkOffsets();
  }

  resetSingleLinkToDefault(linkIndex) {
    this.hasCustomSTL[linkIndex] = false;
    this.rawGeometries[linkIndex] = null;
    this.stlBounds[linkIndex] = null;
    if (this.linkOffsets[linkIndex]) {
      this.linkOffsets[linkIndex].stl_file = null;
      this.linkOffsets[linkIndex].coord_mode = "dh_joint";
      this.linkOffsets[linkIndex].dx = 0;
      this.linkOffsets[linkIndex].dy = 0;
      this.linkOffsets[linkIndex].dz = 0;
      this.linkOffsets[linkIndex].rx = 0;
      this.linkOffsets[linkIndex].ry = 0;
      this.linkOffsets[linkIndex].rz = 0;
      this.linkOffsets[linkIndex].scale = 1.0;
    }
    this.applyDHGeometry();
  }

  resetToDefaultMeshes() {
    for (let i = 0; i <= 6; i++) {
      this.hasCustomSTL[i] = false;
      this.rawGeometries[i] = null;
      this.stlBounds[i] = null;
      if (this.linkOffsets[i]) {
        this.linkOffsets[i].stl_file = null;
        this.linkOffsets[i].coord_mode = "dh_joint";
        this.linkOffsets[i].dx = 0;
        this.linkOffsets[i].dy = 0;
        this.linkOffsets[i].dz = 0;
        this.linkOffsets[i].rx = 0;
        this.linkOffsets[i].ry = 0;
        this.linkOffsets[i].rz = 0;
        this.linkOffsets[i].scale = 1.0;
      }
    }
    this.applyDHGeometry();
  }
}
