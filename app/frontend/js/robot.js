/**
 * PAROL6 Robot 3D Model & Dynamic DH Kinematic Hierarchy (PAROL6-python-API/app)
 * Right-Handed Robotics Z-up Coordinate Mapping:
 *   Three.js +X = Robotics +X (Đỏ)
 *   Three.js -Z = Robotics +Y (Xanh lá)
 *   Three.js +Y = Robotics +Z (Xanh dương - hướng lên)
 *
 * Two-Level Part Transform:
 * 1. Gốc Riêng Của STL (Pivot): pivot_dx, pivot_dy, pivot_dz, pivot_rx, pivot_ry, pivot_rz
 *    -> Dịch/xoay khối STL tương đối so với Gốc 2 (🟡), cho phép chỉnh lại vị trí Gốc 2 trên file STL.
 * 2. Di chuyển STL + Gốc STL theo Gốc Fix (⚪ 0,0,0): dx, dy, dz, rx, ry, rz, scale
 *    -> Di chuyển cả STL + Gốc 2 cùng nhau so với Gốc Cố Định (0,0,0).
 */
class RobotModel {
  constructor(scene) {
    this.scene = scene;
    this.joints = [0, 0, 0, 0, 0, 0];
    this.tcpOffset = { x: 0, y: 0, z: 65, roll: 0, pitch: 0, yaw: 0 };
    this.selectedLinkIndex = -1;
    this.showDHJointFrames = false;

    this.dhTable = [
      { joint: 1, name: "J1 (Shoulder Base)", a: 0.0,   alpha_deg: 90.0,  d: 118.0, offset_deg: 0.0,  min_deg: -170.0, max_deg: 170.0, rot_axis: "z" },
      { joint: 2, name: "J2 (Upper Arm)",     a: 145.0, alpha_deg: 0.0,   d: 0.0,   offset_deg: 90.0, min_deg: -100.0, max_deg: 90.0,  rot_axis: "y" },
      { joint: 3, name: "J3 (Elbow)",         a: 0.0,   alpha_deg: 90.0,  d: 0.0,   offset_deg: 0.0,  min_deg: -150.0, max_deg: 130.0, rot_axis: "y" },
      { joint: 4, name: "J4 (Forearm)",       a: 0.0,   alpha_deg: -90.0, d: 175.0, offset_deg: 0.0,  min_deg: -180.0, max_deg: 180.0, rot_axis: "z" },
      { joint: 5, name: "J5 (Wrist)",         a: 0.0,   alpha_deg: 90.0,  d: 0.0,   offset_deg: 0.0,  min_deg: -105.0, max_deg: 105.0, rot_axis: "y" },
      { joint: 6, name: "J6 (Flange)",        a: 0.0,   alpha_deg: 0.0,   d: 70.0,  offset_deg: 0.0,  min_deg: -360.0, max_deg: 360.0, rot_axis: "z" }
    ];

    this.linkOffsets = [
      { link: 0, name: "Part 0: Đế Robot (Base)",        dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "z", auto_center: false, deleted: false, stl_file: null },
      { link: 1, name: "Part 1: Cụm Vai (Khâu J1)",      dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "z", auto_center: false, deleted: false, stl_file: null },
      { link: 2, name: "Part 2: Cánh Tay Trên (Khâu J2)", dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "y", auto_center: false, deleted: false, stl_file: null },
      { link: 3, name: "Part 3: Khuỷu Tay (Khâu J3)",    dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "y", auto_center: false, deleted: false, stl_file: null },
      { link: 4, name: "Part 4: Cẳng Tay (Khâu J4)",     dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "z", auto_center: false, deleted: false, stl_file: null },
      { link: 5, name: "Part 5: Cổ Tay (Khâu J5)",       dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "y", auto_center: false, deleted: false, stl_file: null },
      { link: 6, name: "Part 6: Mặt Bích (Khâu J6)",     dx: 0, dy: 0, dz: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0, coord_mode: "dh_joint", align_mode: "cad", stl_up_axis: "z", rot_axis: "z", auto_center: false, deleted: false, stl_file: null }
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
    this.deletedGeometriesCache = [null, null, null, null, null, null, null];
    this.stlBounds = [null, null, null, null, null, null, null];
    this.hasCustomSTL = [false, false, false, false, false, false, false];

    this.buildHierarchy();
  }

  /**
   * Hệ trục chuẩn Robotics Z-up thống nhất toàn bộ App:
   *   Đỏ = X (+X_three) | Xanh Lá = Y (-Z_three) | Xanh Dương = Z (+Y_three hướng lên)
   */
  createRoboticsAxesHelper(len = 50) {
    const group = new THREE.Group();
    const headLen = len * 0.2;
    const headW = len * 0.08;
    const axX = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), len, 0xff2e4c, headLen, headW);
    const axY = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), len, 0x00e676, headLen, headW);
    const axZ = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), len, 0x00b0ff, headLen, headW);
    group.add(axX, axY, axZ);
    return group;
  }

  /**
   * Gốc Tọa Độ 2 (Gốc Riêng Của Part - Tâm Vàng):
   * Hệ trục bàn tay phải khớp 100% với Gốc Cố Định: X (Đỏ: +X), Y (Lục: -Z_three), Z (Lam: +Y_three)
   */
  createPartOriginFrame() {
    const group = this.createRoboticsAxesHelper(95);
    const centerDot = new THREE.Mesh(
      new THREE.SphereGeometry(5.5, 14, 14),
      new THREE.MeshBasicMaterial({ color: 0xffd740 })
    );
    group.add(centerDot);
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
      const dhAxes = this.createRoboticsAxesHelper(55);
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

    this.tcpAxes = this.createRoboticsAxesHelper(55);
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
    const jNodes = [this.j1Node, this.j2Node, this.j3Node, this.j4Node, this.j5Node, this.j6Node];
    for (let i = 0; i < 6; i++) {
      this._applyJointNodeRotation(jNodes[i], i, 0);
    }
    this.root.updateMatrixWorld(true);

    const homeMat = this.parentNodes[linkIndex].matrixWorld.clone();

    for (let i = 0; i < 6; i++) {
      this._applyJointNodeRotation(jNodes[i], i, savedJoints[i] || 0);
    }
    this.root.updateMatrixWorld(true);

    return homeMat;
  }

  /**
   * Trả về TỌA ĐỘ THỰC (X, Y, Z mm) và GÓC QUAY THỰC (Rx, Ry, Rz °) của Gốc Riêng Part i (Gốc 2)
   * tính trực tiếp từ GỐC CỐ ĐỊNH (0, 0, 0) theo hệ trục Robotics bàn tay phải (Y = -Z_three, Z = +Y_three).
   */
  getPartTrueWorldPose(linkIndex) {
    const off = this.linkOffsets[linkIndex];
    if (!off) {
      return { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0, pivot_dx: 0, pivot_dy: 0, pivot_dz: 0, pivot_rx: 0, pivot_ry: 0, pivot_rz: 0, scale: 1.0 };
    }

    if (off.coord_mode === "world_abs") {
      return {
        x: off.dx || 0,
        y: off.dy || 0,
        z: off.dz || 0,
        rx: off.rx || 0,
        ry: off.ry || 0,
        rz: off.rz || 0,
        pivot_dx: off.pivot_dx || 0,
        pivot_dy: off.pivot_dy || 0,
        pivot_dz: off.pivot_dz || 0,
        pivot_rx: off.pivot_rx || 0,
        pivot_ry: off.pivot_ry || 0,
        pivot_rz: off.pivot_rz || 0,
        scale: off.scale || 1.0
      };
    }

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
      y: rnd(-wPos.z),
      z: rnd(wPos.y),
      rx: deg(wEuler.x),
      ry: deg(-wEuler.z),
      rz: deg(wEuler.y),
      pivot_dx: off.pivot_dx || 0,
      pivot_dy: off.pivot_dy || 0,
      pivot_dz: off.pivot_dz || 0,
      pivot_rx: off.pivot_rx || 0,
      pivot_ry: off.pivot_ry || 0,
      pivot_rz: off.pivot_rz || 0,
      scale: rnd(off.scale || 1.0)
    };
  }

  /**
   * Di chuyển STL + Gốc Riêng Của STL (Gốc 2) theo Gốc Fix (0,0,0)
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
   * Điều chỉnh Gốc Riêng Của STL (Pivot Offset): dịch/xoay khối STL tương đối so với Gốc 2
   */
  setPartPivotOffset(linkIndex, { pivot_dx, pivot_dy, pivot_dz, pivot_rx, pivot_ry, pivot_rz }) {
    const off = this.linkOffsets[linkIndex];
    if (!off) return;
    off.pivot_dx = parseFloat(pivot_dx) || 0;
    off.pivot_dy = parseFloat(pivot_dy) || 0;
    off.pivot_dz = parseFloat(pivot_dz) || 0;
    off.pivot_rx = parseFloat(pivot_rx) || 0;
    off.pivot_ry = parseFloat(pivot_ry) || 0;
    off.pivot_rz = parseFloat(pivot_rz) || 0;
    this.applyLinkOffsets();
  }

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

  /**
   * Tự động tính Bảng DH & Tâm Khớp Quay J1..J6 dựa trực tiếp trên Gốc Riêng (🟡 Gốc 2) của Part 1..6!
   * Mỗi khớp Ji sẽ đặt tâm quay tại đúng Gốc 2 của Part i.
   */
  autoSetDHFromPartOrigins() {
    const poses = [];
    for (let i = 0; i <= 6; i++) {
      poses.push(this.getPartTrueWorldPose(i));
    }
    const rnd = (v) => Math.round(v * 1000) / 1000;
    const dist = (a, b) => Math.round(Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z) * 100) / 100;

    // P0 = Base origin (if Part 0 is deleted/at 0, use 0,0,0; otherwise Part 0's origin)
    const p0 = this.linkOffsets[0]?.deleted ? { x: 0, y: 0, z: 0 } : poses[0];
    const p1 = poses[1];
    const p2 = poses[2];
    const p3 = poses[3];
    const p4 = poses[4];
    const p5 = poses[5];
    const p6 = poses[6];

    // Vector deltas between consecutive Part origins
    const d01 = { x: p1.x - p0.x, y: p1.y - p0.y, z: p1.z - p0.z };
    const d12 = { x: p2.x - p1.x, y: p2.y - p1.y, z: p2.z - p1.z };
    const d23 = { x: p3.x - p2.x, y: p3.y - p2.y, z: p3.z - p2.z };
    const d34 = { x: p4.x - p3.x, y: p4.y - p3.y, z: p4.z - p3.z };
    const d45 = { x: p5.x - p4.x, y: p5.y - p4.y, z: p5.z - p4.z };
    const d56 = { x: p6.x - p5.x, y: p6.y - p5.y, z: p6.z - p5.z };

    // Populate DH table rows: store readable DH approximations in a, d AND exact 3D joint offsets in jx, jy, jz
    // Note: in applyDHGeometry(), j_iNode.position uses (a_prev + jx, d_prev + jz, -(d_cur + jy))
    // So we put a1 = rnd(d12.x), d2 = rnd(d12.y), jz(1) = rnd(d12.z) and keep extra offsets in jx, jy, jz!
    this.dhTable[0].d = rnd(d01.z);
    this.dhTable[0].a = rnd(d12.x);
    this.dhTable[0].jx = rnd(d01.x);
    this.dhTable[0].jy = rnd(d01.y);
    this.dhTable[0].jz = 0;

    this.dhTable[1].d = rnd(d12.y);
    this.dhTable[1].a = rnd(d23.z);
    this.dhTable[1].jx = 0;
    this.dhTable[1].jy = 0;
    this.dhTable[1].jz = rnd(d12.z);

    this.dhTable[2].a = rnd(d23.x);
    this.dhTable[2].d = rnd(d23.y);
    this.dhTable[2].jx = 0;
    this.dhTable[2].jy = 0;
    this.dhTable[2].jz = 0;

    this.dhTable[3].a = rnd(d34.x);
    this.dhTable[3].d = rnd(dist(p3, p5));
    this.dhTable[3].jx = 0;
    this.dhTable[3].jy = rnd(d34.y);
    this.dhTable[3].jz = rnd(d34.z);

    // Exact J5 relative to J4
    this.dhTable[4].a = rnd(d45.x);
    this.dhTable[4].d = rnd(d45.y);
    this.dhTable[4].jx = 0;
    this.dhTable[4].jy = 0;
    this.dhTable[4].jz = rnd(d45.z - this.dhTable[3].d);

    // Exact J6 relative to J5
    this.dhTable[5].a = rnd(d56.x);
    this.dhTable[5].d = rnd(d56.z);
    this.dhTable[5].jx = 0;
    this.dhTable[5].jy = rnd(d56.y);
    this.dhTable[5].jz = 0;

    // Copy each Part's chosen rotation axis (rot_axis) into the DH table automatically!
    const defaultAxes = ["z", "y", "y", "z", "y", "z"];
    for (let j = 0; j < 6; j++) {
      this.dhTable[j].rot_axis = this.linkOffsets[j + 1]?.rot_axis || this.dhTable[j].rot_axis || defaultAxes[j];
    }

    this.applyDHGeometry();
    return this.dhTable;
  }

  applyDHGeometry() {
    const num = (v, fb = 0) => (v !== undefined && v !== null && !isNaN(parseFloat(v))) ? parseFloat(v) : fb;
    const d1 = num(this.dhTable[0]?.d, 118);
    const a1 = num(this.dhTable[0]?.a, 0);
    const a2 = num(this.dhTable[1]?.a, 145);
    const d2 = num(this.dhTable[1]?.d, 0);
    const a3 = num(this.dhTable[2]?.a, 0);
    const d3 = num(this.dhTable[2]?.d, 0);
    const d4 = num(this.dhTable[3]?.d, 175);
    const a4 = num(this.dhTable[3]?.a, 0);
    const d5 = num(this.dhTable[4]?.d, 0);
    const a5 = num(this.dhTable[4]?.a, 0);
    const d6 = num(this.dhTable[5]?.d, 70);
    const a6 = num(this.dhTable[5]?.a, 0);

    const jx = (idx) => num(this.dhTable[idx]?.jx, 0);
    const jy = (idx) => num(this.dhTable[idx]?.jy, 0);
    const jz = (idx) => num(this.dhTable[idx]?.jz, 0);

    // Three.js +X = Robotics X, +Y = Robotics Z (up), -Z = Robotics Y
    this.j1Node.position.set(jx(0), d1 + jz(0), -jy(0));
    this.j2Node.position.set(a1 + jx(1), jz(1), -(d2 + jy(1)));
    this.j3Node.position.set(a3 + jx(2), a2 + jz(2), -(d3 + jy(2)));
    this.j4Node.position.set(a4 + jx(3), jz(3), -jy(3));
    this.j5Node.position.set(a5 + jx(4), d4 + jz(4), -(d5 + jy(4)));
    this.j6Node.position.set(a6 + jx(5), d6 + jz(5), -jy(5));
    this.flangeNode.position.set(0, 0, 0);

    this.jointLimits = this.dhTable.map(r => [
      parseFloat(r.min_deg ?? -180),
      parseFloat(r.max_deg ?? 180)
    ]);

    for (let i = 0; i <= 6; i++) {
      if (!this.hasCustomSTL[i] && this.meshHolders[i]) {
        if (this.linkMeshes[i]) {
          this.meshHolders[i].remove(this.linkMeshes[i]);
          this.linkMeshes[i] = null;
        }
        if (!this.linkOffsets[i]?.deleted) {
          const newDefault = this.createDefaultMeshForLink(i);
          this.tagMeshHierarchy(newDefault, i);
          this.meshHolders[i].add(newDefault);
          this.linkMeshes[i] = newDefault;
        }
      }
    }

    if (this.toolMesh) {
      this.toolMesh.visible = !this.linkOffsets[6]?.deleted && !this.hasCustomSTL[6];
    }

    this.applyLinkOffsets();
    this.setJointAngles(this.joints);
  }

  updateDHTable(newTable) {
    if (!Array.isArray(newTable) || newTable.length !== 6) return;
    const defaultAxes = ["z", "y", "y", "z", "y", "z"];
    this.dhTable = newTable.map((r, i) => ({
      joint: i + 1,
      name: r.name || `J${i + 1}`,
      a: parseFloat(r.a) || 0,
      alpha_deg: parseFloat(r.alpha_deg) || 0,
      d: parseFloat(r.d) || 0,
      offset_deg: parseFloat(r.offset_deg) || 0,
      min_deg: parseFloat(r.min_deg ?? -180),
      max_deg: parseFloat(r.max_deg ?? 180),
      jx: parseFloat(r.jx) || 0,
      jy: parseFloat(r.jy) || 0,
      jz: parseFloat(r.jz) || 0,
      rot_axis: r.rot_axis || defaultAxes[i]
    }));
    this.applyDHGeometry();
  }

  updateLinkOffsets(newOffsets) {
    if (!Array.isArray(newOffsets) || newOffsets.length !== 7) return;
    const prevAlign = this.linkOffsets.map(o => o.align_mode || 'cad');
    const prevUpAxis = this.linkOffsets.map(o => o.stl_up_axis || 'z');

    this.linkOffsets = newOffsets.map((o, i) => {
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
        pivot_dx: parseFloat(o.pivot_dx) || 0,
        pivot_dy: parseFloat(o.pivot_dy) || 0,
        pivot_dz: parseFloat(o.pivot_dz) || 0,
        pivot_rx: parseFloat(o.pivot_rx) || 0,
        pivot_ry: parseFloat(o.pivot_ry) || 0,
        pivot_rz: parseFloat(o.pivot_rz) || 0,
        scale: parseFloat(o.scale) || 1.0,
        coord_mode: mode,
        align_mode: o.align_mode || this.linkOffsets[i]?.align_mode || "cad",
        stl_up_axis: o.stl_up_axis || this.linkOffsets[i]?.stl_up_axis || "z",
        rot_axis: o.rot_axis || this.linkOffsets[i]?.rot_axis || (["z","z","y","y","z","y","z"][i]),
        auto_center: o.auto_center !== undefined ? !!o.auto_center : false,
        deleted: !!o.deleted,
        stl_file: o.stl_file !== undefined ? o.stl_file : (this.linkOffsets[i]?.stl_file || null)
      };
    });

    for (let i = 0; i <= 6; i++) {
      if (this.linkOffsets[i].deleted) {
        if (this.linkMeshes[i] && this.meshHolders[i]) {
          this.meshHolders[i].remove(this.linkMeshes[i]);
          this.linkMeshes[i] = null;
        }
        this.hasCustomSTL[i] = false;
      } else if (
        this.hasCustomSTL[i] &&
        this.rawGeometries[i] &&
        (prevAlign[i] !== this.linkOffsets[i].align_mode || prevUpAxis[i] !== this.linkOffsets[i].stl_up_axis)
      ) {
        this._applyGeometryToHolder(i, this.rawGeometries[i]);
      } else if (!this.hasCustomSTL[i] && !this.linkMeshes[i] && this.meshHolders[i]) {
        const newDefault = this.createDefaultMeshForLink(i);
        this.tagMeshHierarchy(newDefault, i);
        this.meshHolders[i].add(newDefault);
        this.linkMeshes[i] = newDefault;
      }
    }

    if (this.toolMesh) {
      this.toolMesh.visible = !this.linkOffsets[6]?.deleted && !this.hasCustomSTL[6];
    }

    this.applyLinkOffsets();
  }

  applyLinkOffsets() {
    const r = (deg) => THREE.MathUtils.degToRad(deg || 0);
    for (let i = 0; i <= 6; i++) {
      const holder = this.meshHolders[i];
      const mesh = this.linkMeshes[i];
      const off = this.linkOffsets[i];
      if (!holder || !off) continue;

      const s = off.scale && off.scale > 0 ? off.scale : 1.0;
      const targetScale = new THREE.Vector3(s, s, s);

      // 1. Apply World Transform (STL + Gốc 2 together) relative to Fixed Origin (0,0,0)
      if (off.coord_mode === "world_abs") {
        const targetPos = new THREE.Vector3(off.dx || 0, off.dz || 0, -(off.dy || 0));
        const targetEuler = new THREE.Euler(r(off.rx), r(off.rz), r(-(off.ry || 0)), 'XYZ');
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
        holder.position.set(0, 0, 0);
        holder.quaternion.identity();
        holder.scale.copy(targetScale);
      }

      // 2. Apply Pivot Offset (STL mesh relative to Gốc 2 inside meshHolders[i])
      if (mesh) {
        mesh.position.set(off.pivot_dx || 0, off.pivot_dz || 0, -(off.pivot_dy || 0));
        mesh.rotation.set(r(off.pivot_rx), r(off.pivot_rz), r(-(off.pivot_ry || 0)), 'XYZ');
      }

      // Keep Part Origin Frame (Gốc 2) unscaled
      if (this.partOriginFrames[i]) {
        const invScale = 1.0 / (s || 1.0);
        this.partOriginFrames[i].scale.set(invScale, invScale, invScale);
      }
    }
    this.root.updateMatrixWorld(true);
  }

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
    off.dy = rnd(-wPos.z);
    off.dz = rnd(wPos.y);
    off.rx = deg(wEuler.x);
    off.ry = deg(-wEuler.z);
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

  _applyJointNodeRotation(node, jointIdx, angleDeg) {
    const r = (deg) => THREE.MathUtils.degToRad(deg);
    const defaultAxes = ["z", "y", "y", "z", "y", "z"];

    const row = this.dhTable[jointIdx] || {};
    const axis = (row.rot_axis || defaultAxes[jointIdx]).toLowerCase();

    // Rotate cleanly around the joint's configured axis by angleDeg (Home 0° = exact assembled STL pose)
    if (axis === "x") {
      node.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), r(angleDeg));
    } else if (axis === "y") {
      node.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, -1), r(angleDeg));
    } else {
      node.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), r(angleDeg));
    }
  }

  setJointAngles(anglesDeg) {
    this.joints = [...anglesDeg];
    const jNodes = [this.j1Node, this.j2Node, this.j3Node, this.j4Node, this.j5Node, this.j6Node];
    for (let i = 0; i < 6; i++) {
      this._applyJointNodeRotation(jNodes[i], i, anglesDeg[i] || 0);
    }
    this.root.updateMatrixWorld(true);
  }

  setTCPOffset(offset) {
    this.tcpOffset = { ...this.tcpOffset, ...offset };
    this.updateTCPTransform();
  }

  updateTCPTransform() {
    this.tcpTarget.position.set(this.tcpOffset.x, this.tcpOffset.z, -this.tcpOffset.y);
    this.tcpTarget.rotation.set(
      THREE.MathUtils.degToRad(this.tcpOffset.roll),
      THREE.MathUtils.degToRad(this.tcpOffset.yaw),
      THREE.MathUtils.degToRad(-this.tcpOffset.pitch)
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
      y: Math.round(-worldPos.z * 100) / 100,
      z: Math.round(worldPos.y * 100) / 100,
      roll: Math.round(THREE.MathUtils.radToDeg(euler.x) * 100) / 100,
      pitch: Math.round(-THREE.MathUtils.radToDeg(euler.z) * 100) / 100,
      yaw: Math.round(THREE.MathUtils.radToDeg(euler.y) * 100) / 100
    };
  }

  /**
   * Khi nạp STL vào Part:
   * - Tự động quy đổi từ hệ CAD Z-up (X, Y, Z_up) sang hệ Robotics bàn tay phải (rotateX(-90°))
   *   mà KHÔNG làm thay đổi tọa độ gốc (0,0,0) bên trong file STL!
   * - align_mode = 'cad' (Mặc định): Giữ nguyên 100% gốc tọa độ của file STL khi xuất từ CAD.
   * - align_mode = 'bottom': Đưa tâm đáy về gốc.
   * - align_mode = 'center': Đưa tâm giữa về gốc.
   */
  _applyGeometryToHolder(linkIndex, rawGeom) {
    const geom = rawGeom.clone();
    const upAxis = this.linkOffsets[linkIndex]?.stl_up_axis || 'z';
    if (upAxis === 'z') {
      // Convert CAD Z-up (x, y, z) -> Three.js Right-Handed Robotics Z-up (x, z, -y)
      geom.rotateX(-Math.PI / 2);
    }
    geom.computeVertexNormals();
    geom.computeBoundingBox();

    const bb = geom.boundingBox;
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    bb.getSize(size);
    bb.getCenter(center);

    // Note: in Three.js, size.x = X_robot, size.z = Y_robot, size.y = Z_robot (height)
    this.stlBounds[linkIndex] = {
      width: Math.round(size.x * 10) / 10,
      depth: Math.round(size.z * 10) / 10,
      height: Math.round(size.y * 10) / 10,
      minX: Math.round(bb.min.x * 10) / 10,
      maxX: Math.round(bb.max.x * 10) / 10,
      minY: Math.round(-bb.max.z * 10) / 10,
      maxY: Math.round(-bb.min.z * 10) / 10,
      minZ: Math.round(bb.min.y * 10) / 10,
      maxZ: Math.round(bb.max.y * 10) / 10,
      maxDim: Math.max(size.x, size.y, size.z)
    };

    const alignMode = this.linkOffsets[linkIndex]?.align_mode || 'cad';
    if (alignMode === 'bottom') {
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
    if (this.linkOffsets[linkIndex]) {
      this.linkOffsets[linkIndex].deleted = false;
      if (filename) {
        this.linkOffsets[linkIndex].stl_file = filename;
      }
    }
    this.rawGeometries[linkIndex] = geometry.clone();
    this._applyGeometryToHolder(linkIndex, this.rawGeometries[linkIndex]);

    const bounds = this.stlBounds[linkIndex];
    if (bounds && bounds.maxDim > 0 && bounds.maxDim < 2.0 && this.linkOffsets[linkIndex]?.scale === 1.0) {
      this.linkOffsets[linkIndex].scale = 1000.0;
    }

    if (this.toolMesh) {
      this.toolMesh.visible = !this.linkOffsets[6]?.deleted && !this.hasCustomSTL[6];
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

  setPartUpAxis(linkIndex, upAxis) {
    if (!this.linkOffsets[linkIndex]) return;
    this.linkOffsets[linkIndex].stl_up_axis = upAxis;
    if (this.hasCustomSTL[linkIndex] && this.rawGeometries[linkIndex]) {
      this._applyGeometryToHolder(linkIndex, this.rawGeometries[linkIndex]);
    }
    this.applyLinkOffsets();
  }

  deleteSingleLinkMesh(linkIndex) {
    if (this.rawGeometries[linkIndex]) {
      this.deletedGeometriesCache[linkIndex] = this.rawGeometries[linkIndex].clone();
    }
    this.hasCustomSTL[linkIndex] = false;
    this.rawGeometries[linkIndex] = null;
    this.stlBounds[linkIndex] = null;
    if (this.meshHolders[linkIndex] && this.linkMeshes[linkIndex]) {
      this.meshHolders[linkIndex].remove(this.linkMeshes[linkIndex]);
      this.linkMeshes[linkIndex] = null;
    }
    if (this.partOriginFrames[linkIndex]) {
      this.partOriginFrames[linkIndex].visible = false;
    }
    if (this.linkOffsets[linkIndex]) {
      this.linkOffsets[linkIndex].stl_file = null;
      this.linkOffsets[linkIndex].deleted = true;
      this.linkOffsets[linkIndex].dx = 0;
      this.linkOffsets[linkIndex].dy = 0;
      this.linkOffsets[linkIndex].dz = 0;
      this.linkOffsets[linkIndex].rx = 0;
      this.linkOffsets[linkIndex].ry = 0;
      this.linkOffsets[linkIndex].rz = 0;
      this.linkOffsets[linkIndex].pivot_dx = 0;
      this.linkOffsets[linkIndex].pivot_dy = 0;
      this.linkOffsets[linkIndex].pivot_dz = 0;
      this.linkOffsets[linkIndex].pivot_rx = 0;
      this.linkOffsets[linkIndex].pivot_ry = 0;
      this.linkOffsets[linkIndex].pivot_rz = 0;
      this.linkOffsets[linkIndex].scale = 1.0;
    }
    if (this.toolMesh) {
      this.toolMesh.visible = !this.linkOffsets[6]?.deleted && !this.hasCustomSTL[6];
    }
  }

  clearAllLinkMeshes() {
    for (let i = 0; i <= 6; i++) {
      this.deleteSingleLinkMesh(i);
    }
    if (this.toolMesh) {
      this.toolMesh.visible = false;
    }
  }

  resetSingleLinkToDefault(linkIndex) {
    this.hasCustomSTL[linkIndex] = false;
    this.rawGeometries[linkIndex] = null;
    this.stlBounds[linkIndex] = null;
    if (this.linkOffsets[linkIndex]) {
      this.linkOffsets[linkIndex].stl_file = null;
      this.linkOffsets[linkIndex].deleted = false;
      this.linkOffsets[linkIndex].coord_mode = "dh_joint";
      this.linkOffsets[linkIndex].dx = 0;
      this.linkOffsets[linkIndex].dy = 0;
      this.linkOffsets[linkIndex].dz = 0;
      this.linkOffsets[linkIndex].rx = 0;
      this.linkOffsets[linkIndex].ry = 0;
      this.linkOffsets[linkIndex].rz = 0;
      this.linkOffsets[linkIndex].pivot_dx = 0;
      this.linkOffsets[linkIndex].pivot_dy = 0;
      this.linkOffsets[linkIndex].pivot_dz = 0;
      this.linkOffsets[linkIndex].pivot_rx = 0;
      this.linkOffsets[linkIndex].pivot_ry = 0;
      this.linkOffsets[linkIndex].pivot_rz = 0;
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
        this.linkOffsets[i].deleted = false;
        this.linkOffsets[i].coord_mode = "dh_joint";
        this.linkOffsets[i].dx = 0;
        this.linkOffsets[i].dy = 0;
        this.linkOffsets[i].dz = 0;
        this.linkOffsets[i].rx = 0;
        this.linkOffsets[i].ry = 0;
        this.linkOffsets[i].rz = 0;
        this.linkOffsets[i].pivot_dx = 0;
        this.linkOffsets[i].pivot_dy = 0;
        this.linkOffsets[i].pivot_dz = 0;
        this.linkOffsets[i].pivot_rx = 0;
        this.linkOffsets[i].pivot_ry = 0;
        this.linkOffsets[i].pivot_rz = 0;
        this.linkOffsets[i].scale = 1.0;
      }
    }
    this.applyDHGeometry();
  }

  /**
   * Khôi phục toàn bộ trạng thái Robot (dùng cho Undo Ctrl+Z / Redo Ctrl+Y)
   */
  restoreSnapshot(snap) {
    if (!snap) return;
    if (snap.dhTable) {
      this.updateDHTable(JSON.parse(JSON.stringify(snap.dhTable)));
    }
    if (snap.linkOffsets) {
      const clonedOffsets = JSON.parse(JSON.stringify(snap.linkOffsets));
      for (let i = 0; i <= 6; i++) {
        const off = clonedOffsets[i];
        if (!off) continue;
        if (!off.deleted && off.stl_file && !this.hasCustomSTL[i]) {
          const cachedGeom = this.rawGeometries[i] || this.deletedGeometriesCache[i];
          if (cachedGeom) {
            this.rawGeometries[i] = cachedGeom.clone();
            this._applyGeometryToHolder(i, this.rawGeometries[i]);
          }
        }
      }
      this.updateLinkOffsets(clonedOffsets);
    }
    if (snap.tcpOffset) {
      this.setTCPOffset(JSON.parse(JSON.stringify(snap.tcpOffset)));
    }
    if (snap.joints) {
      this.setJointAngles([...snap.joints]);
    }
  }
}
