/**
 * 3D Viewport Module using Three.js (PAROL6-python-API/app)
 * Features:
 * - Right-Handed Robotics Z-up Coordinate Mapping:
 *     Three.js +X = Robotics +X (Đỏ)
 *     Three.js -Z = Robotics +Y (Xanh lá)
 *     Three.js +Y = Robotics +Z (Xanh dương - hướng lên)
 * - Fixed Reference Origin (Gốc Cố Định 0,0,0) always visible
 * - 3D Raycaster for clicking any Robot Part or Workcell Object directly in 3D View
 * - Interactive 2-Point 3D Measure Tool (📏 Đo Kích Thước)
 */
class Viewport3D {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.width = this.canvas.clientWidth || 800;
    this.height = this.canvas.clientHeight || 600;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0c1017);

    this.camera = new THREE.PerspectiveCamera(45, this.width / this.height, 1, 5000);
    this.camera.position.set(600, 520, 580);

    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: false });
    this.renderer.setSize(this.width, this.height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.06;
    this.controls.target.set(0, 120, 0);
    this.controls.maxPolarAngle = Math.PI / 2 + 0.15;

    this.raycaster = new THREE.Raycaster();
    this.pointerDownPos = new THREE.Vector2();
    this.isGizmoDragging = false;
    this.pickableObjects = [];
    this.onPartSelected3D = null;

    // Measure Tool State
    this.measureMode = false;
    this.measurePoints = [];
    this.measureGroup = new THREE.Group();
    this.measureGroup.name = "Measure_Overlay";
    this.scene.add(this.measureGroup);
    this.onMeasureUpdate = null;

    this.setupLighting();
    this.setupFixedReferenceOrigin();
    this.setupGizmo();
    this.setupRaycasterEvents();

    window.addEventListener('resize', () => this.onResize());

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  setupLighting() {
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
    dirLight.position.set(500, 900, 600);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 10;
    dirLight.shadow.camera.far = 2500;
    const d = 600;
    dirLight.shadow.camera.left = -d;
    dirLight.shadow.camera.right = d;
    dirLight.shadow.camera.top = d;
    dirLight.shadow.camera.bottom = -d;
    this.scene.add(dirLight);

    const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.35);
    fillLight.position.set(-450, 300, -400);
    this.scene.add(fillLight);
  }

  /**
   * GỐC TỌA ĐỘ 1: Gốc Cố Định (Fixed Reference Origin 0,0,0)
   * Hệ trục bàn tay phải (Right-Handed CAD Z-up):
   *   X (Đỏ) = +X_three
   *   Y (Xanh lá) = -Z_three
   *   Z (Xanh dương - hướng lên) = +Y_three
   */
  setupFixedReferenceOrigin() {
    const gridHelper = new THREE.GridHelper(1200, 24, 0x1e293b, 0x151d2a);
    gridHelper.position.y = 0;
    this.scene.add(gridHelper);

    this.fixedOriginGroup = new THREE.Group();
    this.fixedOriginGroup.name = "Fixed_Reference_Origin";

    const axisLen = 180;
    const headLen = 18;
    const headWidth = 8;

    const arrowX = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0xff2e4c,
      headLen,
      headWidth
    );
    const arrowY = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, -1),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0x00e676,
      headLen,
      headWidth
    );
    const arrowZ = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0x00b0ff,
      headLen,
      headWidth
    );

    const originSphere = new THREE.Mesh(
      new THREE.SphereGeometry(5, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffffff })
    );
    const originRing = new THREE.Mesh(
      new THREE.RingGeometry(18, 21, 32),
      new THREE.MeshBasicMaterial({ color: 0x00d2ff, side: THREE.DoubleSide })
    );
    originRing.rotation.x = -Math.PI / 2;
    originRing.position.y = 0.5;

    this.fixedOriginGroup.add(arrowX, arrowY, arrowZ, originSphere, originRing);
    this.scene.add(this.fixedOriginGroup);
  }

  setupGizmo() {
    this.gizmo = new THREE.TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.setSize(0.75);
    this.gizmo.setSpace('world');

    // Unify Gizmo axis colors to match Robotics Z-up convention:
    //   Three.js X -> Red (Robotics X)
    //   Three.js Y (Up) -> Blue (Robotics Z)
    //   Three.js Z -> Green (Robotics Y)
    this.gizmo.traverse((obj) => {
      if (!obj.material) return;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      mats.forEach((mat) => {
        if (!mat || !mat.color) return;
        if (obj.name === 'Y') {
          mat.color.setHex(0x00b0ff);
          if (mat._color) mat._color.setHex(0x00b0ff);
        } else if (obj.name === 'Z') {
          mat.color.setHex(0x00e676);
          if (mat._color) mat._color.setHex(0x00e676);
        } else if (obj.name === 'X') {
          mat.color.setHex(0xff2e4c);
          if (mat._color) mat._color.setHex(0xff2e4c);
        }
      });
    });

    this.gizmo.addEventListener('dragging-changed', (event) => {
      this.isGizmoDragging = event.value;
      this.controls.enabled = !event.value;
      if (this.onGizmoDragging) {
        this.onGizmoDragging(event.value);
      }
    });

    this.gizmo.addEventListener('change', () => {
      if (this.onGizmoChange && this.gizmo.object && this.isGizmoDragging) {
        this.onGizmoChange(this.gizmo.object);
      }
    });

    this.scene.add(this.gizmo);
  }

  /* --------- Measure Tool (📏 Đo Khoảng Cách & Kích Thước 3D) --------- */
  setMeasureMode(enabled) {
    this.measureMode = !!enabled;
    this.canvas.style.cursor = this.measureMode ? 'crosshair' : 'default';
    if (!this.measureMode) {
      this.clearMeasurePoints();
    }
  }

  clearMeasurePoints() {
    this.measurePoints = [];
    while (this.measureGroup.children.length > 0) {
      this.measureGroup.remove(this.measureGroup.children[0]);
    }
    if (this.onMeasureUpdate) {
      this.onMeasureUpdate(null);
    }
  }

  addMeasurePoint(worldVec3) {
    if (this.measurePoints.length >= 2) {
      this.clearMeasurePoints();
    }
    this.measurePoints.push(worldVec3.clone());

    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(4, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xffeb3b, depthTest: false })
    );
    marker.renderOrder = 999;
    marker.position.copy(worldVec3);
    this.measureGroup.add(marker);

    if (this.measurePoints.length === 2) {
      const p1 = this.measurePoints[0];
      const p2 = this.measurePoints[1];

      const lineGeo = new THREE.BufferGeometry().setFromPoints([p1, p2]);
      const lineMat = new THREE.LineBasicMaterial({ color: 0xffeb3b, linewidth: 2, depthTest: false });
      const line = new THREE.Line(lineGeo, lineMat);
      line.renderOrder = 999;
      this.measureGroup.add(line);

      // Convert Three.js (X, Y_up, -Z) -> Robotics (X, Y, Z)
      const toRob = (v) => ({ x: v.x, y: -v.z, z: v.y });
      const r1 = toRob(p1);
      const r2 = toRob(p2);
      const dx = r2.x - r1.x;
      const dy = r2.y - r1.y;
      const dz = r2.z - r1.z;
      const dist = Math.hypot(dx, dy, dz);

      if (this.onMeasureUpdate) {
        this.onMeasureUpdate({
          p1: r1,
          p2: r2,
          dx: Math.round(dx * 100) / 100,
          dy: Math.round(dy * 100) / 100,
          dz: Math.round(dz * 100) / 100,
          distance: Math.round(dist * 100) / 100
        });
      }
    } else if (this.onMeasureUpdate) {
      const r1 = { x: worldVec3.x, y: -worldVec3.z, z: worldVec3.y };
      this.onMeasureUpdate({ p1: r1, p2: null, distance: 0 });
    }
  }

  setupRaycasterEvents() {
    this.canvas.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      this.pointerDownPos.set(e.clientX, e.clientY);
    });

    this.canvas.addEventListener('pointerup', (e) => {
      if (e.button !== 0 || this.isGizmoDragging) return;
      const dx = e.clientX - this.pointerDownPos.x;
      const dy = e.clientY - this.pointerDownPos.y;
      if (Math.hypot(dx, dy) > 5) return;

      const rect = this.canvas.getBoundingClientRect();
      const mouse = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1
      );

      this.raycaster.setFromCamera(mouse, this.camera);

      if (!this.pickableObjects || this.pickableObjects.length === 0) return;
      const intersects = this.raycaster.intersectObjects(this.pickableObjects, true);

      if (this.measureMode) {
        // Filter out gizmo / helper lines, pick first real mesh surface hit
        for (const hit of intersects) {
          if (hit.object && hit.object.isMesh) {
            this.addMeasurePoint(hit.point);
            return;
          }
        }
        // Fallback: intersect ground plane Z=0
        const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const target = new THREE.Vector3();
        if (this.raycaster.ray.intersectPlane(plane, target)) {
          this.addMeasurePoint(target);
        }
        return;
      }

      for (const hit of intersects) {
        let cur = hit.object;
        while (cur) {
          if (cur.userData && cur.userData.linkIndex !== undefined) {
            if (this.onPartSelected3D) {
              this.onPartSelected3D({ type: 'robot_link', index: cur.userData.linkIndex });
            }
            return;
          }
          if (cur.userData && cur.userData.workcellIndex !== undefined) {
            if (this.onPartSelected3D) {
              this.onPartSelected3D({ type: 'workcell', index: cur.userData.workcellIndex });
            }
            return;
          }
          cur = cur.parent;
        }
      }

      if (this.onPartSelected3D) {
        this.onPartSelected3D(null);
      }
    });
  }

  setPickableObjects(objectsArray) {
    this.pickableObjects = objectsArray;
  }

  setGizmoMode(mode) {
    if (mode === 'hide') {
      this.gizmo.detach();
    } else {
      if (this.currentGizmoObject) {
        this.gizmo.attach(this.currentGizmoObject);
        this.gizmo.setMode(mode);
      }
    }
  }

  attachGizmoTo(object) {
    this.currentGizmoObject = object;
    if (object) {
      this.gizmo.attach(object);
    } else {
      this.gizmo.detach();
    }
  }

  detachGizmo() {
    this.currentGizmoObject = null;
    this.gizmo.detach();
  }

  setCameraView(view) {
    const dist = 750;
    switch (view) {
      case 'iso':
        this.camera.position.set(dist * 0.7, dist * 0.65, dist * 0.7);
        break;
      case 'top':
        this.camera.position.set(0, dist * 1.3, 1);
        break;
      case 'front':
        this.camera.position.set(0, 160, dist);
        break;
      case 'right':
        this.camera.position.set(dist, 160, 0);
        break;
    }
    this.controls.target.set(0, 140, 0);
    this.controls.update();
  }

  onResize() {
    if (!this.canvas.parentElement) return;
    this.width = Math.max(100, this.canvas.parentElement.clientWidth);
    this.height = Math.max(100, this.canvas.parentElement.clientHeight);
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(this.width, this.height);
  }

  animate() {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
