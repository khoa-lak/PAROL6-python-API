/**
 * 3D Viewport Module using Three.js (PAROL6-python-API/app)
 * Includes:
 * - Fixed Reference Origin (Gốc Cố Định 0,0,0) always visible
 * - 3D Raycaster for clicking any Robot Part or Workcell Object directly in 3D View
 * - OrbitControls + TransformControls (Gizmo)
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
    this.pickableRoot = null;
    this.onPartSelected3D = null;

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
   * Luôn hiển thị cố định tại tâm bàn máy (0,0,0) để làm chuẩn tham chiếu.
   * Quy ước màu Robotics: Trục X (Đỏ), Trục Y (Xanh lá), Trục Z (Xanh dương - hướng lên)
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

    // X Axis (Red) -> Three.js +X
    const arrowX = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0xff2e4c,
      headLen,
      headWidth
    );
    // Y Axis (Green) -> Three.js +Z
    const arrowY = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0x00e676,
      headLen,
      headWidth
    );
    // Z Axis (Blue - Vertical Up) -> Three.js +Y
    const arrowZ = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      axisLen,
      0x00b0ff,
      headLen,
      headWidth
    );

    // Center Reference Sphere & Ring at (0,0,0)
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

  setupRaycasterEvents() {
    this.canvas.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      this.pointerDownPos.set(e.clientX, e.clientY);
    });

    this.canvas.addEventListener('pointerup', (e) => {
      if (e.button !== 0 || this.isGizmoDragging) return;
      const dx = e.clientX - this.pointerDownPos.x;
      const dy = e.clientY - this.pointerDownPos.y;
      // Ignore if the user dragged the camera (distance > 5px)
      if (Math.hypot(dx, dy) > 5) return;

      const rect = this.canvas.getBoundingClientRect();
      const mouse = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1
      );

      this.raycaster.setFromCamera(mouse, this.camera);

      if (!this.pickableObjects || this.pickableObjects.length === 0) return;
      const intersects = this.raycaster.intersectObjects(this.pickableObjects, true);

      for (const hit of intersects) {
        // Walk up parent hierarchy to find a tagged Robot Link or Workcell Mesh
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

      // Clicked empty space -> deselect
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
