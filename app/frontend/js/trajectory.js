/**
 * Trajectory Programming & 3D Simulation Player (PAROL6-python-API/app)
 */
class TrajectoryManager {
  constructor(robot, wsClient) {
    this.robot = robot;
    this.wsClient = wsClient;
    this.waypoints = [];
    this.isPlaying = false;

    this.setupUI();
  }

  setupUI() {
    document.getElementById('btn-add-movej').onclick = () => this.addWaypoint('MoveJ');
    document.getElementById('btn-add-movel').onclick = () => this.addWaypoint('MoveL');
    document.getElementById('btn-add-gripper-open').onclick = () => this.addGripperWaypoint('OPEN');
    document.getElementById('btn-add-gripper-close').onclick = () => this.addGripperWaypoint('CLOSE');
    document.getElementById('btn-add-wait').onclick = () => this.addWaitWaypoint(500);

    document.getElementById('btn-clear-waypoints').onclick = () => {
      if (confirm("Bạn có chắc muốn xóa toàn bộ kịch bản không?")) {
        this.waypoints = [];
        this.updateListUI();
      }
    };

    document.getElementById('btn-play-sim').onclick = () => this.toggleSimulation();
    document.getElementById('btn-export-gcode').onclick = () => this.exportGCodeFile();
    document.getElementById('btn-stream-usb').onclick = () => this.streamToUSB();
    const btnAvoid = document.getElementById('btn-auto-avoid');
    if (btnAvoid) {
      btnAvoid.onclick = () => this.autoAvoidObstacles();
    }
  }

  addWaypoint(type) {
    const tcp = this.robot.getTCPWorldPose();
    const joints = [...this.robot.joints];
    
    const wp = {
      id: Date.now(),
      type: type,
      joints: joints,
      cartesian: { x: tcp.x, y: tcp.y, z: tcp.z },
      orientation: { roll: tcp.roll, pitch: tcp.pitch, yaw: tcp.yaw },
      speed: 1000
    };

    this.waypoints.push(wp);
    this.updateListUI();
  }

  addGripperWaypoint(state) {
    this.waypoints.push({
      id: Date.now(),
      type: 'Gripper',
      state: state,
      pin: 1
    });
    this.updateListUI();
  }

  addWaitWaypoint(ms) {
    this.waypoints.push({
      id: Date.now(),
      type: 'Wait',
      delay_ms: ms
    });
    this.updateListUI();
  }

  updateListUI() {
    const container = document.getElementById('waypoint-list');
    document.getElementById('waypoint-count').innerText = `${this.waypoints.length} Điểm`;

    if (this.waypoints.length === 0) {
      container.innerHTML = '<div style="color: var(--text-dim); text-align: center; padding: 16px;">Chưa có điểm nào trong kịch bản.</div>';
      return;
    }

    container.innerHTML = this.waypoints.map((wp, idx) => {
      let badgeClass = 'type-movej';
      let infoText = '';

      if (wp.type === 'MoveJ') {
        badgeClass = 'type-movej';
        infoText = `J1:${wp.joints[0]}° J2:${wp.joints[1]}°...`;
      } else if (wp.type === 'MoveL') {
        badgeClass = 'type-movel';
        infoText = `X:${wp.cartesian.x} Y:${wp.cartesian.y} Z:${wp.cartesian.z}`;
      } else if (wp.type === 'Gripper') {
        badgeClass = 'type-gripper';
        infoText = `Kẹp ${wp.state === 'OPEN' ? 'MỞ' : 'ĐÓNG'}`;
      } else if (wp.type === 'Wait') {
        badgeClass = 'type-gripper';
        infoText = `Chờ ${wp.delay_ms}ms`;
      }

      return `
        <div class="waypoint-item" id="wp-item-${idx}">
          <div style="display: flex; align-items: center; gap: 6px;">
            <span style="color: var(--text-dim); font-size: 10px;">#${idx + 1}</span>
            <span class="waypoint-type ${badgeClass}">${wp.type}</span>
            <span style="font-size: 11px; color: #cbd5e1;">${infoText}</span>
          </div>
          <div style="display: flex; gap: 4px;">
            <button class="btn" style="padding: 2px 4px; font-size: 10px;" onclick="window.trajectoryManager.goToWaypoint(${idx})" title="Di chuyển robot tới điểm này">📍</button>
            <button class="btn" style="padding: 2px 4px; font-size: 10px;" onclick="window.trajectoryManager.deleteWaypoint(${idx})" title="Xóa">✕</button>
          </div>
        </div>
      `;
    }).join('');
  }

  goToWaypoint(idx) {
    const wp = this.waypoints[idx];
    if (wp && wp.joints) {
      window.app.setJointsFromExternal(wp.joints);
    }
  }

  deleteWaypoint(idx) {
    this.waypoints.splice(idx, 1);
    this.updateListUI();
  }

  async toggleSimulation() {
    if (this.isPlaying) {
      this.isPlaying = false;
      document.getElementById('btn-play-sim').innerText = "▶ Chạy Mô Phỏng 3D";
      return;
    }

    if (this.waypoints.length < 2) {
      alert("Cần tối thiểu 2 điểm để mô phỏng đường đi!");
      return;
    }

    this.isPlaying = true;
    document.getElementById('btn-play-sim').innerText = "⏹ Dừng Mô Phỏng";

    for (let i = 0; i < this.waypoints.length; i++) {
      if (!this.isPlaying) break;

      document.querySelectorAll('.waypoint-item').forEach(el => el.classList.remove('active'));
      const activeEl = document.getElementById(`wp-item-${i}`);
      if (activeEl) activeEl.classList.add('active');

      const wp = this.waypoints[i];
      if (wp.joints) {
        await this.animateInterpolation(this.robot.joints, wp.joints, 30);
      } else if (wp.type === 'Wait') {
        await new Promise(r => setTimeout(r, wp.delay_ms));
      }
    }

    this.isPlaying = false;
    document.getElementById('btn-play-sim').innerText = "▶ Chạy Mô Phỏng 3D";
    document.querySelectorAll('.waypoint-item').forEach(el => el.classList.remove('active'));
  }

  animateInterpolation(startQ, endQ, steps) {
    return new Promise((resolve) => {
      let currentStep = 0;
      const interval = setInterval(() => {
        if (!this.isPlaying || currentStep >= steps) {
          clearInterval(interval);
          window.app.setJointsFromExternal(endQ);
          resolve();
          return;
        }

        const t = currentStep / steps;
        const s = 10 * Math.pow(t, 3) - 15 * Math.pow(t, 4) + 6 * Math.pow(t, 5);
        const interpQ = startQ.map((q, i) => q + s * (endQ[i] - q));
        window.app.setJointsFromExternal(interpQ);
        currentStep++;
      }, 25);
    });
  }

  async exportGCodeFile() {
    if (this.waypoints.length === 0) {
      alert("Kịch bản đang trống! Hãy thêm ít nhất 1 điểm.");
      return;
    }

    try {
      const res = await fetch('/api/export_gcode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints: this.waypoints })
      });
      const data = await res.json();
      
      const blob = new Blob([data.gcode], { type: 'text/plain' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `parol6_program_${Date.now()}.gcode`;
      a.click();
      URL.revokeObjectURL(url);
    } catch (e) {
      alert("Lỗi khi xuất G-code: " + e.message);
    }
  }

  async streamToUSB() {
    if (this.waypoints.length === 0) {
      alert("Chưa có điểm nào để truyền!");
      return;
    }

    if (!confirm("Bắt đầu truyền toàn bộ kịch bản xuống vi điều khiển STM32 qua USB?")) return;

    try {
      const res = await fetch('/api/stream', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ waypoints: this.waypoints, mode: 'gcode' })
      });
      const data = await res.json();
      if (data.success) {
        alert(`Đã đưa ${data.lines_queued} lệnh vào hàng đợi truyền USB! Theo dõi tiến trình trong Terminal.`);
      } else {
        alert("Lỗi: Không thể truyền dữ liệu. Hãy đảm bảo đã kết nối cổng USB!");
      }
    } catch (e) {
      alert("Lỗi kết nối API: " + e.message);
    }
  }

  async autoAvoidObstacles() {
    const obstacles = window.stlManager ? window.stlManager.getWorkcellObstaclesAABB() : [];
    if (obstacles.length === 0) {
      alert("Bạn chưa import vật thể (Workcell STL) nào để né! Hãy bấm '+ Thêm STL' ở mục Vật Thể Môi Trường.");
      return;
    }

    const motionIndices = [];
    this.waypoints.forEach((wp, idx) => {
      if (wp.joints) motionIndices.push(idx);
    });
    if (motionIndices.length < 2) {
      alert("Cần ít nhất 2 điểm MoveJ/MoveL trong kịch bản để tính đường né vật cản!");
      return;
    }

    const tcpOff = [
      this.robot.tcpOffset.x, this.robot.tcpOffset.y, this.robot.tcpOffset.z,
      this.robot.tcpOffset.roll, this.robot.tcpOffset.pitch, this.robot.tcpOffset.yaw
    ];

    // Estimate each robot link's cross-section radius from its loaded STL bounds (use smallest dim, capped sensibly)
    const linkRadii = [36, 36, 32, 30, 28, 26, 24];
    for (let li = 0; li <= 6; li++) {
      const b = this.robot.stlBounds?.[li];
      if (b) {
        const sc = this.robot.linkOffsets?.[li]?.scale || 1.0;
        const dims = [b.width * sc, b.depth * sc, b.height * sc].sort((a, c) => a - c);
        linkRadii[li] = Math.min(48, Math.max(24, Math.round(dims[0] * 0.45)));
      }
    }

    const btnAvoid = document.getElementById('btn-auto-avoid');
    const origBtnText = btnAvoid ? btnAvoid.innerText : '';
    if (btnAvoid) {
      btnAvoid.disabled = true;
      btnAvoid.innerText = '⏳ Đang quét & né...';
    }

    try {
      const newWaypoints = [];
      let addedAvoidPoints = 0;

      for (let i = 0; i < this.waypoints.length; i++) {
        const wp = this.waypoints[i];
        newWaypoints.push(wp);

        // Find next motion waypoint
        if (!wp.joints) continue;
        let nextMotionIdx = -1;
        for (let k = i + 1; k < this.waypoints.length; k++) {
          if (this.waypoints[k].joints) {
            nextMotionIdx = k;
            break;
          }
        }
        if (nextMotionIdx === -1) continue;

        const wpNext = this.waypoints[nextMotionIdx];
        const res = await fetch('/api/plan_avoidance', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            start_joints: wp.joints,
            goal_joints: wpNext.joints,
            obstacles: obstacles,
            dh_table: this.robot.dhTable,
            tcp_offset: tcpOff,
            safety_margin_mm: 15.0,
            link_radii: linkRadii
          })
        });
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`);
        }
        const data = await res.json();
        if (!data.success) {
          alert(`⚠️ Đoạn #${i + 1} → #${nextMotionIdx + 1}: ${data.message}`);
          return;
        }
        if (!data.direct_clear && data.path && data.path.length > 2) {
          // Insert intermediate avoidance MoveJ waypoints
          for (let p = 1; p < data.path.length - 1; p++) {
            addedAvoidPoints++;
            newWaypoints.push({
              id: Date.now() + addedAvoidPoints,
              type: 'MoveJ',
              joints: data.path[p],
              cartesian: { x: 0, y: 0, z: 0 },
              orientation: { roll: 0, pitch: 0, yaw: 0 },
              speed: 1000,
              note: 'Né vật cản'
            });
          }
        }
      }

      const obsNames = obstacles.map(o => o.name).join(', ');
      if (addedAvoidPoints > 0) {
        this.waypoints = newWaypoints;
        this.updateListUI();
        alert(`✅ Đã quét toàn bộ ${obstacles.length} vật thể (${obsNames}) và tự động sinh thêm ${addedAvoidPoints} điểm MoveJ trung gian né an toàn!`);
      } else {
        alert(`✅ Đã quét toàn bộ ${obstacles.length} vật thể (${obsNames}): đường đi hiện tại thông thoáng, không va chạm!`);
      }
    } catch (err) {
      alert("Lỗi khi tính toán né vật cản: " + err.message);
    } finally {
      if (btnAvoid) {
        btnAvoid.disabled = false;
        btnAvoid.innerText = origBtnText || '🛡️ Né Vật Cản';
      }
    }
  }
}
