/**
 * USB Serial UI Manager (PAROL6-python-API/app)
 */
class SerialUI {
  constructor() {
    this.portSelect = document.getElementById('serial-port-select');
    this.baudSelect = document.getElementById('serial-baudrate-select');
    this.btnConnect = document.getElementById('btn-connect-usb');
    this.btnRefresh = document.getElementById('btn-refresh-ports');
    this.btnEStop = document.getElementById('btn-estop');
    this.terminal = document.getElementById('serial-terminal');
    this.manualInput = document.getElementById('manual-gcode-input');
    this.btnManualSend = document.getElementById('btn-manual-send');
    this.btnClearTerm = document.getElementById('btn-clear-term');

    this.isConnected = false;
    this.setupListeners();
    this.refreshPorts();
  }

  setupListeners() {
    this.btnRefresh.onclick = () => this.refreshPorts();
    this.btnConnect.onclick = () => this.toggleConnect();
    this.btnEStop.onclick = () => this.triggerEStop();
    
    this.btnManualSend.onclick = () => this.sendManualCommand();
    this.manualInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') this.sendManualCommand();
    });

    this.btnClearTerm.onclick = () => {
      this.terminal.innerHTML = '';
    };
  }

  async refreshPorts() {
    try {
      const res = await fetch('/api/ports');
      const data = await res.json();
      this.portSelect.innerHTML = '';

      if (data.ports.length === 0) {
        this.portSelect.innerHTML = '<option value="">Không tìm thấy cổng COM</option>';
        return;
      }

      data.ports.forEach(p => {
        const opt = document.createElement('option');
        opt.value = p.port;
        opt.innerText = `${p.port} (${p.description})`;
        if (p.is_stm32) opt.selected = true;
        this.portSelect.appendChild(opt);
      });
    } catch (e) {
      console.warn("Could not query ports:", e);
    }
  }

  async toggleConnect() {
    if (this.isConnected) {
      try {
        await fetch('/api/disconnect', { method: 'POST' });
        this.isConnected = false;
        this.btnConnect.innerText = "Kết Nối";
        this.btnConnect.classList.remove('btn-danger');
        this.btnConnect.classList.add('btn-primary');
        this.logMessage("SYS", "Đã ngắt kết nối cổng USB");
      } catch (e) {
        alert("Lỗi ngắt kết nối: " + e.message);
      }
    } else {
      const port = this.portSelect.value;
      const baud = parseInt(this.baudSelect.value);
      if (!port) {
        alert("Vui lòng chọn cổng COM!");
        return;
      }

      try {
        const res = await fetch('/api/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ port: port, baudrate: baud })
        });
        const data = await res.json();
        if (data.success) {
          this.isConnected = true;
          this.btnConnect.innerText = "Ngắt Kết Nối";
          this.btnConnect.classList.remove('btn-primary');
          this.btnConnect.classList.add('btn-danger');
          this.logMessage("SYS", `Đã kết nối thành công tới ${port} @ ${baud} bps`);
        } else {
          alert("Không thể kết nối cổng: " + (data.error || "Lỗi không xác định"));
        }
      } catch (e) {
        alert("Lỗi gọi API: " + e.message);
      }
    }
  }

  async triggerEStop() {
    try {
      await fetch('/api/emergency_stop', { method: 'POST' });
      this.logMessage("ERR", "⛔ DỪNG KHẨN CẤP (E-STOP) ĐÃ ĐƯỢC KÍCH HOẠT!");
    } catch (e) {
      alert("Lỗi E-Stop: " + e.message);
    }
  }

  async sendManualCommand() {
    const text = this.manualInput.value.trim();
    if (!text) return;

    this.logMessage("TX", text);
    this.manualInput.value = '';

    try {
      await fetch('/api/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: text })
      });
    } catch (e) {
      this.logMessage("ERR", "Lỗi gửi lệnh: " + e.message);
    }
  }

  logMessage(direction, msg) {
    const line = document.createElement('div');
    line.className = 'term-line';
    const time = new Date().toTimeString().split(' ')[0];
    
    let dirClass = direction.toLowerCase();
    line.innerHTML = `
      <span class="term-time">[${time}]</span>
      <span class="term-dir ${dirClass}">[${direction}]</span>
      <span class="term-msg">${msg}</span>
    `;

    this.terminal.appendChild(line);
    this.terminal.scrollTop = this.terminal.scrollHeight;
  }
}
