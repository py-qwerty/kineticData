"""
BLE IMU Viewer — XIAO nRF52840 Sense (SensorKit firmware)
Interfaz Tkinter con:
  - Gráficas en tiempo real (acelerómetro + giroscopio)
  - Dron 3D que rota con los datos del sensor (filtro complementario)
  - Terminal de datos en vivo
  - Grabación a CSV

Firmware esperado: sensor.ino (arduino/sensor/sensor.ino)
  - Una sola característica BLE con 24 bytes (6 floats)
  - Unidades: acelerómetro en m/s², giroscopio en rad/s
  - Nombre BLE: "SensorKit"

Uso:
    python scripts/ble_imu_viewer.py
"""

import asyncio
import math
import struct
import time
import threading
import csv
from collections import deque
from datetime import datetime

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from bleak import BleakClient, BleakScanner

# ── BLE config (debe coincidir con sensor.ino) ─────────────
# UUID de la característica (debe coincidir con sensor.ino)
IMU_CHAR_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"
DEVICE_NAME   = "SensorKit"

# ── Constantes ──────────────────────────────────────────────
G = 9.80665               # m/s² por g (para convertir a g en la gráfica)
MAX_POINTS = 300           # puntos en gráficas (ventana deslizante)
TERMINAL_MAX_LINES = 200   # líneas máximas en la terminal
COMP_ALPHA = 0.98          # filtro complementario (peso del giroscopio)


# ════════════════════════════════════════════════════════════
#  Geometría del dron (cuadricóptero)
# ════════════════════════════════════════════════════════════

def _build_drone_geometry():
    """
    Genera las piezas 3D de un cuadricóptero centrado en el origen.
    Retorna: list[(faces, color)]  donde faces es lista de polígonos.
    """
    arm_len = 1.0
    arm_w   = 0.08
    arm_h   = 0.04
    body_r  = 0.25
    motor_r = 0.30
    motor_h = 0.06

    parts = []

    # ─ Cuerpo central ─
    s = body_r
    body = np.array([
        [-s, -s, -arm_h], [ s, -s, -arm_h], [ s,  s, -arm_h], [-s,  s, -arm_h],
        [-s, -s,  arm_h], [ s, -s,  arm_h], [ s,  s,  arm_h], [-s,  s,  arm_h],
    ])
    body_faces = [
        [body[0], body[1], body[2], body[3]],
        [body[4], body[5], body[6], body[7]],
        [body[0], body[1], body[5], body[4]],
        [body[1], body[2], body[6], body[5]],
        [body[2], body[3], body[7], body[6]],
        [body[3], body[0], body[4], body[7]],
    ]
    parts.append((body_faces, "#37474f"))

    # ─ 4 brazos + motores ─
    angles = [45, 135, 225, 315]
    front_angles = {45, 315}
    for ang_deg in angles:
        ang = math.radians(ang_deg)
        dx, dy = math.cos(ang), math.sin(ang)
        px, py = -dy * arm_w, dx * arm_w

        cx, cy = dx * arm_len * 0.5, dy * arm_len * 0.5
        arm_verts = np.array([
            [cx - dx*arm_len*0.5 - px, cy - dy*arm_len*0.5 - py, 0],
            [cx + dx*arm_len*0.5 - px, cy + dy*arm_len*0.5 - py, 0],
            [cx + dx*arm_len*0.5 + px, cy + dy*arm_len*0.5 + py, 0],
            [cx - dx*arm_len*0.5 + px, cy - dy*arm_len*0.5 - py, 0],
        ])
        arm_top = arm_verts.copy(); arm_top[:, 2] = arm_h
        arm_bot = arm_verts.copy(); arm_bot[:, 2] = -arm_h
        arm_faces = [
            list(arm_top), list(arm_bot),
            [arm_top[0], arm_top[1], arm_bot[1], arm_bot[0]],
            [arm_top[1], arm_top[2], arm_bot[2], arm_bot[1]],
            [arm_top[2], arm_top[3], arm_bot[3], arm_bot[2]],
            [arm_top[3], arm_top[0], arm_bot[0], arm_bot[3]],
        ]
        parts.append((arm_faces, "#546e7a"))

        # Disco del motor
        tip_x, tip_y = dx * arm_len, dy * arm_len
        n_seg = 10
        circle_top, circle_bot = [], []
        for i in range(n_seg):
            a = 2 * math.pi * i / n_seg
            mx = tip_x + motor_r * math.cos(a)
            my = tip_y + motor_r * math.sin(a)
            circle_top.append([mx, my, arm_h + motor_h])
            circle_bot.append([mx, my, arm_h])
        parts.append(([circle_top], "#e53935" if ang_deg in front_angles else "#1e88e5"))
        parts.append(([circle_bot], "#b0bec5"))

    # ─ Flecha de frente ─
    arrow = [
        [body_r + 0.15, 0, arm_h + 0.01],
        [body_r - 0.05, -0.10, arm_h + 0.01],
        [body_r - 0.05,  0.10, arm_h + 0.01],
    ]
    parts.append(([arrow], "#e53935"))

    return parts


DRONE_PARTS = _build_drone_geometry()


def _rotation_matrix(roll, pitch, yaw):
    """Matriz de rotación 3x3 (Euler ZYX) en radianes."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,            cp*cr   ],
    ])


# ════════════════════════════════════════════════════════════
#  Conexión BLE
# ════════════════════════════════════════════════════════════

class BLEConnection:
    """Gestiona la conexión BLE en un hilo separado."""

    def __init__(self, on_imu_data, on_status):
        self.on_imu_data = on_imu_data   # callback(ax, ay, az, gx, gy, gz)
        self.on_status   = on_status

        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._connected = False
        self._stop_event = asyncio.Event()

    def start_scan_and_connect(self, device_name: str):
        self._stop_event = asyncio.Event()
        self._thread = threading.Thread(
            target=self._run_loop,
            args=(device_name,),
            daemon=True,
        )
        self._thread.start()

    def disconnect(self):
        self._stop_event.set()  # funciona aunque no haya loop todavía
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._stop_event.set)
        # Si está escaneando/conectando, cancelar todas las tareas del loop
        if self._loop and not self._loop.is_closed():
            for task in asyncio.all_tasks(self._loop):
                self._loop.call_soon_threadsafe(task.cancel)

    @property
    def connected(self):
        return self._connected

    def _run_loop(self, device_name: str):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ble_task(device_name))
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.on_status(f"Error: {exc}")
        finally:
            self._loop.close()
            self._connected = False

    async def _ble_task(self, device_name: str):
        try:
            self.on_status(f"Escaneando '{device_name}'...")

            device = await BleakScanner.find_device_by_name(
                device_name, timeout=12.0
            )
            if device is None:
                self.on_status(f"No se encontró '{device_name}'")
                return

            if self._stop_event.is_set():
                return

            self.on_status(f"Encontrado: {device.address} — conectando...")

            async with BleakClient(device, timeout=20.0) as client:
                self._connected = True
                self.on_status(f"Conectado a {device_name}")

                await client.start_notify(IMU_CHAR_UUID, self._imu_cb)
                await self._stop_event.wait()
                await client.stop_notify(IMU_CHAR_UUID)

        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.on_status(f"Error BLE: {exc}")
        finally:
            self._connected = False
            self.on_status("Desconectado")

    def _imu_cb(self, _sender, data: bytearray):
        """
        Payload del firmware: 24 bytes = 6 floats little-endian
        [ax, ay, az] en m/s²   [gx, gy, gz] en rad/s
        """
        if len(data) >= 24:
            ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', data[:24])
            self.on_imu_data(ax, ay, az, gx, gy, gz)


# ════════════════════════════════════════════════════════════
#  Aplicación principal
# ════════════════════════════════════════════════════════════

class IMUViewerApp:
    """
    Layout:
      ┌────────────────────────────────────────────────────┐
      │  [Dispositivo] [Conectar] [Desconectar] [CSV]  RPY │
      │  Accel: ...               Gyro: ...     Muestras   │
      ├─────────────────────────┬──────────────────────────┤
      │  Gráfica Acelerómetro   │    Dron 3D rotando       │
      │  Gráfica Giroscopio     ├──────────────────────────┤
      │                         │    Terminal de datos      │
      ├─────────────────────────┴──────────────────────────┤
      │  Status: Conectado a SensorKit                     │
      └────────────────────────────────────────────────────┘
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SensorKit — BLE IMU Viewer")
        self.root.geometry("1400x800")
        self.root.minsize(1000, 600)

        # Datos (ventana deslizante)
        self.t_data  = deque(maxlen=MAX_POINTS)
        self.ax_data = deque(maxlen=MAX_POINTS)
        self.ay_data = deque(maxlen=MAX_POINTS)
        self.az_data = deque(maxlen=MAX_POINTS)
        self.gx_data = deque(maxlen=MAX_POINTS)
        self.gy_data = deque(maxlen=MAX_POINTS)
        self.gz_data = deque(maxlen=MAX_POINTS)

        self._t0: float | None = None
        self._last_time: float | None = None
        self._recording = False
        self._csv_rows: list[list] = []
        self._sample_count = 0

        # Actitud estimada (filtro complementario)
        self._roll  = 0.0   # rad
        self._pitch = 0.0   # rad
        self._yaw   = 0.0   # rad

        # BLE — una sola característica con 6 floats
        self.ble = BLEConnection(
            on_imu_data=self._on_imu_data,
            on_status=self._on_status,
        )

        self._build_ui()
        self._schedule_plot_update()

    # ── UI ──────────────────────────────────────────────────

    def _build_ui(self):
        # ═══ Barra superior ═══
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Dispositivo:").pack(side=tk.LEFT)
        self.device_var = tk.StringVar(value=DEVICE_NAME)
        ttk.Entry(top, textvariable=self.device_var, width=16).pack(side=tk.LEFT, padx=(4, 10))

        self.connect_btn = ttk.Button(top, text="Conectar", command=self._on_connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.disconnect_btn = ttk.Button(top, text="Desconectar", command=self._on_disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=(4, 20))

        self.record_btn = ttk.Button(top, text="Grabar CSV", command=self._on_record)
        self.record_btn.pack(side=tk.LEFT)
        self.save_btn = ttk.Button(top, text="Guardar CSV", command=self._on_save, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Limpiar", command=self._on_clear).pack(side=tk.LEFT, padx=4)

        self.attitude_label = ttk.Label(top, text="R:  0.0°  P:  0.0°  Y:  0.0°", font=("Menlo", 11))
        self.attitude_label.pack(side=tk.RIGHT, padx=10)

        # ═══ Status bar ═══
        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=4).pack(
            fill=tk.X, side=tk.BOTTOM
        )

        # ═══ Valores numéricos ═══
        vals = ttk.Frame(self.root, padding=(6, 2))
        vals.pack(fill=tk.X)
        self.accel_label = ttk.Label(vals, text="Accel: —", font=("Menlo", 11))
        self.accel_label.pack(side=tk.LEFT, expand=True)
        self.gyro_label = ttk.Label(vals, text="Gyro: —", font=("Menlo", 11))
        self.gyro_label.pack(side=tk.LEFT, expand=True)
        self.samples_label = ttk.Label(vals, text="Muestras: 0  |  0.0 Hz", font=("Menlo", 11))
        self.samples_label.pack(side=tk.RIGHT, padx=10)

        # ═══ Contenido principal ═══
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ─── Columna izquierda: gráficas ───
        left = ttk.Frame(paned)
        paned.add(left, weight=3)

        self.fig_plots = Figure(figsize=(6, 4), dpi=100, facecolor="#f5f5f5")
        self.ax_accel = self.fig_plots.add_subplot(2, 1, 1)
        self.ax_gyro  = self.fig_plots.add_subplot(2, 1, 2)
        self._setup_plot_axes()
        self.fig_plots.tight_layout(pad=2.0)

        self.canvas_plots = FigureCanvasTkAgg(self.fig_plots, master=left)
        self.canvas_plots.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ─── Columna derecha: dron + terminal ───
        right = ttk.PanedWindow(paned, orient=tk.VERTICAL)
        paned.add(right, weight=2)

        # Dron 3D
        drone_frame = ttk.Frame(right)
        right.add(drone_frame, weight=3)

        self.fig_drone = Figure(figsize=(4, 3), dpi=100, facecolor="#eceff1")
        self.ax_drone = self.fig_drone.add_subplot(111, projection="3d")
        self._setup_drone_axes()
        self.fig_drone.tight_layout(pad=0.5)

        self.canvas_drone = FigureCanvasTkAgg(self.fig_drone, master=drone_frame)
        self.canvas_drone.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Terminal
        term_frame = ttk.LabelFrame(right, text=" Terminal ", padding=2)
        right.add(term_frame, weight=2)

        self.terminal = tk.Text(
            term_frame, bg="#1e1e1e", fg="#d4d4d4",
            font=("Menlo", 9), state=tk.DISABLED, wrap=tk.NONE, height=8,
        )
        term_scroll = ttk.Scrollbar(term_frame, orient=tk.VERTICAL, command=self.terminal.yview)
        self.terminal.configure(yscrollcommand=term_scroll.set)
        term_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.terminal.pack(fill=tk.BOTH, expand=True)

        self.terminal.tag_configure("time",  foreground="#6a9955")
        self.terminal.tag_configure("accel", foreground="#569cd6")
        self.terminal.tag_configure("gyro",  foreground="#ce9178")
        self.terminal.tag_configure("info",  foreground="#608b4e")

    def _setup_plot_axes(self):
        ax = self.ax_accel
        ax.set_title("Acelerómetro", fontsize=10, fontweight="bold", loc="left")
        ax.set_ylabel("m/s²")
        ax.grid(True, alpha=0.3)
        self.line_ax, = ax.plot([], [], color="#e53935", lw=1, label="X")
        self.line_ay, = ax.plot([], [], color="#43a047", lw=1, label="Y")
        self.line_az, = ax.plot([], [], color="#1e88e5", lw=1, label="Z")
        ax.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.7)

        ax2 = self.ax_gyro
        ax2.set_title("Giroscopio", fontsize=10, fontweight="bold", loc="left")
        ax2.set_ylabel("rad/s")
        ax2.set_xlabel("Tiempo (s)")
        ax2.grid(True, alpha=0.3)
        self.line_gx, = ax2.plot([], [], color="#e53935", lw=1, label="X")
        self.line_gy, = ax2.plot([], [], color="#43a047", lw=1, label="Y")
        self.line_gz, = ax2.plot([], [], color="#1e88e5", lw=1, label="Z")
        ax2.legend(loc="upper right", fontsize=8, ncol=3, framealpha=0.7)

    def _setup_drone_axes(self):
        ax = self.ax_drone
        ax.set_xlim(-1.8, 1.8)
        ax.set_ylim(-1.8, 1.8)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.set_title("Actitud del dron", fontsize=10, fontweight="bold", loc="left")
        ax.view_init(elev=25, azim=-135)
        ax.set_box_aspect([1, 1, 0.6])
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False

    # ── Terminal ────────────────────────────────────────────

    def _term_write(self, text: str, tag: str = ""):
        def _do():
            self.terminal.config(state=tk.NORMAL)
            if tag:
                self.terminal.insert(tk.END, text, tag)
            else:
                self.terminal.insert(tk.END, text)
            line_count = int(self.terminal.index("end-1c").split(".")[0])
            if line_count > TERMINAL_MAX_LINES:
                self.terminal.delete("1.0", f"{line_count - TERMINAL_MAX_LINES}.0")
            self.terminal.see(tk.END)
            self.terminal.config(state=tk.DISABLED)
        self.root.after_idle(_do)

    # ── Callback BLE (un solo paquete con 6 floats) ─────────

    def _on_imu_data(self, ax_ms2, ay_ms2, az_ms2, gx_rad, gy_rad, gz_rad):
        """
        Recibe un paquete completo del firmware:
        accel en m/s², gyro en rad/s (ya calibrado).
        """
        now = time.time()
        if self._t0 is None:
            self._t0 = now
            self._last_time = now

        t  = now - self._t0
        dt = now - self._last_time
        self._last_time = now
        self._sample_count += 1

        # Almacenar datos raw (en unidades del firmware)
        self.t_data.append(t)
        self.ax_data.append(ax_ms2)
        self.ay_data.append(ay_ms2)
        self.az_data.append(az_ms2)
        self.gx_data.append(gx_rad)
        self.gy_data.append(gy_rad)
        self.gz_data.append(gz_rad)

        # ─ Filtro complementario para actitud ─
        # Convertir accel m/s² → g para calcular ángulos
        ax_g = ax_ms2 / G
        ay_g = ay_ms2 / G
        az_g = az_ms2 / G

        accel_roll  = math.atan2(ay_g, az_g)
        accel_pitch = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2))

        if 0 < dt < 0.5:
            self._roll  = COMP_ALPHA * (self._roll  + gx_rad * dt) + (1 - COMP_ALPHA) * accel_roll
            self._pitch = COMP_ALPHA * (self._pitch + gy_rad * dt) + (1 - COMP_ALPHA) * accel_pitch
            self._yaw  += gz_rad * dt
        else:
            self._roll  = accel_roll
            self._pitch = accel_pitch

        r_deg = math.degrees(self._roll)
        p_deg = math.degrees(self._pitch)
        y_deg = math.degrees(self._yaw)

        # Labels (thread-safe)
        self.root.after_idle(
            self.accel_label.config,
            {"text": f"Accel: X={ax_ms2:+7.3f}  Y={ay_ms2:+7.3f}  Z={az_ms2:+7.3f} m/s²"}
        )
        self.root.after_idle(
            self.gyro_label.config,
            {"text": f"Gyro: X={gx_rad:+6.3f}  Y={gy_rad:+6.3f}  Z={gz_rad:+6.3f} rad/s"}
        )
        self.root.after_idle(
            self.attitude_label.config,
            {"text": f"R:{r_deg:+6.1f}°  P:{p_deg:+6.1f}°  Y:{y_deg:+6.1f}°"}
        )

        # Frecuencia de muestreo
        hz = 1.0 / dt if dt > 0 else 0
        self.root.after_idle(
            self.samples_label.config,
            {"text": f"Muestras: {self._sample_count}  |  {hz:.0f} Hz"}
        )

        # Grabar CSV
        if self._recording:
            self._csv_rows.append([t, ax_ms2, ay_ms2, az_ms2, gx_rad, gy_rad, gz_rad])

        # Terminal (cada 5 muestras para no saturar)
        if self._sample_count % 5 == 0:
            self._term_write(f"[{t:7.2f}s] ", "time")
            self._term_write(
                f"A: {ax_ms2:+7.3f} {ay_ms2:+7.3f} {az_ms2:+7.3f} m/s²  ", "accel"
            )
            self._term_write(
                f"G: {gx_rad:+6.3f} {gy_rad:+6.3f} {gz_rad:+6.3f} rad/s  ", "gyro"
            )
            self._term_write(f"RPY: {r_deg:+6.1f} {p_deg:+6.1f} {y_deg:+6.1f}\n")

    def _on_status(self, msg: str):
        self.root.after_idle(self.status_var.set, msg)
        self._term_write(f"[STATUS] {msg}\n", "info")

    # ── Botones ─────────────────────────────────────────────

    def _on_connect(self):
        name = self.device_var.get().strip()
        if not name:
            messagebox.showwarning("Aviso", "Escribe un nombre de dispositivo")
            return
        self.connect_btn.config(state=tk.DISABLED)
        self.disconnect_btn.config(state=tk.NORMAL)
        self._on_clear()
        self.ble.start_scan_and_connect(name)

    def _on_disconnect(self):
        self.ble.disconnect()
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)

    def _on_clear(self):
        self._t0 = None
        self._last_time = None
        self._sample_count = 0
        self._roll = self._pitch = self._yaw = 0.0
        for d in (self.t_data, self.ax_data, self.ay_data, self.az_data,
                  self.gx_data, self.gy_data, self.gz_data):
            d.clear()
        self.accel_label.config(text="Accel: —")
        self.gyro_label.config(text="Gyro: —")
        self.samples_label.config(text="Muestras: 0  |  0.0 Hz")
        self.attitude_label.config(text="R:  0.0°  P:  0.0°  Y:  0.0°")
        self.terminal.config(state=tk.NORMAL)
        self.terminal.delete("1.0", tk.END)
        self.terminal.config(state=tk.DISABLED)

    def _on_record(self):
        if self._recording:
            self._recording = False
            self.record_btn.config(text="Grabar CSV")
            self.save_btn.config(state=tk.NORMAL)
            self._on_status(f"Grabación detenida — {len(self._csv_rows)} muestras")
        else:
            self._csv_rows = []
            self._recording = True
            self.record_btn.config(text="Parar")
            self.save_btn.config(state=tk.DISABLED)
            self._on_status("Grabando...")

    def _on_save(self):
        if not self._csv_rows:
            messagebox.showinfo("Info", "No hay datos grabados")
            return

        ts = datetime.now().strftime("%Y-%m-%dT%H%M%S")
        path = filedialog.asksaveasfilename(
            initialdir="data",
            initialfile=f"ble_capture_{ts}.csv",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if not path:
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "accel_x", "accel_y", "accel_z",
                             "gyro_x", "gyro_y", "gyro_z"])
            writer.writerows(self._csv_rows)

        self._on_status(f"CSV guardado: {path} ({len(self._csv_rows)} filas)")
        self._csv_rows = []
        self.save_btn.config(state=tk.DISABLED)

    # ── Actualización visual (~16 fps) ──────────────────────

    def _schedule_plot_update(self):
        self._update_plots()
        self._update_drone()
        self._update_job = self.root.after(60, self._schedule_plot_update)

    def _update_plots(self):
        if len(self.t_data) < 2:
            return

        t = list(self.t_data)
        n = min(len(t), len(self.ax_data), len(self.ay_data), len(self.az_data))
        if n < 2:
            return
        t = t[:n]

        # Acelerómetro
        self.line_ax.set_data(t, list(self.ax_data)[:n])
        self.line_ay.set_data(t, list(self.ay_data)[:n])
        self.line_az.set_data(t, list(self.az_data)[:n])

        ax = self.ax_accel
        ax.set_xlim(t[0], t[-1])
        all_a = list(self.ax_data) + list(self.ay_data) + list(self.az_data)
        if all_a:
            a_min, a_max = min(all_a), max(all_a)
            margin = max(0.5, (a_max - a_min) * 0.1)
            ax.set_ylim(a_min - margin, a_max + margin)

        # Giroscopio
        ng = min(n, len(self.gx_data), len(self.gy_data), len(self.gz_data))
        if ng >= 2:
            tg = t[:ng]
            self.line_gx.set_data(tg, list(self.gx_data)[:ng])
            self.line_gy.set_data(tg, list(self.gy_data)[:ng])
            self.line_gz.set_data(tg, list(self.gz_data)[:ng])

            ax2 = self.ax_gyro
            ax2.set_xlim(tg[0], tg[-1])
            all_g = list(self.gx_data) + list(self.gy_data) + list(self.gz_data)
            if all_g:
                g_min, g_max = min(all_g), max(all_g)
                margin = max(0.05, (g_max - g_min) * 0.1)
                ax2.set_ylim(g_min - margin, g_max + margin)

        self.canvas_plots.draw_idle()

    def _update_drone(self):
        ax = self.ax_drone
        ax.cla()
        self._setup_drone_axes()

        R = _rotation_matrix(self._roll, self._pitch, self._yaw)

        for faces, color in DRONE_PARTS:
            transformed = []
            for face in faces:
                rotated = (R @ np.array(face).T).T
                transformed.append(rotated.tolist())
            poly = Poly3DCollection(transformed, alpha=0.85)
            poly.set_facecolor(color)
            poly.set_edgecolor("#263238")
            poly.set_linewidth(0.5)
            ax.add_collection3d(poly)

        self.canvas_drone.draw_idle()

    # ── Cierre ──────────────────────────────────────────────

    def on_close(self):
        if hasattr(self, "_update_job"):
            self.root.after_cancel(self._update_job)
        self.ble.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = IMUViewerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
