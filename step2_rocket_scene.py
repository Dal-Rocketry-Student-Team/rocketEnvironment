import sys, math, time, serial
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

def euler_zyx_from_R(R):
    """
    Decompose R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    Returns roll, pitch, yaw in radians.
    """
    yaw   = math.atan2(R[1,0], R[0,0])
    pitch = -math.asin(max(-1.0, min(1.0, R[2,0])))
    roll  = math.atan2(R[2,1], R[2,2])
    return roll, pitch, yaw

def make_axes(L=1.0):
    """Return 3 GLLinePlotItems for X (red), Y (green), Z (blue)."""
    x = gl.GLLinePlotItem(pos=np.array([[0,0,0],[L,0,0]], float), color=(1,0,0,1), width=6, antialias=True)
    y = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,L,0]], float), color=(0,1,0,1), width=6, antialias=True)
    z = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,L]], float), color=(0,0,1,1), width=6, antialias=True)
    return [x,y,z]


def make_fin_mesh(R_BODY, *, span = 0.6, root_chord = 0.8, sweep = 0.2, thickness = 0.03):
    """
    A triangluar fin built in local coordinates
    - root along the body at x = R_BODY (z from 0 -> root_chord)
    - tip at x = R_BODY + span, z = root_chord * 0.5 + sweep
    - extrude +-thickness/2 along the Y axis
    Return a pyqt mesh that can by copied.
    """
    t = thickness
    # triangle in xz
    xA, zA = R_BODY, 0.0            # leading root
    xB, zB = R_BODY, root_chord     # trailing root
    xC, zC = (R_BODY + span), (root_chord * 0.5 + sweep)    # swept tip

    V = np.array([
        [xA, +t/2, zA],     # 0 this is the +Y side
        [xB, +t/2, zB],     # 1
        [xC, +t/2, zC],     # 2
        [xA, -t/2, zA],     # 3 -Y side
        [xB, -t/2, zB],     # 4
        [xC, -t/2, zC]      # 5

    ], dtype = float)

    # Faces: 2 big triangles, side wals are rectangles split into triangles
    F = np.array([
        [0, 1, 2],      # +Y face
        [3, 5, 4],      # -Y face (reverse)
        [0, 3, 4], [0, 4, 1],       # side along root edge (A-B)
        [1, 4, 5], [1, 5, 2],       # side along B-C
        [2, 5, 3], [2, 3, 0]        # side along C-A

    ], dtype = np.int32)

    return gl.MeshData(vertexes = V, faces = F)

def rot_to_axis_angle(R):
    # robust axis-angle from rotation matrix
    tr = R[0,0] + R[1,1] + R[2,2]
    cosang = max(-1.0, min(1.0, 0.5*(tr - 1.0)))
    ang = math.acos(cosang)
    if ang < 1e-9:
        return 0.0, np.array([1.0, 0.0, 0.0])
    rx = R[2,1] - R[1,2]
    ry = R[0,2] - R[2,0]
    rz = R[1,0] - R[0,1]
    axis = np.array([rx, ry, rz], float)
    axis /= (np.linalg.norm(axis) + 1e-12)
    return ang, axis


class SerialWorker(QtCore.QObject):
    newQuat = QtCore.pyqtSignal(float, float, float, float)          # w,x,y,z
    newIMU  = QtCore.pyqtSignal(float, float, float, float, float, float)  # gx,gy,gz, ax,ay,az
    status  = QtCore.pyqtSignal(str)
    stopped = False

    def __init__(self, port: str = "COM7", baud: int = 115200):
        super().__init__()
        self.port = port
        self.baud = baud

    @QtCore.pyqtSlot()
    def run(self):
        try:
            self.status.emit(f"Opening {self.port} @ {self.baud}…")
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                self.status.emit("Connected.")
                while not self.stopped:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="ignore").strip()
                    # Expect: IMU,t,q0,q1,q2,q3,gx,gy,gz,ax,ay,az
                    if not line or not line.startswith("IMU"):
                        continue
                    try:
                        parts = [p.strip() for p in line.split(",")]
                        if len(parts) < 12:
                            continue
                        # q0..q3 (assumed q0 = w)
                        w = float(parts[2]); x = float(parts[3]); y = float(parts[4]); z = float(parts[5])
                        gx = float(parts[6]); gy = float(parts[7]); gz = float(parts[8])
                        ax = float(parts[9]); ay = float(parts[10]); az = float(parts[11])
                        self.newQuat.emit(w, x, y, z)
                        self.newIMU.emit(gx, gy, gz, ax, ay, az)
                    except Exception:
                        # ignore malformed lines
                        continue
        except Exception as e:
            self.status.emit(f"Serial error: {e}")


class RocketView(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Step 2: Create 3D view")
        self.setCameraPosition(distance=8, elevation=30, azimuth=40)

        # the grid on the ground (XZ plane). Z is up in pyqtgraph.opengl
        grid = gl.GLGridItem()
        grid.setSize(10, 10)
        grid.scale(1, 1, 1)
        self.addItem(grid)
        
        # Building a simple rocket with cylinder, cone, and tail
        self.parts = []

        L_BODY = 4.0
        L_CONE = 1.0
        L_NOZZLE = 0.6
        self.R_BODY = 0.3

        body_md = gl.MeshData.cylinder(rows=20, cols=40, radius=[self.R_BODY,self.R_BODY], length=L_BODY)
        body = gl.GLMeshItem(meshdata=body_md, smooth=True, shader="shaded", drawFaces=True)
        body.translate(0, 0, 0)     # centered at origin, length runs along Z axis
        self.addItem(body)
        self.parts.append(body)

        cone_md = gl.MeshData.cylinder(rows=20, cols=40, radius=[self.R_BODY, 0.001], length=L_CONE)
        cone = gl.GLMeshItem(meshdata=cone_md, smooth=True, shader='shaded', drawFaces=True)
        cone.translate(0, 0, L_BODY)    # on top of body
        self.addItem(cone)
        self.parts.append(cone)

        nozzle_md = gl.MeshData.cylinder(rows=20, cols=40, radius=[0.4, 0.1], length=L_NOZZLE)
        nozzle = gl.GLMeshItem(meshdata=nozzle_md, smooth=True, shader='shaded', drawFaces=True)
        nozzle.translate(0, 0, -L_NOZZLE)   # bottom-ish
        self.addItem(nozzle)
        self.parts.append(nozzle)

        # ---- FINS ----
        self._add_fins(
            n=4,                # 3 for 120°, 4 for 90°
            z_base=0,           # place root leading edge 0.20 m up from body base (tweak)
            span=1,             # radial length
            root_chord=1.5,       # along-Z length against the body
            sweep=-0.75,           # tip forward (nose-ward); negative moves it back
            thickness=0.03)

        self.axes = make_axes(L=2.0)
        for a in self.axes:
            self.addItem(a)
            self.parts.append(a)   # so they get the same rotation in _tick()

        # --- HUD state ---
        self._last_pkt_t = 0.0              # when the last serial packet was received
        self._fps_frames = 0
        self._fps_t0 = time.time()
        self._fps = 0.0

        # --- HUD label (overlay) ---
        self.hud = QtWidgets.QLabel(self)   # parent = GL view ⇒ draws on top
        self.hud.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.hud.setStyleSheet(
            "background: rgba(0,0,0,120);"
            "color: white;"
            "padding: 1px 9px;"
            "border-radius: 6px;"
            "font-family: Consolas, 'Courier New', monospace;"
            "font-size: 16px;"
        )
        self.hud.move(10, 10)               # top-left corner
        self.hud.setText("HUD…")
        self.hud.raise_()                   # ensure it stays above the GL canvas

        self._hud_last = ""  # track last text so we only resize when it changes

        # make it bigger
        self.hud.setStyleSheet(
            "background: rgba(0,0,0,120);"
            "color: white;"
            "padding: 8px 10px;"
            "border-radius: 6px;"
            "font-family: Consolas, 'Courier New', monospace;"
            "font-size: 16px;"          # <- larger
            "line-height: 1.2;"
        )

        # place it once now
        self._place_hud()

        self.current_R = np.eye(3)   # latest from serial
        self.draw_R    = np.eye(3)   # what we've drawn so far
        self._have_init_R = False    # to avoid a jump on the first packet
        self.smooth_alpha = 0.25     # 0..1, higher = snappier

        self._gyro = None
        self._accel = None

        # --- start a ~60 FPS frame timer to refresh the HUD (and, later, orientation) ---
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(16)                 # ~60 Hz
        self._timer.timeout.connect(self._frame_update)
        self._timer.start()


    
    def _add_fins(self, *, n = 4, z_base = 0.2, span = 0.6, root_chord = 0.8, sweep = 0.2, thickness = 0.03):
        md = make_fin_mesh(self.R_BODY, span=span, root_chord=root_chord, sweep=sweep, thickness=thickness)

        for k in range(n):
            fin = gl.GLMeshItem(meshdata = md, smooth = False, shader = 'shaded', drawFaces = True)
            fin.rotate(k * (360.0/n), 0, 0, 1)      # spinning around z thru the rocket center
            fin.translate(0, 0, z_base)             # moving up or down the body
            # (Optional) sink slightly into body to hide seam:
            # fin.translate(-0.005, 0, 0)
            self.addItem(fin)
            self.parts.append(fin)

    def _frame_update(self):
        # --- Euler from current rotation (identity until you feed data) ---
        r, p, y = euler_zyx_from_R(self.current_R)
        r_deg, p_deg, y_deg = map(math.degrees, (r, p, y))

        # --- FPS (smoothed every ~0.5 s) ---
        self._fps_frames += 1
        now = time.time()
        dt = now - self._fps_t0
        if dt >= 0.5:
            self._fps = self._fps_frames / dt
            self._fps_frames = 0
            self._fps_t0 = now

        # --- age since last packet (will be NaN until you stamp it) ---
        age_ms = (now - self._last_pkt_t) * 1000.0 if self._last_pkt_t > 0 else float('nan')

        g_line = ""
        if self._gyro is not None:
            gx, gy, gz = self._gyro
            g_line += f" | ω(dps): {gx:+6.1f} {gy:+6.1f} {gz:+6.1f}"
        if self._accel is not None:
            ax, ay, az = self._accel
            g_line += f" | a(g): {ax:+5.2f} {ay:+5.2f} {az:+5.2f}"


        # then build the text
        text = (
            f"RPY  : {r_deg:6.1f}°  {p_deg:6.1f}°  {y_deg:6.1f}°\n"
            f"FPS  : {self._fps:5.1f}    PktΔ : {0 if math.isnan(age_ms) else int(age_ms)} ms{g_line}"
        )

        if text != self._hud_last:
            self._hud_last = text
            self.hud.setText(text)
            self._place_hud()   # adjusts size + re-anchors

        # --- smooth toward target orientation ---
        R_target = self.current_R
        R_interp = self.smooth_alpha * R_target + (1.0 - self.smooth_alpha) * self.draw_R

        # re-orthonormalize (polar decomposition via SVD) so it stays a true rotation
        U, S, Vt = np.linalg.svd(R_interp)
        R_smooth = U @ Vt
        if np.linalg.det(R_smooth) < 0:
            U[:, -1] *= -1
            R_smooth = U @ Vt

        # --- rotate meshes by the delta from last frame ---
        R_delta = R_smooth @ self.draw_R.T
        ang, axis = rot_to_axis_angle(R_delta)
        if ang > 1e-6:
            deg = math.degrees(ang)
            for item in self.parts:          # body, cone, nozzle, fins, axes
                item.rotate(deg, axis[0], axis[1], axis[2])

        self.draw_R = R_smooth

    def _mark_packet(self):
        self._last_pkt_t = time.time()

    def _quat_to_rotmat(self, w, x, y, z):
        n = max(1e-12, math.sqrt(w*w + x*x + y*y + z*z))
        w, x, y, z = w/n, x/n, y/n, z/n
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
            [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
            [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)]
        ], dtype=float)

    def set_quaternion(self, w, x, y, z):
        R = self._quat_to_rotmat(w, x, y, z)
        self.current_R = R
        if not self._have_init_R:
            self.draw_R = R.copy()
            self._have_init_R = True
        self._last_pkt_t = time.time()

    def set_imu(self, gx, gy, gz, ax, ay, az):
        self._gyro = (gx, gy, gz)
        self._accel = (ax, ay, az)
        self._last_pkt_t = time.time()

    def _place_hud(self, corner="tl"):
        """Anchor HUD to a window corner."""
        m = 12  # margin
        self.hud.adjustSize()  # ensure label fits its text
        if corner == "tl":
            x, y = m, m
        elif corner == "tr":
            x, y = self.width() - self.hud.width() - m, m
        elif corner == "bl":
            x, y = m, self.height() - self.hud.height() - m
        else:  # "br"
            x, y = self.width() - self.hud.width() - m, self.height() - self.hud.height() - m
        self.hud.move(int(x), int(y))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._place_hud()  # keep it anchored when the window changes size


def main():
    app = QtWidgets.QApplication(sys.argv)
    view = RocketView()
    view.resize(1000, 700)
    view.show()

    # ---- Serial wiring ----
    thread = QtCore.QThread()
    worker = SerialWorker(port="COM7", baud=115200)
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    worker.newQuat.connect(view.set_quaternion)
    worker.newIMU.connect(view.set_imu)
    worker.status.connect(lambda s: print("[SERIAL]", s))
    thread.start()

    # For a clean shutdown
    def _cleanup():
        worker.stopped = True
        thread.quit()
        thread.wait(1000)
    
    app.aboutToQuit.connect(_cleanup)

    sys.exit(app.exec())

if __name__ == "__main__":
    main()