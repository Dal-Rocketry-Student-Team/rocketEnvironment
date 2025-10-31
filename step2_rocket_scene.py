import sys
from PyQt6 import QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

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

        # grid
        grid = gl.GLGridItem()
        grid.setSize(10, 10)
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

        self.axes = make_axes(L=2.0)
        for a in self.axes:
            self.addItem(a)
            self.parts.append(a)   # so they get the same rotation in _tick()


        # ---- FINS ----
        self._add_fins(
            n=4,                # 3 for 120°, 4 for 90°
            z_base=0,           # place root leading edge 0.20 m up from body base (tweak)
            span=1,             # radial length
            root_chord=1.5,       # along-Z length against the body
            sweep=-0.75,           # tip forward (nose-ward); negative moves it back
            thickness=0.03)

    
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

def main():
    app = QtWidgets.QApplication(sys.argv)
    view = RocketView()
    view.resize(1000, 700)
    view.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()