import numpy as np
import pyqtgraph as pg 
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector 

class QuadRotor_Viewer:
    def __init__(self):
        self.application = pg.QtGui.QApplication([])
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Flight Simulator')
        self.window.setGeometry(0, 0, 750, 750)
        grid = gl.GLGridItem()
        grid.scale(20, 20, 20)
        self.window.addItem(grid) #Maybe do 3 orthogonal ones?
        self.window.setCameraPosition(distance=200)
        self.window.setBackgroundColor('k')
        self.window.show()
        self.window.raise_()
        self.plot_initialize = False 
        self.points, self.mesh_colors = self.getQuadPoints()
    
    def getQuadPoints(self):
        points = np.array([[-1.0, 1.0, 0.0],
                           [1.0, 1.0, 0.0],
                           [1.0, -1.0, 0.0],
                           [-1.0, -1.0, 0.0],
                           [-1.0, 0.0, 0.0],
                           [-1.5, 0.5, 0.0],
                           [-2.0, 0.0, 0.0],
                           [-1.5, -0.5, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.5, 1.5, 0.0],
                           [0.0, 2.0, 0.0],
                           [-0.5, 1.5, 0.0],
                           [1.0, 0.0, 0.0],
                           [1.5, -0.5, 0.0],
                           [2.0, 0.0, 0.0],
                           [1.5, 0.5, 0.0],
                           [0.0, -1.0, 0.0],
                           [-0.5, -1.5, 0.0],
                           [0.0, -2.0, 0.0],
                           [0.5, -1.5, 0.0]])
        
        scale = 2.5
        points = scale * points

        red = np.array([1.0, 0.0, 0.0, 1])
        blue = np.array([0.0, 0.0, 1.0, 1])
        green = np.array([0.0, 1.0, 0.0, 1.0])
        mesh_colors = np.empty((10, 3, 4), dtype=np.float32)
        mesh_colors[0] = red 
        mesh_colors[1] = red
        mesh_colors[2] = blue 
        mesh_colors[3] = blue 
        mesh_colors[4] = green 
        mesh_colors[5] = green 
        mesh_colors[6] = blue 
        mesh_colors[7] = blue 
        mesh_colors[8] = blue 
        mesh_colors[9] = blue 

        return points.T, mesh_colors
    
    def update(self, t, R): #Remember that viewer operates in a ENU frame. May need to adjust this a little bit if it doesn't do what I want
        #translate and rotate points here
        trans_pts = R @ self.points
        trans_pts = trans_pts + t[:,None]
        mesh = self.pointsToMesh(trans_pts)

        if not self.plot_initialize:
            self.body = gl.GLMeshItem(vertexes=mesh, vertexColors=self.mesh_colors,
                                        drawEdges=True, smooth=False, computeNormals=False)
            self.window.addItem(self.body)
            self.plot_initialize = True
        else:
            self.body.setMeshData(vertexes=mesh, vertexColors=self.mesh_colors)
        
        view_location = Vector(t[0], t[1], t[2]) #in ENU frame
        self.window.opts['center'] = view_location 
        self.application.processEvents()
    
    def pointsToMesh(self, points):
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],
                        [points[0], points[2], points[3]],
                        [points[4], points[5], points[6]],
                        [points[4], points[6], points[7]],
                        [points[8], points[9], points[10]],
                        [points[8], points[10], points[11]],
                        [points[12], points[13], points[14]],
                        [points[12], points[14], points[15]],
                        [points[16], points[17], points[18]],
                        [points[16], points[18], points[19]]])
        return mesh

if __name__=="__main__":
    from scipy.spatial.transform import Rotation
    simulator = QuadRotor_Viewer()
    simulator.update(np.zeros(3), np.eye(3))
    dt = .01
    t = 0.0
    while t < 2 * np.pi:
        ang = t
        T = np.array([10 * np.sin(t), 10 * np.cos(t), 10 * t])
        R = Rotation.from_rotvec(np.array([t, 0, 0])).as_dcm()
        simulator.update(T, R)
        t += dt
    pg.QtGui.QApplication.instance().exec_()