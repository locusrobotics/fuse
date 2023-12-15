import numpy as np
from scipy.spatial.transform import Rotation as R

# From ROS Calibrator on Logitech C920 (Oscar's) at 640x480
# D = [0.13281739520782995, -0.17255676937880005, -0.0036963860577237523, -0.00884659526000406, 0.0]
# K = [622.2066360931567, 0.0, 315.6497225093459, 0.0, 623.201615897975, 239.80322845004648, 0.0, 0.0, 1.0]
# R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P = [638.3447875976562, 0.0, 310.2906045722684, 0.0, 0.0, 643.107177734375, 237.80861559081677, 0.0, 0.0, 0.0, 1.0, 0.0]

class PinholeCameraProjection():
    def __init__(self,):
        # Define Size
        self.w = 640
        self.h = 480

        # Calib Parameters
        self.fx = 638.34478759765620
        self.fy = 643.10717773437500
        self.cx = 310.29060457226840
        self.cy = 237.80861559081677

        # Dist Parameters
        self.k1 =  0.13281739520782995
        self.k2 = -0.17255676937880005
        self.p1 = -0.0036963860577237523
        self.p2 = -0.00884659526000406
        self.k3 = 0.0

        # Make Matrix
        self.K = np.eye(4)
        self.K[0,0] = self.fx
        self.K[1,1] = self.fy
        self.K[0,2] = self.cx
        self.K[1,2] = self.cy

        # Define 4x4 Array of 3D points at corners of a square (e.g. ARTag)
        self.X = np.zeros((4,4))
        self.X[0,:] = np.array([-1, -1, 0, 1])
        self.X[1,:] = np.array([-1,  1, 0, 1])
        self.X[2,:] = np.array([ 1, -1, 0, 1])
        self.X[3,:] = np.array([ 1,  1, 0, 1])
        # print(self.X)

        # Define 8x4 Array of 3D points (e.g. 2 AR Tags)
        self.X2 = np.zeros((8,4))
        scale = 3.0
        lps = (1.0 * scale)/2 
        lpg = 0.08 * scale
        self.X2[0,:] = np.array([       -lps, -lps, 0, 1])
        self.X2[1,:] = np.array([       -lps,  lps, 0, 1])
        self.X2[2,:] = np.array([        lps, -lps, 0, 1])
        self.X2[3,:] = np.array([        lps,  lps, 0, 1])
        self.X2[4,:] = np.array([2*lps + lpg, -lps, 0, 1])
        self.X2[5,:] = np.array([2*lps + lpg,  lps, 0, 1])
        self.X2[6,:] = np.array([  lps + lpg, -lps, 0, 1])
        self.X2[7,:] = np.array([  lps + lpg,  lps, 0, 1])
        print(self.X2)
        
        # Define T for 
        self.T = np.eye(4)
        self.T[0:3,0:3] = R.from_quat([0, -0.3826834, 0, 0.9238795]).as_matrix()
        self.T[2,3] = 10
        print(self.T)

        # Define Camera Position (identity)
        self.Xc = np.eye(4)

    def print_points(self, pts2d, pts3d):
        print(f"3D:\n{pts3d[:,0:3]}")
        print(f"2D:\n{pts2d[:,0:2]}")

    def project_points(self, camPos, pts3d):
        x = np.matmul(self.K, np.matmul(camPos, pts3d.transpose())).transpose()
        x[:,0]/=x[:,2]
        x[:,1]/=x[:,2]
        x[:,2]/=x[:,2]
        return x
    
    def project_points_snavelly(self, camPos, pts3d):
        Ks = np.eye(4)
        Ks[0,0] = self.fx
        Ks[1,1] = self.fx
        Ks[0,2] = 0
        Ks[1,2] = 0

        x = np.matmul(camPos, pts3d.transpose()).transpose()
        x[:,0]/=-x[:,2]
        x[:,1]/=-x[:,2]
        x[:,2]/=-x[:,2]

        r2 = x*x
        d = 1.0 + r2*(self.k1+self.k2*r2)
        x = Ks[0,0]*d*x
        return x

    def plot(self, pts2d, pts3d, is_snavelly=False):

        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(1,2,1, projection='3d')

        ax.scatter(pts3d[:,0], pts3d[:,1], pts3d[:,2])
        ax = fig.add_subplot(1,2,2)

        ax.scatter(pts2d[:,0], pts2d[:,1])
        if is_snavelly:
            ax.set_xlim([-self.w/2, self.w/2])
            ax.set_ylim([-self.h/2, self.h/2])
        else:
            ax.set_xlim([0, self.w])
            ax.set_ylim([0, self.h])

        plt.show()

    def project_poses(self, pts3d):

        Xc = np.eye(4)
        for i in range(-2,3,1):

            Xc[0, 3] = i
            Xc[1, 3] = i

            if i == 0:
                Xc[0:3, 0:3] = R.from_quat([ 0, 0.0871558, 0, 0.9961947 ]).as_matrix()
                print(Xc)

            x = self.project_points(Xc, pts3d)

    def Optimization(self):
        print("Points for Optimization")
        print(f"fx = {self.fx}")
        print(f"fy = {self.fy}")   
        print(f"cx = {self.cx}")   
        print(f"cy = {self.cy}")   
        X = np.matmul(self.T, self.X.transpose()).transpose()
        x = self.project_points(np.eye(4), X)
        self.print_points(x, X)
        self.plot(x, X)

    def OptimizationSnavelly(self):
        print("Points for OptimizationSnavelly")
        print(f"f = {self.fx}")
        print(f"k1 = {self.k1}")   
        print(f"k2 = {self.k2}")   
        X = np.matmul(self.T, self.X.transpose()).transpose()
        x = self.project_points_snavelly(np.eye(4), X)
        self.print_points(x, X)
        self.plot(x, X, True)

    def OptimizationScaledMarker(self):
        print("Points for OptimizationScaledMarker")
        print(f"fx = {self.fx}")
        print(f"fy = {self.fy}")   
        print(f"cx = {self.cx}")   
        print(f"cy = {self.cy}")

        # Scale X
        s = 0.25
        Y = s * self.X # Scale X
        Y[:,3] = 1.0
        
        # Define T
        self.T = np.eye(4)
        self.T[0:3,0:3] = R.from_quat([0, -0.3826834, 0, 0.9238795]).as_matrix()
        self.T[2,3] = 10

        # Apply T
        Y = np.matmul(self.T, Y.transpose()).transpose()
        
        x = self.project_points(np.eye(4), Y)
        self.print_points(x, Y)
        self.plot(x, Y)

    def OptimizationPoints(self):
        print("Points for OptimizationPoints")
        print(f"fx = {self.fx}")
        print(f"fy = {self.fy}")   
        print(f"cx = {self.cx}")   
        print(f"cy = {self.cy}")

        # Define T
        self.T = np.eye(4)
        self.T[0:3,0:3] = R.from_quat([0, -0.3826834, 0, 0.9238795]).as_matrix()
        self.T[2,3] = 10

        # Apply T
        X2 = np.matmul(self.T, self.X2.transpose()).transpose()
        
        x = self.project_points(np.eye(4), X2)
        self.print_points(x, self.X2)
        self.plot(x, self.X2)

    def MultiViewOptimization(self):
        print("Points for MultiViewOptimization")
        print(f"fx = {self.fx}")
        print(f"fy = {self.fy}")   
        print(f"cx = {self.cx}")   
        print(f"cy = {self.cy}")

        # Define T
        self.T = np.eye(4)
        self.T[0:3,0:3] = R.from_quat([0, -0.3826834, 0, 0.9238795]).as_matrix()
        self.T[2,3] = 10

        # Apply T
        pts3d = np.matmul(self.T, self.X.transpose()).transpose()
        print(self.X)

        Xc = np.eye(4)
        for i in range(-2,3,1):
            
            print(f"\n{i+2}: \n")

            Xc[0, 3] = i
            Xc[1, 3] = i

            if i == 0:
                Xc[0:3, 0:3] = R.from_quat([ 0, 0.0871558, 0, 0.9961947 ]).as_matrix()

            x = self.project_points(Xc, pts3d)
            self.print_points(x, pts3d)
            self.plot(x, pts3d)

if __name__ == '__main__':
    tests = PinholeCameraProjection()

    # tests.Optimization()
    tests.OptimizationSnavelly()
    # tests.OptimizationScaledMarker()
    # tests.OptimizationPoints()
    # tests.MultiViewOptimization()
