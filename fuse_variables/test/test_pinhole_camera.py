import numpy as np

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

        # Make Matrix
        self.K = np.eye(4)
        self.K[0,0] = self.fx
        self.K[1,1] = self.fy
        self.K[0,2] = self.cx
        self.K[1,2] = self.cy

        # Define 8x4 Array of 3D points at corners of a cube (e.g. ARTag)
        self.X = np.zeros((8,4))
        self.X[0,:] = np.array([-1, -1, -1, 1])
        self.X[1,:] = np.array([-1, -1,  1, 1])
        self.X[2,:] = np.array([-1,  1, -1, 1])
        self.X[3,:] = np.array([-1,  1,  1, 1])
        self.X[4,:] = np.array([ 1, -1, -1, 1])
        self.X[5,:] = np.array([ 1, -1,  1, 1])
        self.X[6,:] = np.array([ 1,  1, -1, 1])
        self.X[7,:] = np.array([ 1,  1,  1, 1])

        # Offset on Z to move in front of camera
        self.X[:,2] += 10

    def print_calib(self):   
        print(f"fx = {self.fx}")
        print(f"fy = {self.fy}")   
        print(f"cx = {self.cx}")   
        print(f"cy = {self.cy}")   

    def print_points(self, pts2d, pts3d):
        print(pts3d[:,0:3])
        print(pts2d[:,0:2])

    def project_points(self, camPos, pts3d):
        x = np.matmul(self.K, np.matmul(camPos, pts3d.transpose())).transpose()
        x[:,0]/=x[:,2]
        x[:,1]/=x[:,2]
        x[:,2]/=x[:,2]
        return x

    def plot(self, pts2d, pts3d):

        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(1,2,1, projection='3d')

        ax.scatter(pts3d[:,0], pts3d[:,1], pts3d[:,2])
        ax = fig.add_subplot(1,2,2)

        ax.scatter(pts2d[:,0], pts2d[:,1], pts2d[:,2])
        ax.set_xlim([0, self.w])
        ax.set_ylim([0, self.h])
        plt.show()

    def project_and_print(self):
        x = self.project_points(np.eye(4), self.X)
        self.print_calib()
        self.print_points(x, self.X)
        self.plot(x, self.X)
        
    def FuseProjectionOptimization(self):
        print("Points for FuseProjectionOptimization")
        self.project_and_print()
        
    def ProjectionOptimization(self):
        print("Points for ProjectionOptimization")
        self.project_and_print()

    def PerPointProjectionOptimization(self):
        print("Points for PerPointProjectionOptimization")
        self.project_and_print()

if __name__ == '__main__':
    tests = PinholeCameraProjection()

    # fuseProjectionOptimization Test
    tests.FuseProjectionOptimization()
    tests.ProjectionOptimization()
    tests.PerPointProjectionOptimization()

