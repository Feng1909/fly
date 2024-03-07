import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point
from ius_msgs.msg import Trajectory

class Traj():
    def __init__(self, traj : Trajectory):
        poss = []
        yaws = []
        ts = []
        for i, pos in enumerate(traj.pos):
            poss.append([pos.x, pos.y, pos.z])
            yaws.append(traj.yaw[i])
            ts.append(traj.time[i])

        self._poss = np.array(poss)
        self._yaws = np.array(yaws)
        self._N = self._poss.shape[0]
        if self._N < 2:
            return
        dir = self._poss[1 : ] - self._poss[ : -1]
        self._dir_norm = np.linalg.norm(dir, axis = 1)
        self._u_dir = dir/self._dir_norm[:, np.newaxis]
        self._ts = np.array(ts)

    def sample(self, pos, dt, N):
        # calculate t0 and idx0
        t0 = 0
        idx0 = 0
        
        pos = np.array(pos)
        dl = np.linalg.norm(self._poss - pos, axis=1)
        min_idx = np.argmin(dl)
        if min_idx == 0:
            idx0 = min_idx
            d_v = pos - self._poss[0]
            u_dir = self._u_dir[0]
            u_t =  np.dot(d_v, u_dir) / self._dir_norm[0]
            if u_t < 0:
                t0 = self._ts[0]
            else:
                t0 = u_t * (self._ts[1] - self._ts[0]) + self._ts[0]
        else:
            idx0 = min_idx - 1
            d_v = pos - self._poss[idx0]
            u_dir = self._u_dir[idx0]
            u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
            if u_t > 1:
                idx0 = idx0 + 1
                if idx0 == self._N - 1:
                    t0 = self._ts[-1]
                else:
                    d_v = pos - self._poss[idx0]
                    u_dir = self._u_dir[idx0]
                    u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                    if u_t < 0:
                        t0 = self._ts[idx0]
                    else:
                        t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]
            else:
                t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]

        # sample N points
        ts = np.linspace(t0, t0 + dt * (N - 1), N)
        idx = idx0
        poss = []
        yaws = []
        for t in ts:
            while idx + 1 < self._N and t > self._ts[idx + 1]:
                idx += 1
            if idx == self._N - 1:
                poss.append(self._poss[-1])
                yaws.append(self._yaws[-1])
                continue
            u_dir = self._u_dir[idx]
            u_t = (t - self._ts[idx]) / (self._ts[idx + 1] - self._ts[idx])
            poss.append(self._poss[idx] + u_t * self._dir_norm[idx] * u_dir)
            yaws.append(self._yaws[idx] + u_t * (self._yaws[idx + 1] - self._yaws[idx]))
        
        return np.array(poss), np.array(yaws), ts

# test Traj
if __name__ == "__main__":
    # traj = Trajectory()
    # traj.pos = [Point(0, 0, 0), Point(0, 0, 5), Point(1, 0, 5), Point(2, 0, 5)]
    # traj.yaw = [0, 0, 0, 0]
    # traj.time = [0, 5, 6, 7]
    # traj = Traj(traj)

    traj = Trajectory()

    traj.pos.append(Point(0, 0, 0))
    traj.yaw.append(0)
    traj.time.append(0)
    
    traj.pos.append(Point(0, 0, 5))
    traj.yaw.append(1)
    traj.time.append(5)
    
    traj.pos.append(Point(1, 0, 5))
    traj.yaw.append(2)
    traj.time.append(6)
    
    traj.pos.append(Point(2, 0, 5))
    traj.yaw.append(3)
    traj.time.append(7)
    
    pos, yaw, ts = Traj(traj).sample([0.25642097, 0.01049202, 4.95007753], 0.1, 40)
    print(pos)
    print(ts)
