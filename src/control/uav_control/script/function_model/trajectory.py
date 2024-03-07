import numpy as np
import csv

# Linear interpolation for position and yaw
def _seg_pos_yaw(pos, yaw, time, s_time):
    pos_x = pos[:, 0]
    pos_y = pos[:, 1]
    pos_z = pos[:, 2]
    #
    print("#############\ntrajref_pos_z:",pos_z)
    print("\ntrajref_yaw:",yaw)
    print("\ntrajref_time",time)
    print("\ns_time:",s_time)
    pos_x_seg = np.interp(s_time, time, pos_x)
    pos_y_seg = np.interp(s_time, time, pos_y)
    pos_z_seg = np.interp(s_time, time, pos_z)
    yaw_seg = np.interp(s_time, time, yaw)
    
    pos_seg = np.dstack((pos_x_seg, pos_y_seg, pos_z_seg))[0]
    return pos_seg, yaw_seg

# cubic polynomial interpolation (not used)
def _ploynomial(self, p1, p2, v1, v2, dt):
    a0 = p1
    a1 = v1
    a2 = (3*p2-3*p1-2*v1*dt-v2*dt)/dt/dt
    a3 = (2*p1-2*p2+v2*dt+v1*dt)/dt/dt/dt
    return a0, a1, a2, a3

# calculate the position and velocity with ploynomial (not used)
def _seg_pos_vel(self, poly_coef, dt):
    a0 = poly_coef[:3]
    a1 = poly_coef[3:6]
    a2 = poly_coef[6:9]
    a3 = poly_coef[9:12]

    dtdt = dt*dt
    dtdtdt = dtdt*dt
    pos = a0 + a1*dt + a2*dtdt +a3*dtdtdt
    vel = a1 + 2*a2*dt + 3*a3*dtdt
    return pos, vel

# define the trajectory
class Trajectory_ref():
    def __init__(self):
        #
        self._sample_time = 0
        self._sample_time_idx = 0
        #
        self._pos = np.array([])
        self._yaw = np.array([])
        self._time = np.array([])
        #
        self._N = 0
        
    # load uav_data and calculate ploynomials with dt
    def load_data(self, pos, yaw, time):
        self._pos = pos
        self._yaw = yaw
        self._time = time
        
        # N+1 is the number of trajectory points
        self._N = self._pos.shape[0]-1

    # reset
    def sample_dt_reset(self):
        self._sample_time_idx = 0
        self._sample_time = 0
        
    ##########################################
    # get positions and yaws of sampled_points
    ##########################################
    def sample(self, time_dt, dt, n):
        # sample_time is real time, time_dt == time_now - time_last
        # dt is mpc predicted step_time (0.1)
        # n is the predicted horizon (5)
        # s_time is the time of the sampled point we need to calculate
        self._sample_time += time_dt
        s_time = []
        s_time += [self._sample_time]
        for i in range(n-1):
            s_time += [self._sample_time + dt*(i+1)]
            
        traj_p_seg, traj_yaw_seg = _seg_pos_yaw(self._pos, self._yaw, self._time, s_time)
        return traj_p_seg, traj_yaw_seg 
    
class StateSave():
    def __init__(self, path):
        self._fd = open(path, 'w')
        self._state_writer = csv.writer(self._fd, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        labels = ['t',
                  "p_x", "p_y", "p_z",
                  "v_x", "v_y", "v_z",
                  "q_w", "q_x", "q_y", "q_z",
                  "w_x", "w_y", "w_z",
                  "u_1", "u_2", "u_3", "u_4",
                  "a_x", "a_y", "a_z"]
        self._state_writer.writerow(labels)
    def log(self, t, s, u, a):
        self._state_writer.writerow([t, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11], s[12], u[0], u[1], u[2], u[3], a[0], a[1], a[2]])

    def __del__(self):
        self._fd.close()