import numpy as np
import casadi as ca
import yaml

import time

# Quaternion Multiplication
def quat_mult(q1,q2):
    ans = ca.vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return ca.vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

def RK4(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        k1 = DT*f_c(X1,        U)
        k2 = DT*f_c(X1+0.5*k1, U)
        k3 = DT*f_c(X1+0.5*k2, U)
        k4 = DT*f_c(X1+k3,     U)
        X1 = X1+(k1+2*k2+2*k3+k4)/6
    # F = ca.Function('F', [X0, U], [X1] ,['X0', 'U'], ['X1'])
    return X1

def EulerIntegral(f_c:ca.Function, X0, U, dt, M:int):
    DT = dt/M
    X1 = X0
    for _ in range(M):
        X1 = X1 + DT*f_c(X1, U)
    
    return X1

def constrain(a, lb, ub):
    if a<lb:
        a=lb
    if a>ub:
        a=ub
    return a

class PID(object):
    def __init__(self, p:float, i:float, d:float, sum_lim:float, out_lim:float):
        self._p = p
        self._i = i
        self._d = d
        
        self._sum_lim = sum_lim
        self._out_lim = out_lim
        
        self._sum = 0
        self._last_e = 0
        
    def update(self, e:float):
        self._sum += e
        self._sum = constrain(self._sum, -self._sum_lim, self._sum_lim)
        out = self._p*e + self._i*self._sum + self._d*(e-self._last_e)
        out = constrain(out, -self._out_lim, self._out_lim)
        self._last_e = e
        
        return out    

class QuadrotorSimpleModel(object):
    def __init__(self, cfg_f):
        
        self._G = 9.81
        self._D = np.diag([0.3, 0.3, 0.3])
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 4
        self._omega_z_max = 1
        self._a_z_max = 17
        self._a_z_min = 0

        with open(cfg_f, 'r') as f:
            self._cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if 'omega_xy_max' in self._cfg:
            self._omega_xy_max = self._cfg['omega_xy_max']
        if 'omega_z_max' in self._cfg:
            self._omega_z_max = self._cfg['omega_z_max']
        if 'G' in self._cfg:
            self._G = self._cfg['G']
        if 'a_z_min' in self._cfg:
            self._a_z_min = self._cfg['a_z_min']
        if 'a_z_max' in self._cfg:
            self._a_z_max = self._cfg['a_z_max']
      
        print("_v_xy_max, _v_z_max: ", self._v_xy_max, self._v_z_max)
        print("_omega_xy_max, _omega_z_max: ", self._omega_xy_max, self._omega_z_max)
        print("_a_z_max, _a_z_min: ", self._a_z_max, self._a_z_min)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1]
        self._U_lb = [self._a_z_min, -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._U_ub = [self._a_z_max,  self._omega_xy_max,  self._omega_xy_max,  self._omega_z_max]
        
    def dynamics(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        
        az_B = ca.SX.sym('az_B')
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz)
        U = ca.vertcat(az_B, wx, wy, wz)

        fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * az_B - fdrag[0],
            2 * (qy * qz - qw * qx) * az_B - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) * az_B - self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy)
        )

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        
        X1 = RK4(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

class QuadrotorModel(object):
    def __init__(self, cfg_f):
        
        self._m = 1.0         # total mass
        self._arm_l = 0.23    # arm length
        self._c_tau = 0.0133  # torque constant
        
        self._G = 9.81
        self._J = np.diag([0.01, 0.01, 0.02])     # inertia
        self._J_inv = np.linalg.inv(self._J)
        self._D = np.diag([0.6, 0.6, 0.6])
        
        self._v_xy_max = ca.inf
        self._v_z_max = ca.inf
        self._omega_xy_max = 5
        self._omega_z_max = 1
        self._T_max = 4.179
        self._T_min = 0

        self.load(cfg_f)
        
        self._X_lb = [-ca.inf, -ca.inf, -ca.inf,
                      -self._v_xy_max, -self._v_xy_max, -self._v_z_max,
                      -1,-1,-1,-1,
                      -self._omega_xy_max, -self._omega_xy_max, -self._omega_z_max]
        self._X_ub = [ca.inf, ca.inf, ca.inf,
                      self._v_xy_max, self._v_xy_max, self._v_z_max,
                      1,1,1,1,
                      self._omega_xy_max, self._omega_xy_max, self._omega_z_max]

        self._U_lb = [self._T_min, self._T_min, self._T_min, self._T_min]
        self._U_ub = [self._T_max, self._T_max, self._T_max, self._T_max]

    def load(self, cfg_f):
      with open(cfg_f, 'r') as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        
        if "mass" in cfg:
          self._m = cfg["mass"]
        else:
          print("No mass specified in " + cfg_f)
        
        if "arm_length" in cfg:
          self._arm_l = cfg["arm_length"]
        else:
          print("No arm length specified in " + cfg_f)

        if "inertia" in cfg:
          self._J = np.diag(cfg["inertia"])
          self._J_inv = np.linalg.inv(self._J)
        else:
          print("No inertia specified in " + cfg_f)
        
        if "torque_coeff" in cfg:
          self._c_tau = cfg["torque_coeff"]
        else:
          print("No torque coefficient specified in " + cfg_f)
        
        if "drag_coeff" in cfg:
          self._D = np.diag(cfg["drag_coeff"])
        else:
           print("No drag coefficient specified in " + cfg_f)
        
        if "v_xy_max" in cfg:
          self._v_xy_max = cfg["v_xy_max"]
        else:
          self._v_xy_max = ca.inf
        if "v_z_max" in cfg:
          self._v_z_max = cfg["v_z_max"]
        else:
          self._v_z_max = ca.inf
        
        if "omega_xy_max" in cfg:
          self._omega_xy_max = cfg["omega_xy_max"]
        else:
          print("No max angular velocity xy specfied in " + cfg_f)
        if "omega_z_max" in cfg:
          self._omega_z_max = cfg["omega_z_max"]
        else:
          print("No max angular velocity z specfied in " + cfg_f)
        
        if "thrust_min" in cfg:
          self._T_min = cfg["thrust_min"]
        else:
          print("No min thrust specified in " + cfg_f)
        if "thrust_max" in cfg:
          self._T_max = cfg["thrust_max"]
        else:
          print("No max thrust specified in " +cfg_f)

#
#   T1    T3
#     \  /
#      \/
#      /\
#     /  \
#   T4    T2
#
    def dynamics(self):
        px, py, pz = ca.SX.sym('px'), ca.SX.sym('py'), ca.SX.sym('pz')
        vx, vy, vz = ca.SX.sym('vx'), ca.SX.sym('vy'), ca.SX.sym('vz')
        qw, qx, qy, qz = ca.SX.sym('qw'), ca.SX.sym('qx'), ca.SX.sym('qy'), ca.SX.sym('qz')
        wx, wy, wz = ca.SX.sym('wx'), ca.SX.sym('wy'), ca.SX.sym('wz')
        
        T1, T2, T3, T4 = ca.SX.sym('T1'), ca.SX.sym('T2'), ca.SX.sym('T3'), ca.SX.sym('T4')

        taux = self._arm_l/np.sqrt(2)*(T1+T4-T2-T3)
        tauy = self._arm_l/np.sqrt(2)*(T1+T3-T2-T4)
        tauz = self._c_tau*(T3+T4-T1-T2)
        thrust = (T1+T2+T3+T4)
            
        tau = ca.veccat(taux, tauy, tauz)
        w = ca.veccat(wx, wy, wz)
        w_dot = self._J_inv@( tau - ca.cross(w,self._J@w) )

        fdrag =rotate_quat(ca.veccat(qw,qx,qy,qz) ,self._D@rotate_quat(ca.veccat(qw,-qx,-qy,-qz), ca.veccat(vx,vy,vz)))

        X_dot = ca.vertcat(
            vx,
            vy,
            vz,
            2 * (qw * qy + qx * qz) * (-thrust/self._m) - fdrag[0],
            2 * (qy * qz - qw * qx) * (-thrust/self._m) - fdrag[1],
            (qw * qw - qx * qx - qy * qy + qz * qz) * (-thrust/self._m) + self._G - fdrag[2],
            0.5 * (-wx * qx - wy * qy - wz * qz),
            0.5 * (wx * qw + wz * qy - wy * qz),
            0.5 * (wy * qw - wz * qx + wx * qz),
            0.5 * (wz * qw + wy * qx - wx * qy),
            w_dot[0],
            w_dot[1],
            w_dot[2]
        )

        X = ca.vertcat(px, py, pz,
                       vx, vy, vz,
                       qw, qx, qy, qz,
                       wx, wy, wz)
        U = ca.vertcat(T1, T2, T3, T4)

        fx = ca.Function('f', [X, U], [X_dot], ['X', 'U'], ['X_dot'])
        return fx
    
    def ddynamics(self, dt):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        
        X1 = RK4(f, X0, U, dt, 1)
        # X1 = EulerIntegral(f, X0, U, dt, 1)
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn", [X0, U], [X1], ["X0", "U"], ["X1"])
    
    def ddynamics_dt(self):
        f = self.dynamics()
        X0 = ca.SX.sym("X", f.size1_in(0))
        U = ca.SX.sym("U", f.size1_in(1))
        dt = ca.SX.sym('dt')
        
        # X1 = RK4(f, X0, U, dt, 1)
        X1 = EulerIntegral(f, X0, U, dt, 1)
        
        q_l = ca.sqrt(X1[6:10].T@X1[6:10])
        X1[6:10] = X1[6:10]/q_l
        
        return ca.Function("ddyn_t", [X0, U, dt], [X1], ["X0", "U", "dt"], ["X1"])

class QuadrotorSim(object):
    def __init__(self, quad:QuadrotorModel):
        self._quad = quad
        
        self._dyn_d = quad.ddynamics(0.001) # Continuous State Equation
        # self._dyn_d = RK4(f, 0.001, 4) # Discrete State Equation
        
        # X:=[px,py,pz, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz]
        self._X = np.zeros(13)
        self._X[6] = 1
        self._T = np.zeros(4)
        
        self._pid_wx = PID(15, 0.2, 0, 7, 7)
        self._pid_wy = PID(15, 0.2, 0, 7, 7)
        self._pid_wz = PID(20, 0.3, 0, 7, 7)
        
        self._T_min = quad._T_min
        self._T_max = quad._T_max
    
    # [thrust, wx, wy, wz]
    def low_ctrl(self, U):
        wx = self._X[10]
        wy = self._X[11]
        wz = self._X[12]
        Tx = self._pid_wx.update(U[1]-wx)
        Ty = self._pid_wy.update(U[2]-wy)
        Tz = self._pid_wz.update(U[3]-wz)
        
        T1 = U[0] + Tx + Ty - Tz
        T2 = U[0] - Tx - Ty - Tz
        T3 = U[0] - Tx + Ty + Tz
        T4 = U[0] + Tx - Ty + Tz
        
        self._T[0] = constrain(T1, self._T_min, self._T_max)
        self._T[1] = constrain(T2, self._T_min, self._T_max)
        self._T[2] = constrain(T3, self._T_min, self._T_max)
        self._T[3] = constrain(T4, self._T_min, self._T_max)
    
    # [thrust, wx, wy, wz]
    def step10ms(self, U):
        for i in range(10):
            self.low_ctrl(U)
            self.step1ms()
    
#
#   T1    T3
#     \  /
#      \/
#      /\
#     /  \
#   T4    T2
#
    def step1ms(self):
        X_ = self._dyn_d(self._X, self._T)
        self._X = X_.full().flatten()
        return self._X

if __name__ == "__main__":
    quad = QuadrotorModel('quad/quad_px4.yaml')
    q_sim = QuadrotorSim(quad)
    
    U = np.array([2,1,0,0])
    for i in range(100):
        t1 = time.time()
        q_sim.step10ms(U)
        t2 = time.time()
        print(q_sim._X[6:10])
        print(t2-t1)
    
