import casadi as ca
import numpy as np

# import tf
import os, sys
BASEPATH = os.path.abspath(__file__).split('function_model/', 1)[0]+'function_model/'
sys.path += [BASEPATH]
from os import system
from quadrotor_control import QuadrotorSimpleModel

# calculate yaw error according to q
def yaw_q_error(yaw_d, qw, qz):
    c_yaw = ca.cos(yaw_d / 2)
    s_yaw = ca.sin(yaw_d / 2)
    sqrt_q = ca.sqrt(qw * qw + qz * qz)
    delta_yaw_c = qw - c_yaw * sqrt_q
    delta_yaw_s = qz - s_yaw * sqrt_q
    return delta_yaw_c * delta_yaw_c + delta_yaw_s * delta_yaw_s

def p_cost(v, Th):
    # 1
    c = v.T@v
    return c

def u_cost(U, diag_Q):
    ee = U - ca.DM([9.81, 0, 0, 0])
    c = ee.T@diag_Q@ee
    return c

# pos yaw
class TrackerMPC():
    def __init__(self, quad:QuadrotorSimpleModel):
        # load uav model / mpc horizon / state equation  
        self._quad = quad
        self._Herizon = 5
        self._ddynamics = []
        for n in range(self._Herizon):
            self._ddynamics += [self._quad.ddynamics(0.1)]
        
        # calculate param define
        self._X_dim = self._ddynamics[0].size1_in(0)
        self._U_dim = self._ddynamics[0].size1_in(1)
        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub
        
        self._Xs = ca.SX.sym('Xs', self._X_dim, self._Herizon)
        self._Us = ca.SX.sym('Us', self._U_dim, self._Herizon)
        
        self._X_init = ca.SX.sym("X_init", self._X_dim)
        self._Trj_p = ca.SX.sym("Trj_p", 3, self._Herizon)
        self._Trj_yaw = ca.SX.sym("Trj_yaw", self._Herizon)
        
        self._opt_option = {
            'verbose': False,
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 1e-2,
            'ipopt.max_iter': 25,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
        }
        
        # nlp param init / 0
        self._nlp_x_x = []
        self._nlp_lbx_x = []
        self._nlp_ubx_x = []
        
        self._nlp_x_u = []
        self._nlp_lbx_u = []
        self._nlp_ubx_u = []

        self._nlp_g_dyn = []
        self._nlp_lbg_dyn = []
        self._nlp_ubg_dyn = []

        self._nlp_p_xinit = []
        self._nlp_p_Trj_p = []
        self._nlp_p_Trj_yaw = []

        self._nlp_obj_dyn = 0
        self._nlp_obj_trjp = 0
        self._nlp_obj_trjyaw = 0
        self._nlp_obj_u = 0
        
        self._nlp_x_x += [ self._Xs[:, 0] ]
        self._nlp_lbx_x += self._X_lb
        self._nlp_ubx_x += self._X_ub
        self._nlp_x_u += [ self._Us[:, 0] ]
        self._nlp_lbx_u += self._U_lb
        self._nlp_ubx_u += self._U_ub
        
        dd_dyn = self._Xs[:,0]-self._ddynamics[0]( self._X_init, self._Us[:,0] )
        self._nlp_g_dyn += [ dd_dyn ]
        self._nlp_obj_dyn += dd_dyn.T@dd_dyn
        self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
        self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
        
        self._nlp_obj_trjp += p_cost(self._Xs[:3,0]-self._Trj_p[:,0], 0.5)
        self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[0], self._Xs[6, 0], self._Xs[9, 0])
        self._nlp_obj_u += u_cost(self._Us[:, 0], ca.diag([1, 1, 1, 0]))
        
        # nlp param add (x/g/p) / 1 to horizon
        for i in range(1,self._Herizon):
            self._nlp_x_x += [ self._Xs[:, i] ]
            self._nlp_lbx_x += self._X_lb
            self._nlp_ubx_x += self._X_ub
            self._nlp_x_u += [ self._Us[:, i] ]
            self._nlp_lbx_u += self._U_lb
            self._nlp_ubx_u += self._U_ub
            
            dd_dyn = self._Xs[:,i]-self._ddynamics[i]( self._Xs[:,i-1], self._Us[:,i] )
            self._nlp_g_dyn += [ dd_dyn ]
            self._nlp_obj_dyn += dd_dyn.T@dd_dyn
            self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
            self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
            
            self._nlp_obj_trjp += p_cost(self._Xs[:3,i]-self._Trj_p[:,i], 0.5)
            self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[i], self._Xs[6, i], self._Xs[9, 0])
            self._nlp_obj_u += u_cost(self._Us[:, i], ca.diag([1, 1, 1, 0]))

        self._nlp_p_xinit += [self._X_init]
        for i in range(self._Herizon):
            self._nlp_p_Trj_p += [self._Trj_p[:,i]]
            self._nlp_p_Trj_yaw += [self._Trj_yaw[i]]
    
    # reset qw
    def reset_xut(self):
        self._xu0 = np.zeros((self._X_dim+self._U_dim)*self._Herizon)
        for i in range(self._Herizon):
            self._xu0[i*self._X_dim+6] = 1
    
    # load nlp solver file
    def load_so(self, so_path):
        self._opt_solver = ca.nlpsol("opt", "ipopt", so_path, self._opt_option)
        self.reset_xut()
    
    # define nlp solver
    def define_opt(self):
        nlp_dect = {
            'f': 1*self._nlp_obj_trjp + 0.01*self._nlp_obj_u + 0.1*self._nlp_obj_trjyaw,
            'x': ca.vertcat(*(self._nlp_x_x + self._nlp_x_u)),
            'p': ca.vertcat(*(self._nlp_p_xinit + self._nlp_p_Trj_p + self._nlp_p_Trj_yaw)),
            'g': ca.vertcat(*(self._nlp_g_dyn)),
        }
        self._opt_solver = ca.nlpsol('opt', 'ipopt', nlp_dect, self._opt_option)
        
        self.reset_xut()
        
    # solve nlp problem
    def solve(self, xinit, Trjp, Trjyaw):
        p = np.zeros(self._X_dim + 3*self._Herizon + 1*self._Herizon)
        #load param
        p[:self._X_dim] = xinit
        p[self._X_dim:self._X_dim+3*self._Herizon] = Trjp
        p[self._X_dim+3*self._Herizon:self._X_dim+4*self._Herizon] = Trjyaw
        
        res = self._opt_solver(
            x0=self._xu0,
            lbx=(self._nlp_lbx_x+self._nlp_lbx_u),
            ubx=(self._nlp_ubx_x+self._nlp_ubx_u),
            lbg=(self._nlp_lbg_dyn),
            ubg=(self._nlp_ubg_dyn),
            p=p
        )
        
        self._xu0 = res['x'].full().flatten()
        
        return res

if __name__ == "__main__":
    quad = QuadrotorSimpleModel(BASEPATH+"quad/quad_sim.yaml")
    tracker = TrackerMPC(quad)
    tracker.define_opt()
    print('finish define')
    tracker._opt_solver.generate_dependencies("tracker_mpc.c")
    print('begin gcc')
    system('gcc -fPIC -shared -O3 ' + "tracker_mpc.c" + ' -o ' + './track_mpc.so')

    
