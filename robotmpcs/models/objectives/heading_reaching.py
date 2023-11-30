import casadi as ca
from robotmpcs.models.mpcBase import MpcBase
from robotmpcs.models.utils.utils import diagSX
from robotmpcs.utils.utils import shift_angle_casadi

class HeadingReaching(MpcBase):

    def __init__(self,ineq_modules, **kwargs):
        super().__init__(**kwargs)

    def set_parameters(self, ParamMap, npar):
        self._paramMap = ParamMap
        self._npar = npar
        
        self.addEntry2ParamMap("wheading", 1)

        return self._paramMap, self._npar


    def eval_objective(self, z, p):
        variables = self.extractVariables(z)
        q = variables[0]
        qdot = variables[1]
        speed = ca.sqrt(qdot[0]**2 + qdot[1]**2)
        
        heading_angle = ca.if_else(qdot[0]!=0, ca.atan2(qdot[1], qdot[0]), ca.SX(0.0))
        
        theta = q[2]
        
        w_angle = p[self._paramMap["wheading"]]
        W_angle = diagSX(w_angle, 1)
        

        err_angle = shift_angle_casadi(heading_angle - theta)
        
        Jheading=  ca.if_else(speed>0.01, ca.dot(err_angle, ca.mtimes(W_angle, err_angle)), ca.SX(0.0))
        return Jheading