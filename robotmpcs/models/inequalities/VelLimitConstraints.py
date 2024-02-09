import casadi as ca
from robotmpcs.models.mpcBase import MpcBase
class VelLimitConstraints(MpcBase):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._n_ineq = (self._nx-self._n)*2

    def set_parameters(self, ParamMap,npar):
        self._paramMap = ParamMap
        self._npar = npar

        self.addEntry2ParamMap("lower_limits_vel", self._nx-self._n)
        self.addEntry2ParamMap("upper_limits_vel", self._nx-self._n)
        return self._paramMap, self._npar


    def eval_constraint(self, z, p):
        # Parameters in state boundaries?
        q, qdot, _ = self.extractVariables(z)
        vel = qdot
        lower_limits = p[self._paramMap["lower_limits_vel"]]
        upper_limits = p[self._paramMap["upper_limits_vel"]]
        ineqs = []
        for j in range(self._nx-self._n):
            dist_lower = vel[j] - lower_limits[j]
            dist_upper = upper_limits[j] - vel[j]
            ineqs.append(dist_lower)
            ineqs.append(dist_upper)
        print("!!!!!!!!!!!!!!!!!vel_constraints")
        return ineqs