import casadi as ca
from robotmpcs.models.mpcBase import MpcBase
from robotmpcs.models.utils.utils import diagSX
from robotmpcs.utils.utils import shift_angle_casadi
class GoalPositionReaching(MpcBase):

    def __init__(self,ineq_modules, **kwargs):
        super().__init__(**kwargs)

    def set_parameters(self, ParamMap, npar):
        self._paramMap = ParamMap
        self._npar = npar

        self.addEntry2ParamMap("goal_position", self._m)
        self.addEntry2ParamMap("goal_angle", 1)
        self.addEntry2ParamMap("wgoal_position", self._m)
        self.addEntry2ParamMap("wgoal_angle", 1)

        self._onlyN = True
        return self._paramMap, self._npar


    def eval_objective(self, z, p):
        variables = self.extractVariables(z)
        q = variables[0]
        pos_ee = self._fk.fk(
            q,
            self._robot_config.root_link,
            self._robot_config.end_link,
            positionOnly=False
        )
        goal_position = p[self._paramMap["goal_position"]]
        w_position = p[self._paramMap["wgoal_position"]]
        W = diagSX(w_position, 2)

        err = pos_ee[0:2,3] - goal_position[:2]
        dist = ca.fmax(ca.sqrt(err[0]**2 + err[1]**2),0.01)
        err_normalized = err/dist

        Jgoal =  ca.dot(err_normalized, ca.mtimes(W, err)) 
        return Jgoal