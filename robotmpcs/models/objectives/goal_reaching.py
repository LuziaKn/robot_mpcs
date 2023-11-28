import casadi as ca
from robotmpcs.models.mpcBase import MpcBase
from robotmpcs.models.utils.utils import diagSX
from robotmpcs.utils.utils import shift_angle_casadi
class GoalReaching(MpcBase):

    def __init__(self,ineq_modules, **kwargs):
        super().__init__(**kwargs)

    def set_parameters(self, ParamMap, npar):
        self._paramMap = ParamMap
        self._npar = npar

        self.addEntry2ParamMap("goal_position", self._m)
        self.addEntry2ParamMap("goal_angle", 1)
        self.addEntry2ParamMap("wgoal", self._m)

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
        goal_angle = p[self._paramMap["goal_angle"]]
        w = p[self._paramMap["wgoal"]]
        W = diagSX(w, 2)
        W_angle = diagSX(0.5 * w, 1)
        err = pos_ee[0:2,3] - goal_position[:2]
        err_angle = shift_angle_casadi(ca.acos(pos_ee[0,0])) - shift_angle_casadi(goal_angle)
        Jgoal = ca.dot(err, ca.mtimes(W, err)) #+ ca.dot(err_angle, ca.mtimes(W_angle, err_angle))

        return Jgoal