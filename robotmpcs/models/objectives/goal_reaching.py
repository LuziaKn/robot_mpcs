import casadi as ca
from robotmpcs.models.mpcBase import MpcBase
from robotmpcs.utils.utils import diagSX, orientation_matrix_to_euler


class GoalReaching(MpcBase):

    def __init__(self,ineq_modules, **kwargs):
        super().__init__(**kwargs)

    def set_parameters(self, ParamMap, npar):
        self._paramMap = ParamMap
        self._npar = npar

        self.addEntry2ParamMap("goal_position", self._m)
        self.addEntry2ParamMap("goal_orientation", self._m)
        self.addEntry2ParamMap("wgoal_position", self._m)
        self.addEntry2ParamMap("wgoal_orientation", self._m)
        return self._paramMap, self._npar


    def eval_objective(self, z, p):
        variables = self.extractVariables(z)
        q = variables[0]
        trans_ee = self._fk.fk(
            q,
            self._robot_config.root_link,
            self._robot_config.end_link,
            positionOnly=False
        )

        phi, theta, psi = orientation_matrix_to_euler(trans_ee[0:3,0:3])

        # compute error in position
        pos_ee = trans_ee[0:3,3]
        goal_position = p[self._paramMap["goal_position"]]
        w = p[self._paramMap["wgoal_position"]]
        W_position = diagSX(w, 2)
        err_position = pos_ee[:2] - goal_position[:2]

        # compute error in orientation
        goal_orientation = p[self._paramMap["goal_orientation"]]
        w = p[self._paramMap["wgoal_orientation"]]
        W_orientation = diagSX(w, 1)
        err_orientation = q[2] - goal_orientation[2]

        Jgoal = ca.dot(err_position, ca.mtimes(W_position, err_position)) #+ ca.dot(err_orientation, ca.mtimes(W_orientation, err_orientation))

        return Jgoal