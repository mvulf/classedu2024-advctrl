from regelum.system import System
from regelum.utils import rg
from regelum import callback
from regelum.system import InvertedPendulum, CartPole


class InvertedPendulum(InvertedPendulum):
    _parameters = {"m": 1, "g": 9.8, "l": 1.0}

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m, g, l = (
            self._parameters["m"],
            self._parameters["g"],
            self._parameters["l"],
        )

        Dstate[0] = state[1]
        Dstate[1] = g / l * rg.sin(state[0]) + inputs[0] / (m * l**2)

        return Dstate


class InvertedPendulumWithFriction(InvertedPendulum):
    _parameters = {"m": 1, "g": 9.8, "l": 1.0, "c": 0.08}

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m, g, l, friction_coef = (
            self._parameters["m"],
            self._parameters["g"],
            self._parameters["l"],
            self._parameters["c"],
        )

        Dstate[0] = state[1]
        Dstate[1] = (
            g / l * rg.sin(state[0])
            + inputs[0] / (m * l**2)
            - friction_coef * state[1] ** 2 * rg.sign(state[1])
        )

        return Dstate


class MyCartPole(CartPole):
    _dim_observation = 4
    _dim_state = 4
    _parameters = {"m_c": 0.1, "m_p": 2.0, "g": 9.81, "l": 0.5}
    _action_bounds = [[-300.0, 300.0]]

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        m_c, m_p, g, l = (
            self.parameters["m_c"],
            self.parameters["m_p"],
            self.parameters["g"],
            self.parameters["l"],
        )

        theta = state[0]
        theta_dot = state[2]
        x_dot = state[3]
        Force = inputs[0]
        sin_theta = rg.sin(theta)
        cos_theta = rg.cos(theta)

        #########################
        ## YOUR CODE GOES HERE ##

        ...

        Dstate[0] = ...
        Dstate[1] = ...
        Dstate[2] = ...
        Dstate[3] = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state
