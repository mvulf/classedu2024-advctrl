from regelum.utils import rg
from regelum.system import CartPole


class CartPoleWithFriction(CartPole):
    _dim_observation = 4
    _dim_state = 4
    _parameters = {"m_c": 0.1, "m_p": 2.0, "g": 9.81, "l": 0.5, "friction_coeff": 100.0}
    _action_bounds = [[-600.0, 600.0]]

    def _compute_state_dynamics(self, time, state, inputs):
        Dstate = rg.zeros(
            self.dim_state,
            prototype=(state, inputs),
        )

        mass_cart, mass_pole, grav_const, length_pole, friction_coeff = (
            self.parameters["m_c"],
            self.parameters["m_p"],
            self.parameters["g"],
            self.parameters["l"],
            self._parameters["friction_coeff"],
        )

        angle = state[0]
        angle_vel = state[2]
        vel = state[3]
        force = inputs[0]
        sin_angle = rg.sin(angle)
        cos_angle = rg.cos(angle)

        #########################
        ## YOUR CODE GOES HERE ##

        Dstate[0] = ...
        Dstate[1] = ...
        Dstate[2] = ...
        Dstate[3] = ...

        ## YOUR CODE ENDS HERE ##
        #########################

        return Dstate

    def _get_observation(self, time, state, inputs):
        return state