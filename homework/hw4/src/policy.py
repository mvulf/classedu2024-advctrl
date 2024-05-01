from numpy.core.multiarray import array as array
from regelum.policy import Policy
import numpy as np
from src.system import CartPole
from typing import Union
from regelum.utils import rg
from regelum import CasadiOptimizerConfig


class CartPoleMPC(Policy):
    def __init__(
        self,
        system: CartPole,
        sampling_time: float,
        optimizer_config: CasadiOptimizerConfig,
    ):
        super().__init__(optimizer_config=optimizer_config)
        self.system = system

        #########################
        ## YOUR CODE GOES HERE ##
        
        w_angle = 30.
        w_position = 14.
        w_angle_vel = 3.
        w_vel = 3.

        ## Define hyperparameters
        self.pred_step_size: float = 5*sampling_time
        self.cost_weights = np.array(
            [
                w_angle,
                w_position,
                w_angle_vel,
                w_vel,
            ]
        )
        self.prediction_horizon: int = 15
        ## YOUR CODE ENDS HERE ##
        #########################

        self.instatiate_optimization_problem()

    def instatiate_optimization_problem(self) -> None:
        self.current_state_var = self.create_variable(
            self.system._dim_state, name="current_state", is_constant=True
        )
        self.actions_var = self.create_variable(
            self.prediction_horizon + 1,
            self.system._dim_action,
            name="actions",
            is_constant=False,
        )
        bounds = self.handle_bounds(
            self.system._action_bounds,
            self.system._dim_action,
            self.prediction_horizon + 1,
        )[0]
        self.register_bounds(self.actions_var, bounds)
        self.register_objective(
            self.mpc_objective,
            variables=[self.current_state_var, self.actions_var],
        )

    def mpc_objective(self, current_state, actions) -> float:
        cost_weights = rg.array(self.cost_weights, prototype=current_state)

        first_running_cost = rg.sum(current_state**2 * cost_weights)
        sum_costs = first_running_cost
        state = current_state
        for k in range(self.prediction_horizon + 1):
            #########################
            ## YOUR CODE GOES HERE #
            # time - uses only for the convention satisfaction
            time = k*self.pred_step_size
            # Predict state
            state += (
                self.pred_step_size*self.system._compute_state_dynamics(
                    time=time,
                    state=state,
                    inputs=actions[k, :]
                )
            )
            # Add running cost to sum of costs
            sum_costs += rg.sum(state**2 * cost_weights)

            ## YOUR CODE ENDS HERE ##
            #########################

        return sum_costs

    def get_action(self, observation: np.ndarray) -> np.ndarray:
        actions = self.optimize(current_state=observation[0])
        return np.array([[float(actions["actions"][0, 0])]])
