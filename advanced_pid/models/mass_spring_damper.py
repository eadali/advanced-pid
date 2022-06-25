#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 15:10:31 2022

@author: eadali
"""

from numpy import zeros, random
from advanced_pid.integrate import RK4


class MassSpringDamper:
    r"""
    A mass-spring-damper model.

    Solve a differential equation :math:`m\ddot{x} = -kx - b\dot{x} - u`

    *Note*: https://en.wikipedia.org/wiki/Mass-spring-damper_model

    Parameters
    ----------
    mass : float
        Mass.
    spring_const : float
        Spring constant.
    damping_const : float
        Damping constant.
    noise_std : float
        Standart deviation of measurement noise.
    """

    def __init__(self, mass, spring_const, damping_const, noise_std=0.02):
        # Set initial time and time step
        self.measurement_time, self.time_step = 0.0, 0.01
        # Set model parameters
        self.mass = mass
        self.spring_const = spring_const
        self.damping_const = damping_const
        self.noise_std = noise_std
        # Set initial values
        self.states = zeros(2)
        # Set input
        self.external_force = 0.0

    def set_initial_value(self, initial_position, initial_velocity):
        """Set mass-spring-damper model states.

        Parameters
        ----------
        initial_position : float
            Initial position of mass.
        initial_velocity : float
            Initial velocity of mass.
        """
        self.states[0] = initial_position
        self.states[1] = initial_velocity

    def _state_equation(self, _, states):
        """State equation of state-space form."""
        position, velocity = states
        spring_acceleration = (self.spring_const / self.mass) * position
        damper_acceleration = (self.damping_const / self.mass) * velocity
        external_acceleration = (1.0 / self.mass) * self.external_force
        acceleration = (- spring_acceleration
                        - damper_acceleration
                        + external_acceleration)
        dxdt = [velocity, acceleration]
        return dxdt

    def set_input(self, external_force):
        """Set external force.

        Parameters
        ----------
        external_force : float
            External force is applied to mass.
        """
        self.external_force = external_force

    def get_measurement(self):
        """Get position measurement of mass.

        Returns
        -------
        measurement_time : float
            Measurement timestamp.
        measured_position : float
            Measured position of mass.
        """
        # Update measurement time
        self.measurement_time = self.measurement_time + self.time_step
        # Solve ordinary differential equation for current states
        _, self.states = RK4(self._state_equation,
                             (0.0, self.time_step),
                             self.states,
                             10)
        position = self.states[0]
        noise = random.normal(scale=self.noise_std)
        measured_position = position + noise
        return self.measurement_time, measured_position
