#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 15:10:31 2022

@author: eadali
"""

from numpy import zeros, isscalar, array
import random


def asarray(x):
    """Convert the input to an array.

    Parameters
    ----------
    x : array_like
        Input data, in any form that can be converted to an array.

    Returns
    -------
    out : ndarray
        Array interpretation of x
    """
    if isscalar(x):
        x = [x]
    return array(x)


def RK4(fun, t_span, y0, n):
    """Explicit Runge-Kutta method of order 4.

    Parameters
    ----------
    fun : callable
        Right-hand side of the system. The calling signature is fun(t, y).
    t_span : array_like
        Interval of integration (t0, tf).
    y0 : array_like
        Initial state.
    n : int
        Number of integration steps.

    Returns
    -------
    t : float
        Integration end time.
    y : ndarray
        The integrated value at t
    """
    # Integration initial and final time
    t0, tf = t_span
    t, y = t0, asarray(y0)
    # Calculate step-size
    h = (tf - t0) / n
    for i in range(n):
        # Calculate slopes
        k1 = asarray(fun(t,         y))
        k2 = asarray(fun(t+(h/2.0), y + h * (k1/2.0)))
        k3 = asarray(fun(t+(h/2.0), y + h * (k2/2.0)))
        k4 = asarray(fun(t+h,       y + h * k3))
        # Update time and states
        t = t + h
        y = y + (1.0/6.0) * h * (k1 + 2*k2 + 2*k3 + k4)
    return t, y


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
        noise = random.gauss(0, self.noise_std)
        measured_position = position + noise
        return self.measurement_time, measured_position
