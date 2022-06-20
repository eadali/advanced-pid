#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 16:08:06 2022

@author: eadali
"""

from numpy import isscalar, array, zeros, clip


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


def pid2ss(Kp, Ki, Kd, Tf):
    """Convert PID gains to state-space form.

    Parameters
    ----------
    Kp, Ki, Kd : float
        Proportional, Integral and Derivative gain.
    Tf : float
        Time constant of the first-order derivative filter.

    Returns
    -------
    A, B, C, D : ndarray
        State space representation of the system, in observable canonical form.
    """
    Kn = 1.0 / Tf
    A = array([[0.0, 0.0, 0.0],
               [0.0, 0.0, 0.0],
               [0.0, 0.0, -Kn]])
    B = array([[0.0, ],
               [Ki, ],
               [-Kd*Kn*Kn, ]])
    C = array([[0.0, 1.0, 1.0],
               [1.0, 0.0, 0.0],
               [0.0, 1.0, 0.0],
               [0.0, 0.0, 1.0]])
    D = array([[Kp+Kd*Kn, ],
               [0.0, ],
               [0.0, ],
               [0.0, ]])
    return A, B, C, D


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


class StateSpace:
    r"""
    Linear Time Invariant system in state-space form.

    Represents the system as the continuous-time, first order differential
    equation :math:`\dot{x} = A x + B u`.

    Parameters
    ----------
    A, B, C, D: ndarray
        State space representation of the system

    """

    def __init__(self, A, B, C, D):
        # Create time
        self.t = None
        # Create state-space form matrices
        self.A, self.B = asarray(A), asarray(B)
        self.C, self.D = asarray(C), asarray(D)
        # Create states
        self.x = zeros(self.A.shape[0])
        # Create input
        self.u = None

    def set_initial_value(self, t0, x0):
        """Set initial conditions x(t0) = x0."""
        self.t, self.x = t0, x0

    def _state_equation(self, _, x):
        """State equation of state-space form."""
        return self.A.dot(x) + self.B.dot(self.u)

    def _output_equation(self, _, x):
        """Output equation of state-space form."""
        return self.C.dot(x) + self.D.dot(self.u)

    def integrate(self, t, u):
        """Find x=x(t), set x as an initial condition, and return x.

        Parameters
        ----------
        t : float
            The endpoint of the integration step.

        Returns
        -------
        x : ndarray
            The integrated value at t
        """
        # If first call, set current time
        if self.t is None:
            self.t = t
        # Set input and solve ordinary differential equation
        self.u = asarray(u)
        self.t, self.x = RK4(self._state_equation,
                             (self.t, t),
                             self.x,
                             10)
        return self._output_equation(self.t, self.x)
