#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 16:08:06 2022

@author: eadali
"""

from numpy import isscalar, array, zeros

def clamp(x, lower, upper):
    return min(upper, max(x, lower))


def asarray(x):
    """Convert the input to an array.

    Parameters
    ----------
    x : array_like
        TInput data, in any form that can be converted to an array.

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
    """Find y=y(t), set y as an initial condition, and return y.

    Parameters
    ----------
    t : float
        The endpoint of the integration step.

    Returns
    -------
    y : ndarray
        The integrated value at t
    """
    t0, tf = t_span
    t = t0
    y = asarray(y0)

    # Calculate step-size
    h = (tf - t0) / n
    for i in range(n):
        # Calculate slopes
        k1 = asarray(fun(t,         y))
        k2 = asarray(fun(t+(h/2.0), y + h * (k1/2.0)))
        k3 = asarray(fun(t+(h/2.0), y + h * (k2/2.0)))
        k4 = asarray(fun(t+h,       y + h * k3))
        # Update state and time
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
        self.A, self.B = asarray(A), asarray(B)
        self.C, self.D = asarray(C), asarray(D)
        self.u = 0.0
        self.t = 0.0
        self.x = [0.0, 0.0, 0.0, 0.0]
        # self.solver = ODE(self.__state_equation)

    def set_initial_value(self, T0, X0):
        """Set initial conditions x(T0) = X0."""
        self.t = T0
        self.x = X0
        # self.solver.set_initial_value(T0, asarray(X0))

    def _state_equation(self, t, x):
        """State equation of state-space form."""
        u = self.u
        return self.A.dot(x) + self.B.dot(u)

    def _output_equation(self, t, x):
        """Output equation of state-space form."""
        u = self.u
        return self.C.dot(x) + self.D.dot(u)

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
        self.u = asarray(u)
        self.t, self.x = RK4(self._state_equation,
                             (self.t, t),
                             self.x,
                             10)
        return self._output_equation(self.t, self.x)
