#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 16:08:06 2022

@author: eadali
"""

from numpy import isscalar, array

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


class ODE:
    """
    A generic interface class to numeric integrators.

    Solve an equation system :math:`y'(t) = f(y)`.

    *Note*: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

    Parameters
    ----------
    f : callable ``f(y, *f_args)``
        Right-hand side of the differential equation. t is a scalar,
        ``y.shape == (n,)``.
        ``f_args`` is set by calling ``set_f_params(*args)``.
        `f` should return a scalar, array or list (not a tuple).

    Attributes
    ----------
    t : float
        Current time.
    y : ndarray
        Current variable values.

    """

    def __init__(self, f):
        self.f = f

    def set_initial_value(self, T0, Y0):
        """Set initial conditions y(T0) = Y0."""
        self.t = T0
        self.y = asarray(Y0)

    def set_f_params(self, *args):
        """Set extra parameters for user-supplied function f."""
        self.f_params = args

    def integrate(self, t):
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
        # Calculate step-size
        h = t - self.t
        # Calculate slopes
        k1 = asarray(self.f(self.y,          *self.f_params))
        k2 = asarray(self.f(self.y + h*k1/2, *self.f_params))
        k3 = asarray(self.f(self.y + h*k2/2, *self.f_params))
        k4 = asarray(self.f(self.y + h*k3,   *self.f_params))
        # Update state and time
        self.y = self.y + (1.0/6.0) * h * (k1 + 2*k2 + 2*k3 + k4)
        self.t = self.t + h
        return self.y


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
        self.solver = ODE(self.__state_equation)

    def set_initial_value(self, T0, X0):
        """Set initial conditions x(T0) = X0."""
        self.solver.set_initial_value(T0, asarray(X0))

    def __state_equation(self, x, u):
        """State equation of state-space form."""
        return self.A.dot(x) + self.B.dot(u)

    def __output_equation(self, x, u):
        """Output equation of state-space form."""
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
        u = asarray(u)
        self.solver.set_f_params(u)
        x = self.solver.integrate(t)
        return self.__output_equation(x, u)
