#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 20:06:38 2022

@author: eadali
"""

from warnings import warn
from math import exp


class PID:
    """An advanced PID controller with first-order filter on derivative term.

    Parameters
    ----------
    Kp : float
        Proportional gain.
    Ki: float
        Integral gain.
    Kd : float
        Derivative gain.
    Tf : float
        Time constant of the first-order derivative filter.

    """

    def __init__(self, Kp, Ki, Kd, Tf):
        self.set_gains(Kp, Ki, Kd, Tf)
        self.set_output_limits(None, None)
        self.set_initial_value(None, None, None)

    def __call__(self, t, e):
        """Call integrate method.

        Parameters
        ----------
        t : float
            Current time.
        e : float
            Error signal.

        Returns
        -------
        float
            Control signal.
        """
        return self.integrate(t, e)

    def set_gains(self, Kp, Ki, Kd, Tf):
        """Set PID controller gains.

        Parameters
        ----------
        Kp : float
            Proportional gain.
        Ki: float
            Integral gain.
        Kd : float
            Derivative gain.
        Tf : float
            Time constant of the first-order derivative filter.

        """
        self.Kp, self.Ki, self.Kd, self.Tf = Kp, Ki, Kd, Tf

    def get_gains(self):
        """Get PID controller gains.

        Returns
        -------
        tuple
            Gains of PID controller (Kp, Ki, Kd, Tf).

        """
        return self.Kp, self.Ki, self.Kd, self.Tf

    def set_output_limits(self, lower, upper):
        """Set PID controller output limits for anti-windup.

        Parameters
        ----------
        lower : float or None
            Lower limit for anti-windup,
        upper : flaot or None
            Upper limit for anti-windup.

        """
        self.lower, self.upper = lower, upper
        if lower is None:
            self.lower = -float('inf')
        if upper is None:
            self.upper = +float('inf')

    def get_output_limits(self):
        """Get PID controller output limits for anti-windup.

        Returns
        -------
        tuple
            Output limits (lower, upper).

        """
        return self.lower, self.upper

    def set_initial_value(self, t0, e0, i0):
        """Set PID controller states.

        Parameters
        ----------
        t0 : float or None
            Initial time. None will reset time.
        e0 : float or None
            Initial error. None will reset error.
        i0 : float or None
            Inital integral. None will reset integral.
        """
        self.t0, self.e0, self.i0 = t0, e0, i0

    def get_initial_value(self):
        """Get PID controller states.

        Returns
        -------
        tuple
            Initial states of PID controller (t0, e0, i0)
        """
        return self.t0, self.e0, self.i0

    def __set_none_value(self, t, e):
        """Set None states for first cycle."""
        t0, e0, i0 = self.get_initial_value()
        if t0 is None:
            t0 = t
        if e0 is None:
            e0 = e
        if i0 is None:
            i0 = 0.0
        self.set_initial_value(t0, e0, i0)

    def __check_monotonic_timestamp(t0, t):
        """Check timestamp is monotonic."""
        if t < t0:
            msg = 'Current timestamp is smaller than initial timestamp.'
            warn(msg, RuntimeWarning)
            return False
        return True

    def integrate(self, t, e):
        """Calculates PID controller output.

        Parameters
        ----------
        t : float
            Current time.
        e : float
            Error signal.

        Returns
        -------
        u : float
            Control signal.
        """
        self._set_none_value(t, e)
        t0, e0, i0 = self.get_initial_value()
        # Check monotonic timestamp
        if not self.__check_monotonic_timestamp(t0, t):
            t0 = t
        # Calculate time step
        dt = t - t0
        # Calculate proportional term
        p = self.Kp * e
        # Calculate integral term
        i = i0 + dt * self.Ki * e
        i = min(max(i, self.lower), self.upper)
        # Calcuate derivative term
        d = 0.0
        if self.Kd > 0.0 and self.Tf > 0.0:
            Kn = 1.0 / self.Tf
            x = -Kn * self.Kd * e0
            x = exp(-Kn*dt) * x - Kn * (1.0 - exp(-Kn*dt)) * self.Kd * e
            d = x + Kn * self.Kd * e
            e = -(self.Tf/self.Kd) * x

        self.set_initial_value(t, e, i)
        return min(max(p+i+d, self.lower), self.upper)
