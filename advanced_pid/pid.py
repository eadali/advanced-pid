#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 20:06:38 2022

@author: eadali
"""

from warnings import warn
from numpy import exp, inf, clip


class PID:
    """
    An advanced PID controller with first-order filter on derivative term.

    *Note*: https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf

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
         Derivative term will not be used if the value is not a positive number.

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
        u : float
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
            Derivative term will not be used if the value is not
             a positive number.
        """
        self.Kp, self.Ki, self.Kd, self.Tf = Kp, Ki, Kd, Tf

    def get_gains(self):
        """Get PID controller gains.

        Returns
        -------
        Kp : float
            Proportional gain.
        Ki: float
            Integral gain.
        Kd : float
            Derivative gain.
        Tf : float
            Time constant of the first-order derivative filter.
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
        # If limit is None, set limit to -inf/+inf
        if lower is None:
            self.lower = -inf
        if upper is None:
            self.upper = +inf

    def get_output_limits(self):
        """Get PID controller output limits for anti-windup.

        Returns
        -------
        lower : float or None
            Lower limit for saturation.
        upper : flaot or None
            Upper limit for saturation.
        """
        return self.lower, self.upper

    def set_initial_value(self, t0, e0, i0):
        """Set PID controller states.

        Parameters
        ----------
        t0 : float or None
            Initial current time. None will reset current time.
        e0 : float or None
            Initial current error. None will reset current error.
        i0 : float or None
            Inital current integral. None will reset current integral.
        """
        self.t, self.e, self.i = t0, e0, i0

    def get_initial_value(self):
        """Get PID controller states.

        Returns
        -------
        t : float or None
            Initial current time.
        e : float or None
            Initial current error.
        i : float or None
            Inital current integral.
        """
        return self.t, self.e, self.i

    def _set_none_value(self, t, e):
        t0, e0, i0 = self.get_initial_value()
        if t0 is None:
            t0 = t
        if e0 is None:
            e0 = e
        if i0 is None:
            i0 = 0.0
        self.set_initial_value(t0, e0, i0)

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
        # Check if current time is smaller than previous time
        if t < t0:
            t0 = t
            msg = """Current timestamp is smaller then previous timestamp.
                     Time step will be accepted as zero."""
            warn(msg, RuntimeWarning)

        # Calculate time step
        dt = t - t0
        # Calculate proportional term
        p = self.Kp * e
        # Calculate integral term
        i = i0 + dt * self.Ki * e
        i = clip(i, self.lower, self.upper)
        # Calcuate derivative term
        d = 0.0
        print(self.Tf)
        if self.Kd > 0.0 and self.Tf > 0.0:
            Kn = 1.0 / self.Tf
            x = -Kn * self.Kd * e0
            # x = e0
            x = (exp(-Kn*dt) * x
                 + self.Tf * (1.0-exp(-Kn*dt)) * -Kn**2 *self.Kd * e)
            d = 1.0 * x + Kn * self.Kd * e
            d = x
            # e = x
            e = -self.Tf * (1.0/self.Kd) * x

        self.set_initial_value(t, e, i)
        return clip(p+i+d, self.lower, self.upper)
