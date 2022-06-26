#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 20:06:38 2022

@author: eadali
"""

from warnings import warn
from advanced_pid.integrate import pid2ss, StateSpace, clip


class PID:
    def __init__(self, Kp, Ki, Kd, Tf):
        # Crate Current time
        self._t = None
        # Create error and integral states
        self._e, self._i = None, None
        # Create PID gains and output limits
        self._output_limits = [None, None]
        # Set PID gains and output limits
        self.set_gains(Kp, Ki, Kd, Tf)
        self.set_output_limits(self._output_limits)

    def __call__(self, t, e):
        """Call integrate method.

        Parameters
        ----------
        t : float
            Current time.

        e : float
            Error input signal.

        Returns
        -------
        u : float
            Calculated control signal of controller.
        """
        # Call integrate method
        return self.integrate(t, e)

    def set_gains(self, Kp, Ki, Kd, Tf):
        """Set PID controller gains.

        Parameters
        ----------
        Kp, Ki, Kd : float
            Proportional, Integral and Derivative gain.
        Tf : float
            Time constant of the first-order derivative filter.
        """
        if not Tf > 0:
            raise ValueError('Tf value need to be a positive number.')
        # Create state-space system and set gains
        A, B, C, D = pid2ss(Kp, Ki, Kd, Tf)
        self._ss = StateSpace(A, B, C, D)
        self._Kp, self._Ki, self._Kd, self._Tf = Kp, Ki, Kd, Tf

    def get_gains(self):
        """Get PID controller gains.

        Returns
        -------
        Kp, Ki, Kd : float
            Proportional, Integral and Derivative gain.
        """
        return self._Kp, self._Ki, self._Kd, self._Tf

    def set_output_limits(self, output_limits):
        """Set PID controller output limits for anti-windup.

        Parameters
        ----------
        output_limits : array_like (lower_limit, upper_limit)
            Output limits for anti-windup.
        """
        lower, upper = output_limits
        # If lower limit is None, set lower limit -inf
        if lower is None:
            self._output_limits[0] = -float("inf")
        else:
            self._output_limits[0] = lower
        # If upper limit is None, set upper limit +inf
        if upper is None:
            self._output_limits[1] = +float("inf")
        else:
            self._output_limits[1] = upper

    def get_output_limits(self):
        """Get PID controller output limits for anti-windup.

        Returns
        -------
        output_limits : array_like (lower_limit, upper_limit)
            Output limits for anti-windup.
        """
        return tuple(self._output_limits)

    def set_initial_value(self, T0, E0, I0):
        """Set PID controller state-space model states.

        Parameters
        ----------
        T0 : float or None
            Initial current time. None will reset timer
        E0 : float or None
            Expected error input signal. None will reset derivative state
        I0 : float or None
            Integral state of state-space model. None will reset integral state
        """
        self._t, self._e, self._i = T0, E0, I0

    def get_initial_value(self):
        """Set PID controller state-space model states.

        Returns
        -------
        t : float
            Initial current time.
        i : float
            Integral state of state-space model.
        f : float
            Derivative filter state of state-space model.
        """
        return self._t, self._e, self._i

    def integrate(self, t, e):
        """Calculates PID controller states and returns output of controller.

        Args:
            u (float): Input value of controller
            dt (float): Time step used for integral calculations
        """
        # Set time to current time, If it is not set
        if self._t is None:
            self._t = t
        # Set derivative filter state to zero, If It is not set
        if self._e is None:
            self._e = e
        # Set integral state to zero, If It is not set
        if self._i is None:
            self._i = 0.0
        # Check if current time is smaller than previous time
        if t < self._t:
            t = self._t
            warn('Current time is smaller then previous time.', RuntimeWarning)
        f = -(self._Kd/self._Tf) * self._e
        self._ss.set_initial_value(self._t, [0.0, self._i, f])
        u, _, i, f = self._ss.integrate(t, e)

        # Calculates proportional term
        if self._Kd > 0:
            e = -(self._Tf/self._Kd) * f
        

        lower, upper = self._output_limits
        self.set_initial_value(t, e, clip(i, lower, upper))
        return clip(u, lower, upper)
