#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 15:10:31 2022

@author: eadali
"""

from numpy import random
from scipy.integrate import odeint

class MassSpringDamper:
    def __init__(self, m, k, b, dt, std=0):
        """Inits pendulum constants and initial state
        # Arguments
            m: Mass
            k: Spring coeff
            b: Friction coeff
            x_0: Initial state
        """
        self.dt = dt
        self.t = 0.0
        self.x= [0.0, 0.0]
        
        self.m = m
        self.k = k
        self.b = b
        self.std = std

    def set_initial_value(self, t0, x0):
        self.t = t0
        self.x = x0

    def ode(self, x, t, u):
        """Dynamic equations of pendulum
        # Arguments
            x: [position of mass, velocity of mass]
            t: Time steps for ode solving
            u: External force applied to the mass
        # Returns
            Derivative of internal states
        """

        # ODE of mass-spring-damper model
        position, velocity = x
        dxdt = [velocity,
                -(self.b/self.m)*velocity - (self.k/self.m)*position + (1/self.m)*u]

        return dxdt

    def set_input(self, u):
        """Interface function for pendulum model
        # Arguments
            u: External force applied to the mass
        # Returns
            Position of mass
        """

        # Solving ODE with scipy library
        self.x = odeint(self.ode, self.x, [0, self.dt], args=(u,))[1,:]
        self.t = self.t + self.dt

        
    def get_measurement(self):
        return self.t, self.x[0] +random.randn()*0.01