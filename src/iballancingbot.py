#!/usr/bin/env python3

""" ibalancingbot.py details a class to simulate a self balancing robot
    behavior.It uses a runge-kutta approach to deal with the system dynamic.

    Copyright (C) 2017 
    Remy GUYONNEAU, ISTIA/LARIS, University of Angers (France)
    remy.guyonneau@univ-angers.fr

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>."""

from math import cos
from math import sin
from math import pi
from math import atan

class IBalancingBot:
    """ A class to simulate the behavior of a self balancing robot
        The mathematical model is from the article : 
            'A comparison of Controllers for Balancing Two Wheeled Inverted 
            Pendulum Robot', Amir A. Bature, Salinda Buyamin, Mohammed N. Ahmad,
            Mustapha Muhammad.
    """

    def __init__(self):
        """ The constructor of the class """

        # variables for the model
        self.Mb = None  # kg      mass of main body (pendulum)
        self.Mw = None  # kg      mass of wheels
        self.d = None   # m       center of mass from base
        self.R = None   # m       Radius of wheel
        self.L = None   # m       Distance between the wheels
        self.Ix = None  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = None  # kg.m^2  Moment of inertia of body z-axis
        self.Ia = None  # kg.m^2  Moment of inertia of wheel according to center
        self.g = None   # m/s^2   Acceleration due to gravity

        # variables for dynamic evaluation
        self.phi = None    # angle of the pendulum
        self.phip = None   # angular speed of the pendulum
        self.phipp = None  # angular acceleration of the pendulum
        self.x = None      # x position of the robot
        self.xp = None     # linear x speed of the robot
        self.xpp = None    # linear x acceleration of the robot
        self.psi = None    # rotation of the robot
        self.psip = None   # rotation angular speed of the robot
        self.psipp = None  # rotation angular acceleration of the robot

        # variables for the drawing
        self.d_rw = None       # wheel radius
        self.d_dstw = None     # distance between the wheels
        self.d_widthw = None   # width of a wheel
        self.d_heightp = None  # height of the pendulum
        self.d_centerp = None  # distance between the wheel and the pendulum center
        self.d_widthp = None   # width of the robot pipes

        self.initRobot()  # to initialize all the parameters of the system

    def initRobot(self):
        """ Function to initialize all the parameters of the system
            The value presented here are the value from the paper where the 
            model come from.
        """
        # variables for the model
        self.Mb = 13.3    # kg      mass of main body (pendulum)
        self.Mw = 1.89    # kg      mass of wheels
        self.d = 0.13     # m       center of mass from base
        self.R = 0.130    # m       Radius of wheel
        self.L = 0.325    # m       Distance between the wheels
        self.Ix = 0.1935  # kg.m^2  Moment of inertia of body x-axis
        self.Iz = 0.3379  # kg.m^2  Moment of inertia of body z-axis
        self.Iy = 0.3379  # kg.m^2  Moment of inertia of body y-axis : NOT A VALUE FROM THE PAPER
        self.Ia = 0.1229  # kg.m^2  Moment of inertia of wheel according to center
        self.g = 9.81     # m/s^2   Acceleration due to gravity  

        self.phi = 2*pi/180  # the pendulum is initially unstable 
        self.phip = 0

        self.x = 0
        self.xp = 0

        self.psi = 0
        self.psip = 0

        # variables for the drawing
        self.d_rw = self.R
        self.d_dstw = self.L
        self.d_widthw = 0.01
        self.d_heightp = self.d
        self.d_centerp = self.d
        self.d_widthp = 0.01

        # to deal with the fact that the pendulum can not be lower than the ground
        # we compute the maximal possible angle for the pendulum
        self.phimax = pi/2 + atan(self.R/(2*self.d))
        self.phimin = -pi/2 - atan(self.R/(2*self.d))

    def f(self, phi, phip, x, xp, psi, psip, deltat, F):
        """ Function to evaluate the new state of the system according to:
            - the mathematical model (phipp=, xpp=, psipp=)
            - the current state (phi, phip, x, xp, psi, psip)
            - the time step deltat
            - the command F=[tau1, tau2] (constant over deltat)
            It returns phip, phipp, xp, xpp, psip, psipp
        """

        tau1 = F[0]
        tau2 = F[1]
        
        # to ease the reading
        Mb = self.Mb
        Mw = self.Mw
        d  = self.d
        R  = self.R
        L  = self.L
        Ix = self.Ix
        Iz = self.Iz
        Iy = self.Iy
        Ia = self.Ia
        g  = self.g

        R2 = R**2
        d2 = d**2

        # computation of phipp
        den_phipp = ((R*Mb*d*sin(phi))**2)+((Mb+2*Mw)*(R2)+2*Ia)*Ix+2*Mb*d2*(Mw*R2+Ia)

        phipp = ((Mb*d2+Iy-Iz)*(Mb*R2+2*Mw*R2+2*Ia)*sin(phi)*cos(phi))*(psip**2)/den_phipp \
                - (Mb**2)*d2*R2*sin(phi)*cos(phi)*(phip**2)/den_phipp \
                + (Mb*R2+2*Mw*R2+2*Ia)*Mb*g*d*sin(phi)/den_phipp \
                - ((Mb*R2+2*Mw*R2+2*Ia)+Mb*d*R*cos(phi))*(tau1+tau2)/den_phipp
                
        # computation of xpp
        den_xpp = (Mb*d2+Ix)*(Mb*R2+2*Mw*R2+2*Ia)-((Mb*d*R*cos(phi))**2)

        xpp = (Mb*d*R*cos(phi)*(Mb*d2+Iy-Iz)*sin(phi)*cos(phi)*(psip**2))/den_xpp \
              - ((Mb**2)*d2*g*R2*sin(phi)*cos(phi))/den_xpp \
              + (R2*(Mb*d2+Ix)*Mb*d*sin(phi)*(phip**2))/den_xpp \
              + (R*(Mb*d2+Ix+Mb*d*R*cos(phi))*(tau1+tau2))/den_xpp
        # computation of psipp
        den_psipp = 2*(Mw+Ia/R2)*(L**2)+Iy*(sin(phi)**2)+Iz*(cos(phi)**2)+Mb*d2*sin(phi)

        psipp = (L*(tau1-tau2))/(R*den_psipp) \
                - (2*(Mb*d2+Iy-Iz)*sin(phi)*cos(phi)*psip*phip)/den_psipp

        return phip*deltat, phipp*deltat, xp*deltat, xpp*deltat, psip*deltat, psipp*deltat


    def runge_kutta(self, deltat, F):
        """ Function that call the f() function defined above, and uses a 
            runge-kutta approach to deal with the differential equations
        """
        k1phip, k1phipp, k1xp, k1xpp, k1psip, k1psipp = \
            self.f(self.phi, self.phip, self.x, self.xp, self.psi, self.psip, deltat, F)
        k2phip, k2phipp, k2xp, k2xpp, k2psip, k2psipp = \
            self.f(self.phi + k1phip/2, self.phip + k1phipp/2, self.x + k1xp/2, self.xp + k1xpp/2, self.psi + k1psip/2, self.psip + k1psipp/2, deltat, F)
        k3phip, k3phipp, k3xp, k3xpp, k3psip, k3psipp = \
            self.f(self.phi + k2phip/2, self.phip + k2phipp/2, self.x + k2xp/2, self.xp + k2xpp/2, self.psi + k2psip/2, self.psip + k2psipp/2, deltat, F)
        k4phip, k4phipp, k4xp, k4xpp, k4psip, k4psipp = \
            self.f(self.phi + k3phip, self.phip + k3phipp, self.x + k3xp, self.xp + k3xpp, self.psi + k3psip, self.psip + k3psipp, deltat, F)

        self.phi = self.phi + 1/6.0*(k1phip+2*k2phip+2*k3phip+k4phip)
        self.phip = self.phip + 1/6.0*(k1phipp+2*k2phipp+2*k3phipp+k4phipp)

        self.xp = self.xp + 1/6.0*(k1xpp+2*k2xpp+2*k3xpp+k4xpp)
        self.x = self.x + 1/6.0*(k1xp+2*k2xp+2*k3xp+k4xp)

        self.psip = self.psip + 1/6.0*(k1psipp+2*k2psipp+2*k3psipp+k4psipp)
        self.psi = self.psi + 1/6.0*(k1psip+2*k2psip+2*k3psip+k4psip)

        # to deal with the fact that the pendulum can not be lower than the ground.
        # we also put the acceleration to 0 to avoid that the robot moves
        #   when the pendulum is down
        if (self.phi > self.phimax):
            self.phi = self.phimax
            self.phip = 0
            self.xp = 0
        if (self.phi < self.phimin):
            self.phi = self.phimin
            self.phip = 0
            self.xp = 0


    def dynamics(self, deltat, F):
        """ Function to provide an interface between the model and the display
        """
        self.runge_kutta(deltat, F)

