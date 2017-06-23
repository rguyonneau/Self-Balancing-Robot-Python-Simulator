#!/usr/bin/env python3

""" main.py uses OpenGL to display a IBallancingBot and uses the PID class to
    provide a command for the robot.
    This file is based on the work of Jean-Baptiste Fasquel,
    ISTIA - Angers university Jean-Baptiste.Fasquel@univ-angers.fr

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

import sys as sys;
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from math import cos
from math import sin
from math import pi
from math import sqrt
from iballancingbot import IBalancingBot
from pid import PID


print("<-------------------------------------------------------------->")
print("<--                    IBBOT simulator                       -->")
print("<-- developped by Remy GUYONNEAU, ISTIA/LARIS                -->")
print("<-- University of Angers                                     -->")
print("<-- The model considered is from                             -->")
print("<-- 'A comparision of Controllers for Ballancing Two Wheeled -->")
print("<--    Inverted Pendulum Robot'                              -->")
print("<-- Amir A. Bature, Salinda Buyamin, Mohammed. N. Ahmad,     -->")
print("<--    Mustapha Muhammad, Universiti Teknologi Malaysia      -->")
print("<-------------------------------------------------------------->\n")

print(" Page down : activate simulation")
print(" F1 : move backward")
print(" F2 : move forward")
print(" F3 : turn")
print(" F4 : stop motion")
print(" F5 : ON/OFF PID correction")
print(" F6 : reset simulation")
print(" F7 : add pertubation (push the robot)")
print(" F8 : add pertubation (push the robot)")
print(" F9 : ON/OFF follow the robot")

# -- Programme variables --

# variables for the animation of the dynamics
# ref_time : to compute the time between two redisplay$
# FPS : Frame per second, to 
ref_time, FPS = 0, 10

# position of the camera (glulookat parameters)
CameraPosX = 0.0
CameraPosY = 3
CameraPosZ = 10.0
ViewUpX = 0.0
ViewUpY = 1.0
ViewUpZ = 0.0
CenterX = 0.0
CenterY = 0.0
CenterZ = 0.0
follow_robot = False  # so that the camera will follow the robot or not

# to deal with the camera rotation and zoom
Theta,dtheta=0.0,2*pi/100.0
Radius = sqrt( CameraPosX**2+CameraPosZ**2)

# to draw cylinder
quadric = gluNewQuadric()
gluQuadricNormals(quadric, GLU_SMOOTH)

# the balancing robot
myBot = IBalancingBot()

# for the PIDs correction:
#   1 PID for the pendulum angle (phi)
#   1 PID for the linear x speed (xp)
#   1 PID for the robot angular speed rotation (psip)
myPIDphi = PID()
myPIDx = PID()
myPIDpsi = PID()

# to draw the robot at its actual position in the world
posx = posz = 0

# the command of the robot's wheels
F = [0, 0]

# activate or distactivate the PIDs correction
use_pid = False

# to deal with the "moving forwars" and "turning" commands
speed = 0
current_speed = 0
turn = 0
current_turn = 0

def initPIDs():
    """ Function that initializes the PIDs parameters (Kp, Ki, Kd)
    """
    global myPIDphi, myPIDx, myPIDpsi

    myPIDphi.setKp(7.0)
    myPIDphi.setKi(0.1)
    myPIDphi.setKd(6.0)
    myPIDphi.setPoint(0)

    myPIDx.setKp(0.01)
    myPIDx.setKi(0.005)
    myPIDx.setKd(0.01)
    myPIDx.setPoint(0)

    myPIDpsi.setKp(1)
    myPIDpsi.setKi(1)
    myPIDpsi.setKd(0)
    myPIDpsi.setPoint(0)
    

def correction():
    """ Function that uses the PID and the robot state to generate a new 
        command according to the speed and turn objectives
    """
    global myBot, F, myPIDx, myPIDphi, myPIDpsi, use_pid
    global speed, current_speed, turn, current_turn

    if(use_pid):
        if(current_speed != speed):
            current_speed = speed
            myPIDx.setPoint(speed)  # we only want to reset the PID when the speed changes

        if(current_turn != turn):
            current_turn = turn
            myPIDpsi.setPoint(turn)  # we only want to reset the PID when the rotation changes

        pidx_value = myPIDx.update(myBot.xp)  # Pid over linear a speed
        pidpsi_value = myPIDpsi.update(-myBot.psip)  # Pid over psi angular speed rotation

        tilt = - pidx_value + myBot.phi
        rotation = pidpsi_value

        pidphi_value = myPIDphi.update(tilt)  # pid over the pendulum angle phi

        F = [-pidphi_value-rotation,-pidphi_value+rotation]
    else:
        F = [0, 0]


def animation():
    """ Function to compute the robot state at each time step and to draw it in the world
    """
    global ref_time, FPS, F, posx, posz, myBot
    # FPS expressed in ms between 2 consecutive frame
    delta_t = 0.001 # the time step for the computation of the robot state
    if glutGet(GLUT_ELAPSED_TIME)-ref_time > (1.0/FPS)*1000 :
        # at each redisplay (new display frame)
        dst = 0
        for i in range(0,100):
            # we want the computation of the robot state to be faster than the 
            # display to limit the compution errors
            # display : new frame at each 100ms
            # deltat : 1ms for the differential equation evaluation
            myBot.dynamics(delta_t, F)
            # we also compute the new x and z position of the robot in the world
            dst = (myBot.xp * delta_t)
            posx += dst*cos(myBot.psi)
            posz += (-dst*sin(myBot.psi))

        correction()  # calls the PIDs if enable
        glutPostRedisplay()  # refresh the display
        ref_time=glutGet(GLUT_ELAPSED_TIME)

def drawGround():
    """ Function to draw the ground
    """
    nb_rows = 20
    nb_cols = 20
    for r in range(0,nb_rows):
        for c in range(0,nb_cols):
            if(r%2 and not c%2 or not r%2 and c%2):
                glColor3d(0.3,0.3,0.3)
            else:
                glColor3d(0.1,0.1,0.1)
            glBegin( GL_QUADS )
            glVertex3f(c-nb_cols/2, 0, r-nb_rows/2)
            glVertex3f(c-nb_cols/2+1, 0, r-nb_rows/2)
            glVertex3f(c-nb_cols/2+1, 0, r-nb_rows/2+1)
            glVertex3f(c-nb_cols/2, 0, r-nb_rows/2+1)
            glEnd()

def drawIBot(ibot):
    """ Function to draw the robot
        ibot: the robot (IBalancingBot) to draw
    """
    global myBot
    drawWheels(ibot.d_dstw, ibot.d_widthw, ibot.d_rw)  # draw the wheels
    drawBase(ibot.d_dstw, ibot.d_widthp)  # draw the robot body
    drawPendulum(ibot.d_heightp, ibot.d_widthp, ibot.d_centerp)  # draw the pendulum

def drawWheels(dst_wheels, width_wheel, radius_wheel):
    """ Function to draw the robot's wheels
        dst_wheels : distances between the two wheels
        width_wheel : width of the wheels
        radius_wheel : radius of the wheels
    """
    global quadric  # to use gluCylinder to draw cylinder
    # draw the first wheel
    glPushMatrix()
    glTranslatef(0, 0, -dst_wheels/2)
    drawWheel(radius_wheel/2, width_wheel) 
    glPopMatrix()
    # draw the second wheel
    glPushMatrix()
    glTranslatef(0, 0, dst_wheels/2)
    drawWheel(radius_wheel/2, width_wheel)
    glPopMatrix()


def drawWheel(size, width):
    """ Function to draw a wheel (a cylinder and two disks to close it)
        size : size of the wheel (radius)
        width : width of the wheel (width of the cylinder)
    """
    global quadric
    # draw cylinder of the wheel
    glPushMatrix()
    glColor3d(0.4,0.4,0.4)
    glTranslatef(0,0,-width/2)
    gluCylinder(quadric, size, size, width,32,16)
    glPopMatrix()
    # draw the first disk to close the cylinder
    glPushMatrix()
    glColor3d(0.6,0.6,0.6)
    glTranslatef(0,0,-width/2)
    gluDisk(quadric, 0,size,32,32)
    glPopMatrix()
    # draw the second disk to close the cylinder
    glPushMatrix()
    glColor3d(0.6,0.6,0.6)
    glTranslatef(0,0,width/2)
    gluDisk(quadric, 0,size,32,32)
    glPopMatrix()

def drawBase(dst_wheels, radius_pipe):
    """ Function to draw the body of the robot (a cylinder between the wheels)
        dst_wheels : distance between the wheels
        radius_pipe : the width of the cylinder body
    """
    global quadric
    glPushMatrix()
    glColor3d(0.4,0.4,1)
    glTranslatef(0, 0, -dst_wheels/2)
    gluCylinder(quadric,radius_pipe/2, radius_pipe/2, dst_wheels,32,16)
    glPopMatrix()

def drawPendulum(height_pendulum, radius_pipe, center_pendulum):
    """ Function to draw the pendulum of the robot (a cylinder up)
        height_pendulum : height of the pendulum from the center of the wheels
        radius_pipe : the width of the pendulum cylinder
        center_pendulum :  height of the center of mass of the pendulum from the center of the wheels
    """
    global quadric
    # draw the pendulum
    glPushMatrix()
    glColor3d(1,0.4,0.4)
    glRotatef(90,-1,0,0)
    gluCylinder(quadric,radius_pipe/2, radius_pipe/2, height_pendulum,32,16)
    glPopMatrix()
    # draw the center of mass
    glPushMatrix()
    glColor3d(0.4,1,0.4)
    glTranslatef(0, center_pendulum, 0)
    glutSolidSphere(radius_pipe, 32, 32)
    glPopMatrix()


def Displayfct():
    """ Function to draw the world
    """

    scaleCoeff = 10  # to scale the robot display

    glClearColor(0,0,0,0)  # background color
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1,1,-1,1,1,80.)

    # Camera position and orientation
    global CameraPosX, CameraPosY, CameraPosZ, CenterX, CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ
    global myBot, posx, posz, follow_robot

    if(follow_robot):
        CenterX = -posx*scaleCoeff
        CenterZ = -posz*scaleCoeff

    gluLookAt(CameraPosX, CameraPosY , CameraPosZ, CenterX, CenterY, CenterZ, ViewUpX, ViewUpY, ViewUpZ)

    # Draw the objects and geometrical transformations
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


    # draw the ground
    glPushMatrix()
    glScalef(scaleCoeff, scaleCoeff, scaleCoeff)  # to scale the ground as the robot
    glTranslatef(0,-myBot.d_rw/2,0)
    drawGround()
    glPopMatrix()

    # draw the robot
    glPushMatrix()
    glScalef(scaleCoeff, scaleCoeff, scaleCoeff)  # to scale the robot
    glTranslatef(-posx,0,-posz)
    glRotatef(myBot.psi*180/pi, 0, 1, 0)  # psi rotation (robot)
    glRotatef(myBot.phi*180/pi, 0, 0, 1)  # phi rotation (pendulum)
    drawIBot(myBot)
    glPopMatrix()

    # To efficient display
    glutSwapBuffers()


def ReshapeFunc(w,h):
    """ Function calls when reshaping the window
    """
    glViewport(0,0,w,h)

def rotate_camera(angle):
    """ Function to rotate the camera according to an angle
        angle : step angle for the camera rotation
    """
    global CameraPosX,CameraPosZ,Radius,Theta
    Theta+=angle
    CameraPosZ=Radius*cos(Theta)
    CameraPosX=Radius*sin(Theta)
    return 0

def zoom_camera(factor):
    """ Function to zoom the camera according to a factor
        factor : zoom factor
    """
    global CameraPosX,CameraPosY,CameraPosZ,Radius
    # Update camera center
    CameraPosX, CameraPosY, CameraPosZ = factor*CameraPosX, factor*CameraPosY, factor*CameraPosZ
    # Update radius (for next rotations)
    Radius = sqrt( CameraPosX**2 + CameraPosZ**2 )

def SpecialFunc(skey,x,y):
    """ Function to handle the keybord keys
    """
    global CameraPosY, Theta, dtheta
    global myBot, speed, use_pid, turn
    global posx, posz, follow_robot

    if glutGetModifiers() == GLUT_ACTIVE_SHIFT:
        # SHIFT pressed
        if skey == GLUT_KEY_UP :
            CameraPosY+=0.3  # put the camera higher
        if skey == GLUT_KEY_DOWN :
            CameraPosY-=0.3  # put the camera lower
    else:
        # standard
        if skey == GLUT_KEY_LEFT :
            rotate_camera(-dtheta)
        elif skey == GLUT_KEY_RIGHT :
            rotate_camera(dtheta)
        elif skey == GLUT_KEY_UP :
            zoom_camera(0.9)
        elif skey == GLUT_KEY_DOWN :
            zoom_camera(1.1)
        elif skey == GLUT_KEY_PAGE_DOWN : 
            print("\tSimulation ON")
            glutIdleFunc(animation)
        elif skey == GLUT_KEY_PAGE_UP :
            print("\tSimulation PAUSE")
            glutIdleFunc(None)
        elif skey == GLUT_KEY_F1 :
            speed = 0.15
        elif skey == GLUT_KEY_F2 :
            speed = -0.15
        elif skey == GLUT_KEY_F3 : 
            turn = 0.3
        elif skey == GLUT_KEY_F4 : 
            speed = 0
            turn = 0
        elif skey == GLUT_KEY_F5 : 
            use_pid = not(use_pid)
            print("\tUsing PID : " + str(use_pid))
            if(use_pid == True):
                initPIDs()
        elif skey == GLUT_KEY_F6 : 
            posx = posz = 0
            myBot.initRobot()
            initPIDs()
        elif skey == GLUT_KEY_F7 : 
            myBot.phi += (10*pi/180)
        elif skey == GLUT_KEY_F8 : 
            myBot.phi += (-20*pi/180)
        elif skey == GLUT_KEY_F9 : 
            follow_robot = not(follow_robot)
            print("\tFollowing robot : " + str(follow_robot))
    glutPostRedisplay()
    return 0

# Initialization of the Window stuff
glutInit(sys.argv)
glutInitWindowPosition(100,100)
glutInitWindowSize(250,250)
glutInitDisplayMode(GLUT_RGBA |GLUT_DOUBLE | GLUT_DEPTH)
glutCreateWindow(b"IBBot simulator")

# Opengl Initialization      
glClearColor(0.0,0.0,0.0,0.0)
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
glEnable(GL_DEPTH_TEST) # Hidden objects are not drawn

# Display are reshape function
glutDisplayFunc(Displayfct)
glutReshapeFunc(ReshapeFunc)
glutSpecialFunc(SpecialFunc)

# Infinite loop
glutMainLoop()
