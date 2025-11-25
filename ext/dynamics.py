################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

from math import sin, cos, sqrt, tan, atan, atan2, fmod, pi
import numpy as np

from src.settings import * 
from src.sensing import clamp

def stanley(v, Ye, Le):
    '''Run Stanley-like controller as based from Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing." American Control Conference. 2007, pp. 2296–2301. doi:10.1109/ACC.2007.4282788'''
    k1 = 1.05
    k2 = 2.50
    k3 = 1.00
    
    e = 1.10

    return clamp(k1*Ye + atan(k2*Le / (e + k3*v)), -35.*pi/180., 35.*pi/180.)

def step_dyn(dt, t, x, y, v, a, heading, ua, usteer=0.):
    """
    Apply control u and step forward the dynamics using explicit Runge-Kutta integrator

    Returns:
    t, x, y, v, a, heading
    """
    z = RK4( dyn, dt, t, np.array([x, y, v, a, heading]), np.array([ua, usteer]) )

    t += dt

    # Check for instability in acceleration from discrete lag filter
    if z[3] < -10. or z[3] > 10:
        print(' !!! Unstable acceleration detected - a={:0.2f}'.format(z[3]))
        z[3] = 10. * np.sign(z[3])

    # Prevent reversing and if detected default to previous states
    if z[2] < 0:
        v = 0.
        a = 0.

    else:
        x = z[0]
        y = z[1]
        v = z[2]
        a = z[3]
        heading = z[4]

    return t, x, y, v, a, heading

def dyn(t, x, u):
    '''
    Simulates a kinematic bicycle model with a first-order forward acceleration lag
    x = [x, y, v, a, heading], 
    u = [a, steer]
    '''
    # acceleration TAUINV \in [3.0 5.0]
    # braking TAUINV \in [5.0 10.0]
    TAUINV = 5.0 # First order lag on acceleration tracking
    da = TAUINV*( u[0]-x[3] )
    
    Lr, Lf = 0.5*VEHLENGTH, 0.5*VEHLENGTH
    beta = atan( Lr/(Lf+Lr)*tan(u[1]) )
    
    return np.array([
        x[2]*sin(x[4]+beta),
        x[2]*cos(x[4]+beta),
        x[3],
        da,
        x[2]/Lr*sin(beta)
    ])

def RK2(f, dt, t, x, u):
    '''Single-step explicit second-order Runge-Kutta integrator'''
    k1 = dt*f(t, x, u)
    k2 = dt*f(t+0.5*dt, x+0.5*k1, u)

    t = t+dt
    x = x+0.5*(k1+k2)
    return x

def RK4(f, dt, t, x, u):
    '''Single-step explicit fourth-order Runge-Kutta integrator'''
    k1 = dt*f(t, x, u)
    k2 = dt*f(t+0.5*dt, x+0.5*k1, u)
    k3 = dt*f(t+0.5*dt, x+0.5*k2, u)
    k4 = dt*f(t+dt, x+k3, u)
    
    t = t+dt
    x = x+0.16666667*(k1+2.*k2+2.*k3+k4)
    return x
