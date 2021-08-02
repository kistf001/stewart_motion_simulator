from .ACC import ACC_sMEM
import time

AngularVelocity  = [0,0,0]
Velocity = [0,0,0]
info = 0

def inits():
    global info
    info = ACC_sMEM.SimInfo()

def update():

    global info, AngularVelocity
    AngularVelocity[0] = info.physics.localAngularVel[0]
    AngularVelocity[1] = info.physics.localAngularVel[1]
    AngularVelocity[2] = info.physics.localAngularVel[2]
    Velocity[0] = info.physics.accG[0]
    Velocity[1] = info.physics.accG[1]
    Velocity[2] = info.physics.accG[2]
    time.sleep(0.01)


