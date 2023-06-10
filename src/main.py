# import libraries
import numpy as np
def pitch(quaternion):
    # Create Pitch Angle from Quaternions
    pitch = np.arcsin(2 * quat.w() * quat.y() - quat.x() * quat.z())

    # Convert Radians to Degrees */
    return  57.2958 * pitch

if(sensor1.calibrated and sensor2.calibrated){
    np.save('sensor1.npy', sensor1.);
}

quat1 = sensor1.quaternion
quat2 = sensor2.quaternion

leftAngle = pitch(quat1)
rightAngle = pitch(quat2)

if(leftAngle < -20):
    pass


if(rightAngle > 20):
    pass
