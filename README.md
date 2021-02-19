# solo12-recordings

Zipped file contains data logs of two walking experiments with quadruped Solo12.
The performed gait is a trotting gait with a period of 0.32s.
Legs are ordered as follows: front left, front right, hind left, hind right
Joints are ordered as follows for each leg: HAA, HFE, Knee

All units are expressed in the international system of units (m for positions, rad for angles, m/s or rad/s for velocities, Nm for torques)

The base frame is defined with the first axis pointing forwards, the second axis pointing sideways (to the left) and the third axis pointing upwards. It is located at the center of the robot's trunk.

The world frame is defined at ground level at the initial position of the robot. The initial position of the base frame in world frame is basically (0.0, 0.0, h_initial) with an identity rotation matrix.

The motion capture frame is defined by an L-shaped metal stick that is used to calibrate the motion capture system. If you want to compare data in world frame and motion capture frame you have to re-align the data in translation and rotation. During the two experiments the quadruped moves mostly along the Y axis of the motion capture.

Fields in recordings:
* q_mes : measured position of the 12 actuators
* v_mes : measured velocity of the 12 actuators
* torquesFromCurrentMeasurment : measured torques for the 12 actuators (from motor current measurements with reduction ration of 9 and torque constant of 0.025)
* baseOrientation : orientation of the base measured by the onboard Inertial Measurement Unit (IMU) as a quaternion (x,y,z,w)
* baseAngularVelocity : angular velocity of the base measured by the onboard IMU
* baseLinearAcceleration : linear acceleration of the base measured by the onboard IMU (gravity removed)
* baseAccelerometer : linear acceleration of the base measured by the onboard IMU (gravity included)
* mocapPosition : position of the base in the motion capture frame and measured with motion capture system (ground truth)
* mocapVelocity : linear velocity of the base in the motion capture frame and measured with motion capture system (ground truth)
* mocapAngularVelocity : angular velocity of the base in the motion capture frame and measured with motion capture system (ground truth)
* mocapOrientationMat9 : orientation of the base in the motion capture frame and measured with motion capture system (ground truth), expressed as a 3 by 3 matrix
* mocapOrientationQuat : orientation of the base in the motion capture frame and measured with motion capture system (ground truth), expressed as a quaternion
* estimatorVelocity : linear velocity of the base in base frame and estimated by the state estimator of the controller
* contactStatus : desired contact status of feet (1: stance phase, 0: swing phase)
* referenceVelocity : reference velocity that the robot should follow, expressed in base frame (Vx, Vy, Vz, Wroll, Wpitch, Wyaw). Controlled manually with a joystick during the experiments.
* tstamps : timestamps of the logging process (control runs at 500 Hz so roughly 0.002s between each log)
* log_feet_pos : estimated position of feet in world frame (x,y,z) for (FL, FR, HL, HR) feet
* log_feet_pos_target : desired position of feet in world frame (x,y,z) for (FL, FR, HL, HR) feet
* log_feet_vel_target : desired velocity of feet in world frame (x,y,z) for (FL, FR, HL, HR) feet
* log_feet_acc_target : desired acceleration of feet in world frame (x,y,z) for (FL, FR, HL, HR) feet
* log_q : estimated state of the robot in world frame (x,y,z,roll,pitch,yaw,12actuators)
* log_dq : estimated velocity of the robot in world frame (v_x,v_y,v_z,w_roll,w_pitch,w_yaw,v_12actuators)
* log_tau_ff : desired feedforward torques for the 12 actuators (output of whole body control)
* log_qdes : desired positions for the 12 actuators (output of whole body control)
* log_vdes : desired velocities torques for the 12 actuators (output of whole body control)

Desired torques are tau = tau_ff + P * (qdes - q_mes) + D * (vdes - v_mes). Sadly I forgot to log P and D values when I did the recordings, I think I used P = 6.0 and D = 0.2. 