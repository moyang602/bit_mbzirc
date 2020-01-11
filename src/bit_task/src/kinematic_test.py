from kinematics import *


print "Unit: radian"

current_joint = [-1.57,-1.57,-1.57,-1.57,1.57,0]
# current_joint = [-0.1, -0.2, -0.3, 0.5, -0.6, 0.7]

# pose =  fwd_kin(current_joint, i_unit='r', o_unit='ros')
# print pose
pose = [-0.0677, -0.632, -0.3674, 0, pi, 0]
print inv_kin(pose, current_joint, o_unit='r')
