from kinematics import *


print "Unit: radian"

current_joint = [-123, -111, 13, 21, 67, 83]

pose =  fwd_kin(current_joint, i_unit='d', o_unit='ros')

print inv_kin(pose, current_joint, o_unit='d')
