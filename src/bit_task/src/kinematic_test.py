from kinematics import *

current_joint = [-12, -75, 54, 67, 34, -53]
print current_joint
pose =  fwd_kin(current_joint, i_unit='d', o_unit='ros')

print inv_kin(pose, current_joint, i_unit='d', o_unit='d')
