from kinematics import *


print "Unit: radian"
current_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# print "SE(3) in numpy.array format"
# print fwd_kin(current_joint)

print "SE(3) in ROS Pose format"
print fwd_kin(current_joint, o_unit='p')


# print "Unit: radian"
# desired_solution = [1.7499912644507227, -1.8984856734885085, -1.1905715707581692, 0.21869218099676013, 2.9534288849387984, 0.45469538414663446]

# target_pose = [-0.062216135428015254, 0.7551066318135237, 0.8528777013335628, -1.3064880201804807, -1.078867334463253, 1.371140938984445]


# print "Joint values in radian"
# print inv_kin(target_pose, desired_solution)

# print "Joint values in degree"
# print inv_kin(target_pose, desired_solution, o_unit='d')
