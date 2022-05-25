#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    rospy.Subscriber("chatter", String, callback)



def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    # anonymous=True flag means that rospy will choose a unique name for our 'listener' node so that multiple
    # listeners can run simultaneously.
    rospy.init_node("MPC_red", anonymous=False)
    # need to add ~ before key name
    MPC_drive_topic = rospy.get_param("~MPC_drive_topic")
    scan_frame_red = rospy.get_param("~scan_topic_red")
    wheelbase = rospy.get_param("~wheelbase")
    friction_coeff = rospy.get_param("~friction_coeff")
    h_cg = rospy.get_param("~height_cg")
    l_f = rospy.get_param("~l_cg2front")
    l_r = rospy.get_param("~l_cg2rear")
    cs_f = rospy.get_param("~C_S_front")
    cs_r = rospy.get_param("~C_S_rear")
    mass = rospy.get_param("~mass")
    Iz = rospy.get_param("~moment_inertia")
    Cm1 = rospy.get_param("~empirical_drivetrain_parameters_1")
    Cm2 = rospy.get_param("~empirical_drivetrain_parameters_2")
    Cm3 = rospy.get_param("~empirical_drivetrain_parameters_3")
    B_f = rospy.get_param("~empirical_Pacejka_parameters_B_f")
    C_f = rospy.get_param("~empirical_Pacejka_parameters_C_f")
    D_f = rospy.get_param("~empirical_Pacejka_parameters_D_f")
    B_r = rospy.get_param("~empirical_Pacejka_parameters_B_r")
    C_r = rospy.get_param("~empirical_Pacejka_parameters_C_r")
    D_r = rospy.get_param("~empirical_Pacejka_parameters_D_r")
    max_accel = rospy.get_param("~max_accel")
    max_decel = rospy.get_param("~max_decel")
    max_speed = rospy.get_param("~max_speed")
    max_steering_vel = rospy.get_param("~max_steering_vel")
    max_steering_angle = rospy.get_param("~max_steering_angle")

    listener()
    rospy.loginfo("INNNN")
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()