#!/usr/bin/env python2

import rospy
from irl_can_ros_ctrl.srv import SetAdmittance

class Admittance():
    """
    This utility class set the admittance of the arm motors
    of jn0 by calling the set_admittance services.
    Usage: call the set_admittance function with the wanted side and values
    Where values are the admittance values of
        [elbow_tilt_motor, shoulder_pan_motor, shoulder_roll_motor, shoulder_tilt_motor]
    """

    def __init__(self):
        """  basic initializer """
        self.namespace = "jn0"
        self.package = "jn0_driver"
        self.service_name = "set_admittance"
        self.motors = None

    def set_admittance(self, side, values):
        """ initialize side and set admittances to specified values """
        self.motors = [
            side + "_elbow_tilt_motor",
            side + "_shoulder_pan_motor",
            side + "_shoulder_roll_motor",
            side + "_shoulder_tilt_motor"
        ]

        services = list(map(lambda motor: \
            "/{0}/{1}/{2}/{3}" \
            .format(self.namespace, self.package, motor, self.service_name), \
            self.motors))
            # Wait for the services

        try:
            for service in services:
                rospy.wait_for_service(service, timeout=10)
        except rospy.ROSException:
            rospy.logwarn('Services timed out after 10 sec.')
            exit(1)

        # Create the proxies
        service_proxies = map(lambda service: rospy.ServiceProxy(service, SetAdmittance), \
                            services)

        # Set the admittance for each motors
        for i, service_proxy in enumerate(service_proxies):
            rospy.loginfo("Sending to service '{0}' m=0, b=0, k={1}".format(services[i], values[i]))
            service_proxy(0, 0, values[i])

        rospy.sleep(0.5) #Wait to validate new admittnace
