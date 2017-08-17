#!/usr/bin/env python
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import sys
from uuv_control_msgs.srv import GoTo
from uuv_control_msgs.srv import GoToIncremental
from uuv_control_msgs.msg import Waypoint
from std_msgs.msg import String, Time, Int8
from geometry_msgs.msg import Point

def receive_to_go_direction_callback(data):
    sender(data)

def sender(p):

    try:
        rospy.wait_for_service(rospy.get_namespace()+'go_to_incremental', timeout=2)
    except rospy.ROSException:
        rospy.ROSException('Service not available! Closing node...')

    try:
        init_wp = rospy.ServiceProxy(
            rospy.get_namespace()+'/go_to_incremental',
            GoToIncremental )

    except rospy.ServiceException, e:
        rospy.ROSException('Service call failed, error=' + e)

    #Point step
    step = Point(0,0,0)
    step.z = 0

    if p.data == 0:
        step.x = -1
        step.y = -1
    elif p.data == 1:
        step.x = -1
        step.y = 0
    elif p.data == 2:
        step.x = -1
        step.y = 1
    elif p.data == 3:
        step.x = 0
        step.y = -1
    elif p.data == 4:
        step.x = 0
        step.y = 0
    elif p.data == 5:
        step.x = 0
        step.y = 1
    elif p.data == 6:
        step.x = 1
        step.y = -1
    elif p.data == 7:
        step.x = 1
        step.y = 0
    elif p.data == 8:
        step.x = 1
        step.y = 1
    else:
        step.x = 0
        step.y =  0

    success = init_wp(step, 0.4)

    if success:
        print 'Incremental command successfully sent'
    else:
        print 'Failed'

if __name__ == '__main__':

    print 'Send a waypoint, namespace=', rospy.get_namespace()
    rospy.init_node('move_to_next_step.py')

    if rospy.is_shutdown():
        print 'ROS master not running!'
        sys.exit(-1)

    #if rospy.has_param('~filename'):
    #    filename = rospy.get_param('~filename')
    #else:
    #    raise rospy.ROSException('No filename found')

    # If no start time is provided: start *now*.
    start_time = rospy.Time.now().to_sec()
    start_now = True
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
        if start_time < 0.0:
            print 'Negative start time, setting it to 0.0'
            start_time = 0.0
            start_now = True
    else:
        start_now = True
    
    rospy.Subscriber(rospy.get_namespace()+"direction_to_go", Int8, receive_to_go_direction_callback)
    rospy.spin()
