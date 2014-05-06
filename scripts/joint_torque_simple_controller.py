#!/usr/bin/env python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Torque Example: joint springs
Modified by Dave Coleman
"""

import argparse
import baxter_control
import csv
import rospy
import sys

from dynamic_reconfigure.server import (
    Server,
    )
from std_msgs.msg import (
    Empty,
    )

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
    )

import baxter_control

class JointTorqueSimpleController(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointTorqueSimpleController class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty)

        # Create our PID controllers with gains all set to 0
        self._pid = dict()
        for joint in self._limb.joint_names():
            self._pid[joint] = baxter_control.PID(0.0, 0.0, 0.0)
            self._pid[joint].initialize()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def start_controller(self, trajectory):
        """
        Switches to joint torque mode and begin processing trajectory
        """
        print "start_controller"

        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # track which trajectory point we are on
        trajectory_id = 0

        # record the start time
        self._start_time = rospy.get_time()
        
        # loop at specified rate commanding new joint torques until we finish our trajectory
        while not rospy.is_shutdown():
            #print "trajectory_id:", trajectory_id, " of ", len(trajectory)

            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break

            # decide if its time to increment to new trajectory
            self._cur_time = rospy.get_time()

            if self._cur_time >= self._start_time + trajectory[trajectory_id]['time']:
                # time to process the next trajectory
                trajectory_id += 1
                print "Moving to trajectory", trajectory_id, "                

            # check if we are processing a trajectory
            if trajectory_id >= len(trajectory):
                # turn off controller because we are done with trajectory
                break
                
            # debug
            print trajectory[trajectory_id]

            # set our proportional gain
            joint_id = 0
            for jnt in self._limb.joint_names():
                #print "Kp set to ", trajectory[trajectory_id]['kp'][joint_id]
                self._pid[jnt].set_kp( trajectory[trajectory_id]['kp'][joint_id])
                joint_id += 1


            self._update_forces(trajectory[trajectory_id])
            control_rate.sleep()

    def _update_forces(self, trajectory_pt):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        # get position error
        deltas = self._get_current_error(self_._limb.joint_names(), trajectory_pt['position'])

        # calculate current forces -------------------------------------------------------
        # This is where our PID loop goes

        # Get torques from PIDs
        for delta in deltas:
            cmd[delta[0]] = self._pid[delta[0]].compute_output(delta[1])

        # --------------------------------------------------------------------------------

        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

def loadCSVTrajectory():
    trajectory = []
    
    with open('baxter_trajectory.csv', 'rb') as csvfile:
        trajectory_reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in trajectory_reader:
            trajectory_pt = {'time' : list(), 'position' : list(), 'torque' : list(), 'kp' : list() }
            trajectory_pt['time'] = row[0]
            trajectory_pt['position'] = row[1:8]
            trajectory_pt['torque'] = row[8:15]
            trajectory_pt['kp'] = row[15:23]
            #print trajectory_pt
            #print "-----------------"

            trajectory.append(trajectory_pt)

    #print trajectory
    #print "-----------------"
    return trajectory

def main():
    """Joint Torque Controller
    Dave Coleman

    Moves the specified limb to a neutral location and enters
    torque control mode, running a trajectory controller with values read from CSV
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', dest='limb', required=True, choices=['left', 'right'],
        help='limb on which to control'
        )
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("joint_torque_simple_controller_%s" % (args.limb,))

    # Dyn Reconfig
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)

    # Load CSV
    trajectory = loadCSVTrajectory()

    # Start controller
    js = JointTorqueSimpleController(args.limb, dynamic_cfg_srv)

    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)

    #sys.exit()
    js.move_to_neutral()
    js.start_controller(trajectory)


if __name__ == "__main__":
    main()
