#! /usr/bin/env python

import sys
import rospy
import gflags
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from test_carsim.msg import chassis_cmd

STEERING_LINE_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")


def callback(msg):
    global STEERING_LINE_DATA
    STEERING_LINE_DATA.append(msg.Ctrl_SW)
    if len(STEERING_LINE_DATA) > FLAGS.data_length:
        STEERING_LINE_DATA = STEERING_LINE_DATA[-FLAGS.data_length:]


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    steering_data = compensate(STEERING_LINE_DATA)
    steering_line.set_ydata(steering_data)
    steering_text.set_text('steering = %.1f' % steering_data[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)

    rospy.init_node('py_plot_control', 'anonymous=True')
    rospy.loginfo("control plot is launched.")
    rospy.Subscriber("/control", chassis_cmd, callback)

    fig, ax = plt.subplots()
    fig.suptitle('control')

    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()

    steering_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='steering')
    steering_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    ax.set_ylabel('steer_sw/deg')
    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_ylim(-100, 120)
    ax.set_xlim(-1 * FLAGS.data_length, 10)
    ax.legend(loc="upper left")
    plt.show()

    rospy.spin()
