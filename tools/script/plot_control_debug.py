#! /usr/bin/env python

import sys
import rospy
import gflags
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from test_carsim.msg import chassis_data, simple_mpc_debug

LATERAL_ERROR_LINE_DATA = []
HEADING_ERROR_LINE_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Debug control plot data length")


# def callback_chassis_data(msg):


def callback_mpc_debug(msg):
    global LATERAL_ERROR_LINE_DATA, HEADING_ERROR_LINE_DATA

    LATERAL_ERROR_LINE_DATA.append(msg.lateral_error)
    if len(LATERAL_ERROR_LINE_DATA) > FLAGS.data_length:
        LATERAL_ERROR_LINE_DATA = LATERAL_ERROR_LINE_DATA[-FLAGS.data_length:]

    HEADING_ERROR_LINE_DATA.append(msg.heading_error)
    if len(HEADING_ERROR_LINE_DATA) > FLAGS.data_length:
        HEADING_ERROR_LINE_DATA = HEADING_ERROR_LINE_DATA[-FLAGS.data_length:]


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lateral_error_data = compensate(LATERAL_ERROR_LINE_DATA)
    lateral_error_line.set_ydata(lateral_error_data)

    heading_error_data = compensate(HEADING_ERROR_LINE_DATA)
    heading_error_line.set_ydata(heading_error_data)

    lateral_error_text.set_text('lateral_error = %.1f' %
                                lateral_error_data[-1])
    heading_error_text.set_text('heading_error = %.1f' %
                                heading_error_data[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)

    rospy.init_node('py_plot_control_debug', 'anonymous=True')
    rospy.loginfo("debug control plot is launched.")
    # rospy.Subscriber("/chassis_data", chassis_data, callback_chassis_data)
    rospy.Subscriber("/mpc_debug", simple_mpc_debug, callback_mpc_debug)

    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()

    fig, ax = plt.subplots(nrows=2, ncols=2, figsize=(12, 7))
    fig.suptitle('MPC_DEBUG')

    lateral_error_line, = ax[0][0].plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='lateral_error')
    lateral_error_text = ax[0][0].text(
        0.60, 0.90, '', transform=ax[0][0].transAxes)
    ax[0][0].set_title('Lateral Error')
    ax[0][0].set_ylabel('e_y/m')
    ax[0][0].set_ylim(-0.2, 0.2)
    ax[0][0].set_xlim(-1 * FLAGS.data_length, 10)
    ax[0][0].legend(loc="upper left")

    heading_error_line, = ax[0][1].plot(
        Xs, [0] * FLAGS.data_length, 'g', lw=3, alpha=0.5, label='heading_error')
    heading_error_text = ax[0][1].text(
        0.60, 0.90, '', transform=ax[0][1].transAxes)
    ax[0][1].set_title('Heading Error')
    ax[0][1].set_ylabel('e_psi/rad')
    ax[0][1].set_ylim(-0.1, 0.1)
    ax[0][1].set_xlim(-1 * FLAGS.data_length, 10)
    ax[0][1].legend(loc="upper left")

    ani = animation.FuncAnimation(fig, update, interval=100)

    plt.show()
    rospy.spin()
