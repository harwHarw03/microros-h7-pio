#!/usr/bin/env python3
import sys
from collections import deque
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from imu_visualize.msg import Euler

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

class ExecutorThread(QtCore.QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor
    def run(self):
        self.executor.spin()

class PGEulerPlot(Node):
    def __init__(self):
        super().__init__('pg_euler_plot')
        self.maxlen = 100
        self.data = {k: deque(maxlen=self.maxlen) for k in ['roll','pitch','yaw']}

        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title=" Euler (RPY)")
        self.win.resize(800, 600)
        self.win.setWindowTitle('Euler Plotter')

        self.curves = {}
        plot_pens = ['#0072B2', '#D55E00', '#009E73']
        for i, key in enumerate(['roll','pitch','yaw']):
            p = self.win.addPlot(row=i, col=0, title=key.capitalize())
            p.setYRange(-190, 190, padding=0)
            p.showGrid(x=True, y=True)
            p.getAxis('left').setLabel('Degrees')
            curve = p.plot(pen=plot_pens[i])
            self.curves[key] = curve

        self.create_subscription(Euler, 'imu/euler', self.euler_cb, 10)
        self.get_logger().info("Listening for Euler messages on /imu/euler...")

        self.qt_timer = QtCore.QTimer()
        self.qt_timer.timeout.connect(self.update_plot)
        self.qt_timer.start(33)

    def euler_cb(self, msg: Euler):
        self.data['roll'].append(math.degrees(msg.roll))
        self.data['pitch'].append(math.degrees(msg.pitch))
        self.data['yaw'].append(math.degrees(msg.yaw))

    def update_plot(self):
        for key, curve in self.curves.items():
            y_data = list(self.data[key])
            x_data = list(range(len(y_data)))
            curve.setData(x_data, y_data)

    def run(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        self.exec_thread = ExecutorThread(executor)
        self.exec_thread.start()

        status = self.app.exec_()
        
        self.exec_thread.quit()
        self.exec_thread.wait()
        sys.exit(status)


def main():
    rclpy.init()
    node = PGEulerPlot()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
