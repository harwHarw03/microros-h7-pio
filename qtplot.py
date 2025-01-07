#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
import sys

class PGIMUPlot(Node):
    def __init__(self):
        super().__init__('pg_imu_plot')
        self.maxlen = 500
        self.data = {k: deque(maxlen=self.maxlen) for k in ['x','y','z']}

        # Create Qt application & window
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="IMU RPY")
        self.plots = {}
        self.curves = {}
        for i, key in enumerate(['x','y','z']):
            p = self.win.addPlot(row=i, col=0, title=key.upper())
            p.setYRange(-1.5, 1.5)
            curve = p.plot(pen='y')
            self.plots[key] = p
            self.curves[key] = curve

        # Subscription
        self.create_subscription(Imu, 'imu/data', self.imu_cb, 10)

        # Update timer
        self.timer = self.win.addTimer(50, callback=self.update_plot)

    def imu_cb(self, msg):
        self.data['x'].append(msg.orientation.x)
        self.data['y'].append(msg.orientation.y)
        self.data['z'].append(msg.orientation.z)

    def update_plot(self):
        for key in ['x','y','z']:
            y = list(self.data[key])
            x = list(range(len(y)))
            self.curves[key].setData(x, y)

    def run(self):
        rclpy.spin(self)
        self.app.exec_()

def main():
    rclpy.init()
    node = PGIMUPlot()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
