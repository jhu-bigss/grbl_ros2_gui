import os, sys, time
import random
import numpy as np

import roslibpy

from PyQt5 import Qt, QtCore

from sksurgerynditracker.nditracker import NDITracker

rom_file_0 = os.path.abspath(os.path.join(os.path.dirname(__file__), 'digitizer-02.rom'))

SETTINGS = {
    "tracker type": "polaris",
    "romfiles" : [rom_file_0]
        }
# TRACKER = NDITracker(SETTINGS)
# TRACKER.start_tracking()
# port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()
# for t in tracking:
#   print (t)
# TRACKER.stop_tracking()
# TRACKER.close()


class MainWidget(Qt.QWidget):

  sig_shutdown = QtCore.pyqtSignal()

  def __init__(self, parent=None):
    super(MainWidget, self).__init__()

    # Setup roslibpy client connection
    self.ros = roslibpy.Ros(host='localhost', port=9090)
    self.ros.run(timeout=0.1)
    self.cmd_pub = roslibpy.Topic(self.ros, 'cmd/gcode', 'std_msgs/msg/String')

    # NDI Polaris Tracker
    self.tracker = NDITracker(SETTINGS)

    # GUI Widget
    self.setWindowTitle("Stepper Motor Calibration using NDI Polaris")

    self.axis_name_comboBox = Qt.QComboBox()
    self.axis_name_comboBox.addItems(["X", "Y", "Z", "A", "B"])
    self.axis_type_linear = Qt.QRadioButton("Linear")
    self.axis_type_linear.setChecked(True)
    self.axis_type_angular = Qt.QRadioButton("Angular")
    self.axis_limit_spinBox = Qt.QSpinBox()
    self.axis_limit_spinBox.setMaximum(1000)
    self.axis_limit_spinBox.setValue(100)
    self.num_of_measurement_spinBox = Qt.QSpinBox()
    self.num_of_measurement_spinBox.setValue(10)
    self.plot_chkBox = Qt.QCheckBox("Plot")
    self.run_btn = Qt.QPushButton("Calibrate")

    self.run_btn.clicked.connect(self.calibrate)

    # GUI Layout
    gridlayout_0 = Qt.QGridLayout()

    vlayout_axis_type = Qt.QVBoxLayout()
    vlayout_axis_type.addWidget(self.axis_type_linear)
    vlayout_axis_type.addWidget(self.axis_type_angular)

    vlayout_operation = Qt.QVBoxLayout()
    vlayout_operation.addWidget(self.plot_chkBox)
    vlayout_operation.addWidget(self.run_btn)

    gridlayout_0.addWidget(Qt.QLabel("Axis Name"), 0, 0)
    gridlayout_0.addWidget(Qt.QLabel("Axis Type"), 0, 1)
    gridlayout_0.addWidget(Qt.QLabel("Axis Limit"), 0, 2)
    gridlayout_0.addWidget(Qt.QLabel("Measurements"), 0, 3)
    gridlayout_0.addWidget(self.axis_name_comboBox, 1, 0)
    gridlayout_0.addLayout(vlayout_axis_type, 1, 1)
    gridlayout_0.addWidget(self.axis_limit_spinBox, 1, 2)
    gridlayout_0.addWidget(self.num_of_measurement_spinBox, 1, 3)
    gridlayout_0.addLayout(vlayout_operation, 1, 4)

    self.setLayout(gridlayout_0)
    self.show()


  def calibrate(self):
    # Start tracking
    self.tracker.start_tracking()
    print("Tracking Started")
    pose_measure_list = []
    distance_measure_list = []
    time.sleep(1)
    pose_measure_list.append(self.tracker.get_frame()[3][0])
    
    # Generate random gcode cmd list for single axis
    jnt_val_list = random.sample(range(0, self.axis_limit_spinBox.value()), self.num_of_measurement_spinBox.value())
    jnt_val_list.sort()
    axis_name = self.axis_name_comboBox.currentText()
    gcode = "G0{}".format(axis_name)
    gcode_send_list = []
    for val in jnt_val_list:
      gcode_send_list.append(gcode + "{:0.3f}".format(-val))

    # Send Gcode to Grbl_backend through roslibpy
    if self.ros.is_connected:
      for cmd in gcode_send_list:
        self.cmd_pub.publish(roslibpy.Message({'data': cmd}))
        time.sleep(1)
        port_handles, timestamps, framenumbers, tracking, quality = self.tracker.get_frame()
        pose_measure_list.append(tracking[0])
      self.cmd_pub.unadvertise()

    # Stop tracking
    self.tracker.stop_tracking()
    print("Tracking Stopped")
    initial_pose = pose_measure_list.pop(0)

    # Process the measurement
    if self.axis_type_linear.isChecked():
      for pose in pose_measure_list:
        distance_measure = np.linalg.norm(pose[:3,3] - initial_pose[:3,3])
        distance_measure_list.append(distance_measure)

    # np.savetxt("command.csv", jnt_val_list, delimiter = ',', fmt='%10.3f')
    # np.savetxt("measurement.csv", distance_measure_list, delimiter = ',', fmt='%10.3f')
    # print("File Saved")


  def closeEvent(self, event):
    # disconnect roslibpy client and close the tracker
    self.ros.terminate()
    self.tracker.close()
    super().closeEvent(event)


if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())