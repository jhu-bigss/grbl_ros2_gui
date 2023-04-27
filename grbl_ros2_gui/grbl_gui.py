##############################################################################
# Documentation
##############################################################################
"""
A pyqt-based application panel for interfacing with grbl devices
"""
##############################################################################
# Imports
##############################################################################

import sys, os
from threading import Thread

from PyQt5 import QtCore, QtWidgets

cn5X_module_path = os.path.join(os.path.dirname(__file__), 'cn5X')
sys.path.append(cn5X_module_path)

from grbl_ros2_gui.cn5X.cn5X import *
from grbl_ros2_gui.ros_backend import Backend

from grbl_ros2_gui.toolpath.rectangle_zigzag import RectangleZigzagPath

import rclpy


def main(args=None):
    # ROS init
    rclpy.init(args=args)

    # GUI
    app = QtWidgets.QApplication(sys.argv)

    if getattr(sys, 'frozen', False):
        # frozen
        app_path = os.path.dirname(sys.executable)
    else:
        # unfrozen
        app_path = os.path.dirname(os.path.realpath(__file__))

    app_path = os.path.join(app_path, 'cn5X')

    print("{} v{} running from: {}".format(APP_NAME, APP_VERSION_STRING, app_path))
    print("")
    print("                ####### #     #")
    print("  ####   #    # #        #   #     #       #")
    print(" #    #  ##   # #         # #      #       #")
    print(" #       # #  #  #####     #     #####   #####")
    print(" #       #  # #       #   # #      #       #")
    print(" #    #  #   ## #     #  #   #     #       #")
    print("  ####   #    #  #####  #     #")
    print("")

    translator = QtCore.QTranslator()
    langue = QtCore.QLocale(QtCore.QLocale.French, QtCore.QLocale.France)
    translator.load(langue, "{}/i18n/cn5X".format(app_path), ".")
    app.installTranslator(translator)

    # Définition de la locale pour affichage des dates dans la langue du systeme
    locale.setlocale(locale.LC_TIME, '')

    # Set up global variables in the module
    sys.modules['grbl_ros2_gui.cn5X.cn5X'].app = app
    sys.modules['grbl_ros2_gui.cn5X.cn5X'].app_path = app_path
    sys.modules['grbl_ros2_gui.cn5X.cn5X'].translator = translator

    backend = Backend()
    window = winMain()

    # Connect GUI main_window signals to ROS backend
    window._winMain__grblCom.sig_status.connect(backend.update_status)
    window.ui.dsbJogSpeed.valueChanged.connect(backend.update_jogSpeed)
    window.destroyed.connect(backend.terminate_ros_backend)
    backend.sig_push_gcode.connect(window._winMain__grblCom.gcodePush)

    # Qt/ROS bringup
    ros_thread = Thread(target=backend.spin)
    ros_thread.start()
    window.show()
    result = app.exec_()

    # Shutdown
    backend.node.get_logger().info("{0} terminated".format(backend.node.get_name()))
    ros_thread.join()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()
