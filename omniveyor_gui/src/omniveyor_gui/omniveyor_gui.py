#!/usr/bin/python

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QTreeWidgetItem

class GUI(Plugin):

    def __init__(self, context):
        super().__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        self._context = context
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('omniveyor_gui'), 'resource', 'OmniVeyorGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('OmniVeyorGUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connecting signals
        self._widget.connectButton.clicked.connect(self.connectToRobot)
        self._widget.refreshButton.clicked.connect(self.refreshConnection)
        self._widget.reconnectButton.clicked.connect(self.reconnectToRobot)
        self._widget.removeButton.clicked.connect(self.removeRobot)
        self._widget.eStopButton.clicked.connect(self.stopSelectedRobot)
        self._widget.eStopAllButton.clicked.connect(self.stopAllRobots)

    def stopRobot(self, ip):
        # TODO: stop robot at the IP
        pass

    def establishConnection(self, ip):
        # TODO: connect to robotIP with Nimbro
        pass

    def disconnectRobot(self, ip):
        # TODO: disconnect robot at the IP
        pass

    def connectToRobot(self):
        ip1 = ip2 = ip3 = ip4 = 0
        try:
            if int(self._widget.robotIP1.text()) < 0 or int(self._widget.robotIP1.text()) > 255:
                print("ERROR: IP address is ill-formed!")
            ip1 = int(self._widget.robotIP1.text())
            if int(self._widget.robotIP2.text()) < 0 or int(self._widget.robotIP2.text()) > 255:
                print("ERROR: IP address is ill-formed!")
            ip2 = int(self._widget.robotIP2.text())
            if int(self._widget.robotIP3.text()) < 0 or int(self._widget.robotIP3.text()) > 255:
                print("ERROR: IP address is ill-formed!")
            ip3 = int(self._widget.robotIP3.text())
            if int(self._widget.robotIP4.text()) < 0 or int(self._widget.robotIP4.text()) > 255:
                print("ERROR: IP address is ill-formed!")
            ip4 = int(self._widget.robotIP4.text())
        except ValueError:
            print("ERROR: IP address is ill-formed!")
        robotIP = str(ip1)+'.'+str(ip2)+'.'+str(ip3)+'.'+str(ip4)
        match = self._widget.knownRobots.findItems(robotIP, Qt.MatchExactly, column=1)
        if not len(match):
            self._widget.knownRobots.addTopLevelItem(
                QTreeWidgetItem([str(ip4), robotIP, 'Disconnected'])
            )
            match.append(
                self._widget.knownRobots.topLevelItem(
                    self._widget.knownRobots.topLevelItemCount()
                )
            )
        self.establishConnection(robotIP)

    def refreshConnection(self):
        pass

    def reconnectToRobot(self):
        for item in self._widget.knownRobots.selectedItems():
            robotIP = item.text(1)
            self.establishConnection(robotIP)

    def removeRobot(self):
        for item in self._widget.knownRobots.selectedItems():
            robotIP = item.text(1)
            self.stopRobot(robotIP)
            self.disconnectRobot(robotIP)
            self._widget.knownRobots.takeTopLevelItem(
                self._widget.knownRobots.indexOfTopLevelItem(item)
            )

    def stopSelectedRobot(self):
        for item in self._widget.knownRobots.selectedItems():
            robotIP = item.text(1)
            self.stopRobot(robotIP)

    def stopAllRobots(self):
        for ind in range(0, self._widget.knownRobots.topLevelItemCount()):
            item = self._widget.knownRobots.topLevelItem(ind)
            robotIP = item.text(1)
            self.stopRobot(robotIP)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog 
        pass
