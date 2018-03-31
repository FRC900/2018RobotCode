import os
import rospy
import rospkg
import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from ros_control_boilerplate.msg import AutoMode
from ros_control_boilerplate.msg import MatchSpecificData


class DriverStationSim(Plugin):

    def __init__(self, context):
        super(DriverStationSim, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DriverStationSim')

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
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_driver_station_sim'), 'resource', 'driverStationSim.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DriverStationSim')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        auto_pub = rospy.Publisher("/frcrobot/autonomous_mode", AutoMode, queue_size=3)
        match_pub = rospy.Publisher("/frcrobot/match_data", MatchSpecificData, queue_size=3)

        def pub_data(self):
            r = rospy.Rate(10)
            auto_msg = AutoMode()
            auto_msg.mode = [0, 0, 0, 0]
            auto_msg.delays = [0, 0, 0, 0]
            match_msg = MatchSpecificData()

            modes =  [0, 0, 0, 0]
            match_msg = MatchSpecificData()
            start_time = rospy.get_time()
            enable_last = False
            auto_last = False
            practice_last = False
            auto_duration = 0
            while(not rospy.is_shutdown()):
                #Robot State Values
                enable = self._widget.enable_button_2.isChecked()
                disable = self._widget.disable_button_2.isChecked()
                auto = self._widget.auto_mode.isChecked()
                practice = self._widget.practice_button.isChecked()


                #Time Start and Restart Handling
                if(not enable_last and enable):
                    rospy.logwarn("enableLast")
                    start_time = rospy.get_time()
                if(not auto_last and auto and not practice):
                    rospy.logwarn("autoLast")
                    start_time = rospy.get_time()
                if(not practice_last and practice):
                    rospy.logwarn("practiceLast")
                    start_time = rospy.get_time()
                    auto_duration = 15 #TODO read from DS
                if(enable and practice):
                    if(rospy.get_time() < start_time + auto_duration):
                        auto = True
                        enable = True
                        disable = False
                    elif(rospy.get_time() >= start_time + auto_duration and rospy.get_time < start_time + 150):
                        auto = False
                        enable = True
                        disable = False
                    elif(rospy.get_time() >= start_time + 150):
                        auto = False
                        enable = False
                        disable = True


                if(enable):
                    time_diff = int(rospy.get_time()-start_time)
                    self._widget.minutes.display((150-time_diff)/60)
                    self._widget.seconds.display((150-time_diff)%60)
                    match_msg.matchTimeRemaining = 150-time_diff
                    match_msg.isDisabled = False
                    match_msg.isEnabled = True
                else:
                    match_msg.matchTimeRemaining = 0
                    match_msg.isDisabled = True
                    match_msg.isEnabled = False

                #Publish Data
                match_msg.allianceData = self._widget.match_data.text()
                match_msg.allianceColor = 1
                match_msg.driverStationLocation = 1
                match_msg.matchNumber = 1
                match_msg.isAutonomous = auto

                enable_last = match_msg.isEnabled
                auto_last = auto
                practice_last = practice


                auto_msg.header.stamp = rospy.Time.now()

                auto_msg.mode[0] = self._widget.mode_0.value();
                auto_msg.mode[1] = self._widget.mode_1.value();
                auto_msg.mode[2] = self._widget.mode_2.value();
                auto_msg.mode[3] = self._widget.mode_3.value();
                
                auto_msg.delays[0] = self._widget.delay_0.value();
                auto_msg.delays[1] = self._widget.delay_1.value();
                auto_msg.delays[2] = self._widget.delay_2.value();
                auto_msg.delays[3] = self._widget.delay_3.value();
                
                match_pub.publish(match_msg)
                auto_pub.publish(auto_msg)
                r.sleep()
                
        load_thread = threading.Thread(target=pub_data, args=(self,))
        load_thread.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
