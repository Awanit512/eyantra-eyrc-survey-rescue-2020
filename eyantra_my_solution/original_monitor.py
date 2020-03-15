# Python bytecode 2.7 (62211)
# Embedded file name: monitor.py
# Compiled at: 2020-02-20 18:20:48
# Decompiled by https://python-decompiler.com
import rospy, roslib, rospkg
from std_msgs.msg import Int8, String, Bool
import math, time, csv, threading
from geometry_msgs.msg import PoseArray
import sys, re, operator, ast, json, string, signal, os, Queue as queue
from survey_and_rescue.msg import *

class Led_Control():

    def __init__(self):
        self.whycon_marker = None
        self.current_setpoint = None
        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_data)
        rospy.Subscriber('/decision_info', SRInfo, self.map_setpoint_to_LED)
        rospy.Subscriber('/detection_info', SRInfo, self.verify_LED)
        rospy.Subscriber('/arm_status', Bool, self.sub_arm_status)
        self.stats_pub = rospy.Publisher('/stats_sr', SRDroneStats, 6=10)
        self.serviced_pub = rospy.Publisher('/serviced_info', SRInfo, 6=8)
        self.led_command_pub = rospy.Publisher('/led_control_topic', Int8, 6=8)
        self.base_cell_int_pub = rospy.Publisher('/base_cell_int_topic', Int8, 6=1)
        self.arm_status = False
        self.serviced_message = SRInfo()
        self.number_of_nodes = 8
        self.thresh_val = [
         0.93, 0.93, 1.5]
        self.init_base_thresh_val = [1, 1, 5]
        self.first_inside = False
        self.waypoint_dict = {}
        self.timer_object_list = []
        self.timer_object_dict = {}
        self.timer_object_dict_timeout = {}
        self.total_time_cont = time.time()
        self.prev_time = time.time()
        self.init_time = time.time()
        self.score_params_weights_dict = {'FD': 10, 'MD': 10, 'R': 50, 'CD': 5, 'ID': -5, 'RD': -5, 'IS': -10}
        self.duration_inside_cumul = 0
        self.last_outside_time_in_cumul = time.time()
        self.time_inside_cumul = 0
        self.current_threshold_seconds = 3
        self.current_led_module = None
        self.prev_dest_cell = None
        self.cell_name_list = []
        self.led_seq_list = []
        self.pose_list = []
        self.cell_name_in_timing_list = []
        self.led_service_list_names = []
        self.led_module_status = [
         'OFF'] * self.number_of_nodes
        self.led_serial_code_dict = {'OFF': 0, 'FOOD': 2, 'MEDICINE': 1, 'RESCUE': 3}
        self.payload = {'FOOD': 3, 'MEDICINE': 3}
        self.led_timeout_dict = {'OFF': 0, 'FOOD': 30, 'MEDICINE': 30, 'RESCUE': 10}
        self.led_hover_time_dict = {'FOOD': 3, 'MEDICINE': 3, 'RESCUE': 5, 'BASE': 5}
        self.led_time_delay_list = []
        self.base_cell = None
        self.base_cell_num = 0
        self.current_dest_cell = None
        self.current_lit_dict = {'FOOD': [], 'MEDICINE': [], 'RESCUE': []}
        self.current_detected_dict = {'FOOD': [], 'MEDICINE': [], 'RESCUE': []}
        self.stats = SRDroneStats()
        self.stats.foodOnboard = self.payload['FOOD']
        self.stats.medOnboard = self.payload['MEDICINE']
        self.patient_onboard = False
        self.serial_queue = queue.Queue()
        self.hover_timer = None
        self.stats_timer = None
        self.serial_timer = None
        self.kill_timer = None
        return

    def sub_arm_status(self, msg):
        self.arm_status = msg.data

    def whycon_data(self, msg):
        pos_x = round(msg.poses[0].position.x, 3)
        pos_y = round(msg.poses[0].position.y, 3)
        pos_z = round(msg.poses[0].position.z, 3)
        self.whycon_marker = [pos_x, pos_y, pos_z]

    def clear_stats_decision_events(self):
        setattr(self.stats.decisionEvents, 'FOOD', [])
        setattr(self.stats.decisionEvents, 'MEDICINE', [])
        setattr(self.stats.decisionEvents, 'RESCUE', [])

    def verify_LED(self, msg):
        cell_input_res = re.findall('^[A-F][1-6]$', msg.location)
        if cell_input_res:
            current_coord = msg.location
            if current_coord not in self.cell_name_list:############it cant be in my code since i m detecting only in those roi in which led is kept
                current_coord = None
                print '\nDetection Incorrect. No Beacon placed on that location.\n'
                self.stats.incorrectDetections = min(255, self.stats.incorrectDetections + 1)
                self.stats.score = self.stats.score + self.score_params_weights_dict['ID']
            if current_coord is not None:
                status = string.upper(msg.info)
                led_module = self.led_seq_list[self.cell_name_list.index(current_coord)]
                if status != self.led_module_status[led_module]:
                    print '\nDetection Info recieved: ' + current_coord + ': ' + status + ' Actual status is: ' + self.led_module_status[led_module] + '\n'
                    self.stats.incorrectDetections = min(255, self.stats.incorrectDetections + 1)
                    self.stats.score = self.stats.score + self.score_params_weights_dict['ID']
                elif self.find_whether_in_current_det_dict(status, current_coord):
                    self.stats.redundantDetections = min(255, self.stats.redundantDetections + 1)
                    self.stats.score = self.stats.score + self.score_params_weights_dict['RD']
                else:
                    self.set_current_det_dict(status, current_coord)
                    self.stats.correctDetections = min(255, self.stats.correctDetections + 1)
                    self.stats.score = self.stats.score + self.score_params_weights_dict['CD']
        else:
            print '\nPlease publish valid data\n'
        return

    def publish_serviced_info_for_current_dest_cell(self, info):
        self.serviced_message.location = self.current_dest_cell
        self.serviced_message.info = info
        self.serviced_pub.publish(self.serviced_message)

    def map_setpoint_to_LED(self, msg):
        cell_input_res = re.findall('^[A-F][1-6]$', msg.location)
        if cell_input_res:
            self.current_dest_cell = msg.location
            if self.current_dest_cell not in self.cell_name_list:
                if self.current_dest_cell != self.base_cell:
                    self.current_dest_cell = None
                    print '\nDecision' + 'Incorrect. No Beacon placed on that location, or is not BASE\n'
                    self.stats.incorrectServices = min(255, self.stats.incorrectServices + 1)
                    self.stats.score = self.stats.score + self.score_params_weights_dict['IS']
                    self.publish_serviced_info_for_current_dest_cell('FAILURE')
            if self.current_dest_cell is not None:
                status = string.upper(msg.info)
                try:
                    self.current_threshold_seconds = self.led_hover_time_dict[status]
                except:
                    print '\nInvalid Status Published. Marking Incorrect.\n'
                    self.stats.incorrectServices = min(255, self.stats.incorrectServices + 1)
                    self.stats.score = self.stats.score + self.score_params_weights_dict['IS']

                self.clear_stats_decision_events()
                if status == 'BASE':
                    self.current_setpoint = self.waypoint_dict[self.current_dest_cell]
                    self.current_led_module = None
                else:
                    self.current_led_module = self.led_seq_list[self.cell_name_list.index(self.current_dest_cell)]
                    if status != self.led_module_status[self.current_led_module]:
                        print '\nDecision Info recieved: ' + self.current_dest_cell + ': ' + status + ' Actual status is: ' + self.led_module_status[self.current_led_module] + '\n'
                        self.stats.incorrectServices = min(255, self.stats.incorrectServices + 1)
                        self.stats.score = self.stats.score + self.score_params_weights_dict['IS']
                        self.reset_current_variables_to_none()
                    elif status != 'RESCUE' and self.payload[status] == 0:
                        print '\nYou do not have enough supplies, return to base before servicing this Beacon.\n'
                        self.stats.incorrectServices = min(255, self.stats.incorrectServices + 1)
                        self.stats.score = self.stats.score + self.score_params_weights_dict['IS']
                        self.reset_current_variables_to_none()
                    else:
                        self.current_setpoint = self.waypoint_dict[self.current_dest_cell]
                        setattr(self.stats.decisionEvents, status, [(ord(self.current_dest_cell[0]) - 64) * 10 + int(self.current_dest_cell[1])])
        else:
            print '\nPlease publish valid data\n'
        return

    def query_yes_no(self, question, default=None):
        """Ask a yes/no question via raw_input() and return their answer.

                "question" is a string that is presented to the user.
                "default" is the presumed answer if the user just hits <Enter>.
                It must be "yes" (the default), "no" or None (meaning
                an answer is required of the user).

                The "answer" return value is True for "yes" or False for "no".
                """
        valid = {'yes': True, 'y': True, 'ye': True, 'no': False, 'n': False}
        if default is None:
            prompt = ' [Y/N] '
        elif default == 'yes':
            prompt = ' [Y/N] '
        elif default == 'no':
            prompt = ' [Y/N] '
        else:
            raise ValueError("invalid default answer: '%s'" % default)
        while True:
            sys.stdout.write(question + prompt)
            choice = raw_input().lower()
            if default is not None and choice == '':
                return valid[default]
            if choice in valid:
                return valid[choice]
            sys.stdout.write("Please respond with 'yes' or 'no' (or 'y' or 'n').\n")

        return

    def input_position_config_to_tsv(self):
        for i in range(self.number_of_nodes + 1):
            while True:
                cell_res = raw_input('Location on grid of module ' + str(i) + ': ').upper()
                cell_input_res = re.findall('^[A-F][1-6]$', cell_res)
                if cell_input_res:
                    if cell_res in self.cell_name_list:
                        print "You've already entered the location of this module."
                        continue
                    else:
                        self.cell_name_list.append(cell_input_res[0])
                        cell_input_res = []
                        break
                else:
                    print 'Please enter a valid input'

            self.led_seq_list.append(i)

    def write_tsv_config(self, file_path='LED_Config.tsv'):
        with open(file_path, 1='wb') as (outfile):
            config_writer = csv.writer(outfile, 3='\t', 5=csv.QUOTE_MINIMAL)
            if file_path == 'LED_Config.tsv':
                for i in range(self.number_of_nodes):
                    config_writer.writerow([self.cell_name_list[i], self.led_seq_list[i]])

    def clear_lists(self, compare_strings):
        if compare_strings == 'config_lists':
            self.cell_name_list = []
            self.led_seq_list = []
            self.pose_list = []
        elif compare_strings == 'timing_lists':
            self.cell_name_in_timing_list = []
            self.led_service_list_names = []
            self.led_time_delay_list = []

    def load_cell_config_json(self, file_path=os.path.expanduser('~/catkin_ws/src/survey_and_rescue/scripts/cell_coords.json')):
        with open(file_path, 1='r') as (infile):
            self.waypoint_dict = json.load(infile)

    def read_tsv_config(self, file_path=os.path.expanduser('~/catkin_ws/src/survey_and_rescue/scripts/LED_Config.tsv')):
        with open(file_path, 1='r') as (infile):
            reader = csv.reader(infile, 3='\t')
            if os.path.split(file_path)[1] == 'LED_Config.tsv':
                for row in reader:
                    if all(x.strip() for x in row):
                        row[1] = string.upper(row[1]).strip()
                        row[0] = string.upper(row[0]).strip()
                        cell_input_res = re.findall('^[A-F][1-6]$', row[0])
                        if cell_input_res:
                            if row[1] != 'BASE':
                                self.cell_name_list.append(row[0])
                                self.led_seq_list.append(int(row[1]) - 1)
                                self.timer_object_dict[row[0]] = []
                                self.timer_object_dict_timeout[row[0]] = []
                            else:
                                self.base_cell = row[0]
                        else:
                            raise EnvironmentError('Coordinate incorrect.')

                for i in range(len(self.cell_name_list)):
                    print 'LED no. ' + str(self.led_seq_list[i] + 1) + ' is at ' + self.cell_name_list[i] + (' with WhyCon coordinate of: {}').format(self.waypoint_dict[self.cell_name_list[i]])

            elif os.path.split(file_path)[1] == 'LED_Timing.tsv':
                for row in reader:
                    if all(x.strip() for x in row):
                        self.cell_name_in_timing_list.append(string.upper(row[0]).strip())
                        self.led_service_list_names.append(string.upper(row[1]).strip())
                        self.led_time_delay_list.append(string.upper(row[2]).strip())

    def payload_manager(self):
        if self.current_led_module is not None:
            status = self.led_module_status[self.current_led_module]
        else:
            status = 'Invalid'
        if status == 'FOOD':
            self.payload['FOOD'] = self.payload['FOOD'] - 1
        elif status == 'MEDICINE':
            self.payload['MEDICINE'] = self.payload['MEDICINE'] - 1
        elif status == 'RESCUE':
            self.payload['FOOD'] = 0
            self.payload['MEDICINE'] = 0
        return

    def payload_top_up(self):
        self.payload['FOOD'] = 3
        self.payload['MEDICINE'] = 3

    def stats_update_on_beacon(self):
        self.stats.foodOnboard = self.payload['FOOD']
        self.stats.medOnboard = self.payload['MEDICINE']
        if self.led_module_status[self.current_led_module] == 'FOOD':
            self.stats.foodDistributed = self.stats.foodDistributed + 1
            self.stats.correctServices = self.stats.correctServices + 1
            self.stats.score = self.stats.score + self.score_params_weights_dict['FD']
        elif self.led_module_status[self.current_led_module] == 'MEDICINE':
            self.stats.medDistributed = self.stats.medDistributed + 1
            self.stats.correctServices = self.stats.correctServices + 1
            self.stats.score = self.stats.score + self.score_params_weights_dict['MD']
        elif self.led_module_status[self.current_led_module] == 'RESCUE':
            self.patient_onboard = True

    def stats_update_on_base(self):
        self.stats.foodOnboard = self.payload['FOOD']
        self.stats.medOnboard = self.payload['MEDICINE']
        if self.patient_onboard == True:
            self.patient_onboard = False
            self.stats.correctServices = self.stats.correctServices + 1
            self.stats.score = self.stats.score + self.score_params_weights_dict['R']
            self.stats.rescues = self.stats.rescues + 1

    def check_hover_continuous(self, event=None):
        if self.current_setpoint is not None and self.whycon_marker is not None:
            if all(map(operator.le, map(operator.abs, map(operator.sub, self.whycon_marker, self.current_setpoint)), self.thresh_val)):
                if self.first_inside:
                    self.init_time = time.time()
                    self.first_inside = False
                self.total_time_cont = time.time() - self.init_time
                self.prev_time = time.time()
                if self.total_time_cont > self.current_threshold_seconds and self.current_setpoint is not None:
                    self.payload_manager()
                    self.stats_update_on_beacon()
                    self.publish_led_values(self.current_led_module, 'OFF', self.current_dest_cell)
                    self.serviced_message.location = self.current_dest_cell
                    self.serviced_message.info = 'Success'
                    self.serviced_pub.publish(self.serviced_message)
                    self.first_inside = True
                    self.publish_serviced_info_for_current_dest_cell('SUCCESS')
                    self.reset_current_variables_to_none()
            else:
                self.first_inside = True
                self.total_time_cont = 0
        else:
            self.first_inside = True
            sys.stdout.flush()
        return

    def check_hover_continuous_print(self, event=None):
        if self.current_setpoint is not None and self.whycon_marker is not None:
            if all(map(operator.le, map(operator.abs, map(operator.sub, self.whycon_marker, self.current_setpoint)), self.thresh_val)):
                if self.first_inside:
                    self.init_time = time.time()
                    self.first_inside = False
                self.total_time_cont = time.time() - self.init_time
                self.prev_time = time.time()
                if self.total_time_cont > self.current_threshold_seconds and self.current_setpoint is not None and self.current_led_module is not None:
                    self.payload_manager()
                    self.stats_update_on_beacon()
                    self.publish_led_values(self.current_led_module, 'OFF', self.current_dest_cell)
                    self.first_inside = True
                    self.publish_serviced_info_for_current_dest_cell('SUCCESS')
                    self.reset_current_variables_to_none()
            else:
                self.first_inside = True
                self.total_time_cont = 0
            if self.current_dest_cell is not None:
                sys.stdout.write('\rHovering at ' + self.current_dest_cell + ' for: ' + ('{0:03.1f}').format(self.total_time_cont) + ' seconds')
                sys.stdout.flush()
        else:
            self.first_inside = True
            sys.stdout.flush()
        return

    def check_near_base(self):
        if self.whycon_marker is not None:
            base_setpoint = self.waypoint_dict[self.base_cell]
            if all(map(operator.le, map(operator.abs, map(operator.sub, self.whycon_marker, base_setpoint)), self.thresh_val)):
                return True
        return False

    def whether_destination_changed(self):
        if self.prev_dest_cell != self.current_dest_cell:
            self.prev_dest_cell = self.current_dest_cell
            return True
        else:
            return False

    def check_hover_cumulative_print(self, event=None):
        if self.current_setpoint is not None and self.whycon_marker is not None:
            if self.whether_destination_changed():
                self.duration_inside_cumul = 0
                self.time_inside_cumul = 0
                self.last_outside_time_in_cumul = time.time()
            if all(map(operator.le, map(operator.abs, map(operator.sub, self.whycon_marker, self.current_setpoint)), self.thresh_val)):
                curr_time_cumul = time.time()
                self.time_inside_cumul = self.duration_inside_cumul + (curr_time_cumul - self.last_outside_time_in_cumul)
                if self.time_inside_cumul > self.current_threshold_seconds and self.current_setpoint is not None:
                    self.time_inside_cumul = 0
                    if self.base_cell == self.current_dest_cell:
                        self.payload_top_up()
                        self.stats_update_on_base()
                    else:
                        for t in self.timer_object_dict_timeout[self.current_dest_cell]:
                            if t.isAlive():
                                t.cancel()

                        self.payload_manager()
                        self.stats_update_on_beacon()
                        self.publish_led_values(self.current_led_module, 'OFF', self.current_dest_cell)
                    self.publish_serviced_info_for_current_dest_cell('SUCCESS')
                    self.reset_current_variables_to_none()
            else:
                self.duration_inside_cumul = self.time_inside_cumul
                self.last_outside_time_in_cumul = time.time()
            if self.current_dest_cell is not None:
                sys.stdout.write('\rHovering at ' + self.current_dest_cell + ' for: ' + ('{0:03.1f}').format(self.time_inside_cumul) + ' seconds')
                sys.stdout.flush()
        else:
            self.last_outside_time_in_cumul = time.time()
        return

    def check_hover_cumulative(self, event=None):
        if self.current_setpoint is not None and self.whycon_marker is not None:
            if self.whether_destination_changed():
                self.duration_inside_cumul = 0
                self.time_inside_cumul = 0
                self.last_outside_time_in_cumul = time.time()
            if all(map(operator.le, map(operator.abs, map(operator.sub, self.whycon_marker, self.current_setpoint)), self.thresh_val)):
                curr_time_cumul = time.time()
                self.time_inside_cumul = self.duration_inside_cumul + (curr_time_cumul - self.last_outside_time_in_cumul)
                if self.time_inside_cumul > self.current_threshold_seconds and self.current_setpoint is not None:
                    self.time_inside_cumul = 0
                    if self.base_cell == self.current_dest_cell:
                        self.payload_top_up()
                        self.stats_update_on_base()
                    else:
                        for t in self.timer_object_dict_timeout[self.current_dest_cell]:
                            if t.isAlive():
                                t.cancel()

                        self.payload_manager()
                        self.stats_update_on_beacon()
                        self.publish_led_values(self.current_led_module, 'OFF', self.current_dest_cell)
                    self.publish_serviced_info_for_current_dest_cell('SUCCESS')
                    self.reset_current_variables_to_none()
            else:
                self.duration_inside_cumul = self.time_inside_cumul
                self.last_outside_time_in_cumul = time.time()
        else:
            self.last_outside_time_in_cumul = time.time()
        return

    def set_current_det_dict(self, service_name, cell_name):
        string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
        self.current_detected_dict[service_name].append(string_repr_to_num)
        setattr(self.stats.currentDetected, service_name, self.current_detected_dict[service_name])

    def del_current_det_dict(self, service_name, cell_name):
        if service_name != 'OFF':
            string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
            if string_repr_to_num in self.current_detected_dict[service_name]:
                self.current_detected_dict[service_name].remove(string_repr_to_num)
                setattr(self.stats.currentDetected, service_name, self.current_detected_dict[service_name])

    def find_whether_in_current_det_dict(self, service_name, cell_name):
        string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
        if string_repr_to_num in self.current_detected_dict[service_name]:
            return True
        else:
            return False

    def find_whether_in_current_lit_dict(self, service_name, cell_name):
        string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
        if string_repr_to_num in self.current_lit_dict[service_name]:
            return True
        else:
            return False

    def set_current_lit(self, service_name, cell_name):
        string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
        self.current_lit_dict[service_name].append(string_repr_to_num)
        setattr(self.stats.currentLit, service_name, self.current_lit_dict[service_name])

    def del_from_curr_lit_dict(self, service_name, cell_name):
        if service_name != 'OFF':
            string_repr_to_num = (ord(cell_name[0]) - 64) * 10 + int(cell_name[1])
            self.current_lit_dict[service_name].remove(string_repr_to_num)
            setattr(self.stats.currentLit, service_name, self.current_lit_dict[service_name])

    def timeout_publish_led(self, led_module_no, service_name, cell_name):
        if self.cell_name_list[self.led_seq_list[led_module_no]] == self.current_dest_cell:
            self.reset_current_variables_to_none()
        self.stats.failedServices = self.stats.failedServices + 1
        self.serial_queue.put(led_module_no * 16)
        self.led_module_status[led_module_no] = 'OFF'
        self.serviced_message.location = self.cell_name_list[self.led_seq_list[led_module_no]]
        self.serviced_message.info = 'FAILURE'
        self.serviced_pub.publish(self.serviced_message)
        if self.find_whether_in_current_lit_dict(service_name, cell_name):
            self.del_from_curr_lit_dict(service_name, cell_name)
        if self.find_whether_in_current_det_dict(service_name, cell_name):
            self.del_current_det_dict(service_name, cell_name)

    def publish_led_values(self, led_module_no, service_name, cell_name):
        service_number = self.led_serial_code_dict[service_name]
        numeric_code = led_module_no * 16 + service_number
        prev_led_status = self.led_module_status[led_module_no]
        self.serial_queue.put(numeric_code)
        self.led_module_status[led_module_no] = service_name
        if service_name != 'OFF':
            self.set_current_lit(service_name, cell_name)
            t = threading.Timer(int(self.led_timeout_dict[service_name]), self.timeout_publish_led, [self.led_seq_list[led_module_no], service_name, cell_name])
            self.timer_object_list.append(t)
            self.timer_object_dict_timeout[cell_name].append(t)
            t.start()
        elif prev_led_status != 'OFF':
            if self.find_whether_in_current_lit_dict(prev_led_status, cell_name):
                self.del_from_curr_lit_dict(prev_led_status, cell_name)
            if self.find_whether_in_current_det_dict(prev_led_status, cell_name):
                self.del_current_det_dict(prev_led_status, cell_name)

    def light_leds_in_sequence(self):
        for idx, cell_name in enumerate(self.cell_name_in_timing_list):
            try:
                led_module_index = self.cell_name_list.index(self.cell_name_in_timing_list[idx])
                t = threading.Timer(int(self.led_time_delay_list[idx]), self.publish_led_values, [self.led_seq_list[led_module_index], self.led_service_list_names[idx], cell_name])
                self.timer_object_list.append(t)
                self.timer_object_dict[cell_name].append(t)
                t.start()
            except ValueError:
                print 'Module not mapped'
                continue

    def reset_leds(self):
        if self.cell_name_list is not None:
            for idx, cell_name in enumerate(self.cell_name_list):
                self.publish_led_values(idx, 'OFF', cell_name)

        return

    def reset_current_variables_to_none(self):
        self.current_dest_cell = None
        self.current_led_module = None
        self.current_setpoint = None
        return

    def publish_stats(self, event=None):
        self.stats_pub.publish(self.stats)

    def publish_serial(self, event=None):
        if self.serial_queue.qsize() > 0:
            data_from_queue = self.serial_queue.get()
            self.led_command_pub.publish(data_from_queue)

    def determine_base_num(self):
        self.base_cell_num = (ord(self.base_cell[0]) - 64) * 10 + int(self.base_cell[1])

    def publish_base_num_and_stats(self, event=None):
        self.stats_pub.publish(self.stats)
        self.base_cell_int_pub.publish(self.base_cell_num)

    def kill_sequence(self):
        if self.serial_timer is not None:
            if self.serial_timer.isAlive():
                led_control.reset_leds()
                while led_control.serial_queue.qsize() > 0:
                    pass

                self.hover_timer.shutdown()
                self.stats_timer.shutdown()
                for t in self.timer_object_list:
                    if t.isAlive():
                        t.cancel()

                self.serial_timer.shutdown()
                self.kill_timer.shutdown()
        return

    def kill_node_event(self, event=None):
        os.kill(os.getpid(), signal.SIGINT)

    def hook(self):
        self.kill_sequence()
        print '\nShutting down monitor.pyc'


if __name__ == '__main__':
    print 'monitor.pyc ver. 1.5.0'
    rospy.init_node('monitor', 10=False, 11=False)
    try:
        pkg_name_param = rospy.get_param('~package_name', 'survey_and_rescue')
        print_flag_param = rospy.get_param('~print_time', True)
        path_to_subfolder_param = rospy.get_param('~path_rel_to_pkg', 'scripts')
        stats_rate_param = rospy.get_param('~stats_topic_rate', 15)
        cont_hover_param = rospy.get_param('~continuous_hovering', True)
        stats_rate_param = min(30, max(1, stats_rate_param))
        stats_rate = 1 / float(stats_rate_param)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name_param)
        path = os.path.join(pkg_path, path_to_subfolder_param)
        led_control = Led_Control()
        rospy.on_shutdown(led_control.hook)
        if cont_hover_param:
            print 'Continuous Hovering Mode is ON. Give the appropriate paramters to change mode.'
            if print_flag_param:
                check_hover = led_control.check_hover_continuous_print
            else:
                check_hover = led_control.check_hover_continuous
        else:
            print 'Continuous Hovering Mode is OFF.'
            if print_flag_param:
                check_hover = led_control.check_hover_cumulative_print
            else:
                check_hover = led_control.check_hover_cumulative
        led_control.number_of_nodes = rospy.get_param('~num_beacons', 8)
        led_control.read_tsv_config(os.path.join(path, 'LED_Timing.tsv'))
        led_control.load_cell_config_json(os.path.join(path, 'cell_coords.json'))
        load_flag = True
        if load_flag:
            led_control.read_tsv_config(os.path.join(path, 'LED_Config.tsv'))
        else:
            led_control.input_position_config_to_tsv()
            led_control.write_tsv_config()
        led_control.determine_base_num()
        led_control.reset_leds()
        led_control.hover_timer = rospy.Timer(rospy.Duration(0.03), check_hover)
        led_control.stats_timer = rospy.Timer(rospy.Duration(stats_rate), led_control.publish_base_num_and_stats)
        led_control.serial_timer = rospy.Timer(rospy.Duration(0.02), led_control.publish_serial)
        led_control.kill_timer = rospy.Timer(rospy.Duration(180), led_control.kill_node_event, 32=True)
        start_on_base_param = rospy.get_param('~start_on_base', False)
        start_with_countdown_param = rospy.get_param('~start_with_countdown', True)
        countdown_param = rospy.get_param('~countdown', 3)
        if start_on_base_param:
            print 'Survey and Rescue Mission Approved.\nCleared for take-off from Base Station.'
            while led_control.check_near_base() == False:
                pass

        elif start_with_countdown_param:
            for i in range(countdown_param):
                print 'Beacon sequence starting in: ' + str(countdown_param - i) + ' seconds.'
                time.sleep(1)

        else:
            start_flag = False
            while start_flag == False:
                start_flag = led_control.query_yes_no('Enter Y/y to Start LED_Sequence')

            print 'Starting sequence now.\n'
        led_control.light_leds_in_sequence()
        while not rospy.is_shutdown():
            rospy.spin()

    except (KeyboardInterrupt, EnvironmentError, Exception) as e:
        if e is KeyboardInterrupt:
            print 'Keyboard Interrupt recieved.'
        elif e is EnvironmentError:
            print 'Make sure that Config files exist and/or are correct'
        else:
            print e
            print '\nException raised. Check parameters.'