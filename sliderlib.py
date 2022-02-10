
# TODO measure travel distance
# TODO check move slider/test slider to make sure forward/backward mapping properly

"""
This library contains an API for interacting with an Arduino microcontroller loaded with firmware to move a rotating
slider.
"""

import serial
import serial.tools.list_ports
import datetime
import time
import os
import pickle
import struct
import pandas as pd
import warnings
import statistics

BAUD_RATE = 115200
STEPS_PER_REVOLUTION = 400
FORWARD = 1
REVERSE = 0
DIR_STR = {1: 'f', 0: 'b'}
KEY_LIST = ['start_time', 'end_time', 'stepper_ang', 'cmd_speed', 'direction', 'result', 'notes']


class SliderController(object):
    def __init__(self, mapping=None, angle_order=None, dest_folder=None,
                 animal_name=None, exp_tag=None, cell_id=None, title_string=None):
        """
        create slider object, connect to Arduino

        :param mapping: mapping between slider angle and desired speed
        :param angle_order: order to iterate through angles
        :param dest_folder: folder to store generated data files
        :param animal_name: identifier for animal
        :param exp_tag: tag to identify experiment (eg 'slow')
        :param title_string: custom title string to replace default naming convention of name_tag_dateTime
        """

        # initialize variables
        self.mapping = mapping
        self.angle_order = angle_order
        self.date_time_str = get_datetime_str()

        if not dest_folder:
            dest_folder = os.getcwd()
        self.dest_folder = dest_folder
        try:
            assert(os.path.isdir(self.dest_folder))
        except:
            raise IsADirectoryError('destination directory is not valid:\t' + self.dest_folder)

        if not title_string:
            if not animal_name:
                animal_name = 'noAnimalName'
                warnings.warn('no animal name specified')
            if not exp_tag:
                exp_tag = 'noExpTag'
                warnings.warn('no experiment tag specified')
            if cell_id is None:
                cell_id = 'noCellID'
                warnings.warn('no cell ID specified')
            cell_id = str(cell_id)
            title_string = animal_name + '_' + exp_tag + '_' + cell_id + '_' + self.date_time_str
        self.title_string = title_string

        # check to make sure each angle_order is mapped
        if angle_order:
            for a in self.angle_order:
                if a not in self.mapping.keys():
                    warnings.warn('There is an angle in angle_order with no speed mapped: ' + str(a))

        self.last_time = 0
        self.delay = .5

        self.angle_idx = -1

        self.dir = FORWARD
        self.cmd_pos = 0
        self.real_pos = 0
        self.steps = 0

        self.test_log = {}

        self.log = pd.DataFrame(columns=KEY_LIST)

        # connect to Arduino
        ports = list(serial.tools.list_ports.comports())  # Create a list with info for all devices
        connected = False
        # Look for the word 'Arduino' in device info, store this port
        for p in ports:
            if "Arduino" in p[1]:
                connected = True
                port_arduino = p[0]
                break
        if not connected:
            print("No Arduino Found")
            return
        self.ser = serial.Serial(port_arduino, BAUD_RATE)  # Create serial object for the Arduino
        print('Arduino connected. Port: ' + port_arduino)

        # disable debug messages on Arduino
        time.sleep(2)
        self.send_cmd('d')
        # time.sleep(.5)

    def init(self):
        """
        move stepper motor to first angle, set slider to move forward next

        :return:
        """

        # engage stepper
        self.send_cmd('e')
        time.sleep(1)

        # move to initial angle
        if self.angle_order:
            self.angle_idx = 0
            self.move_stepper_to(self.angle_order[self.angle_idx])
        else:
            self.move_stepper_to(0)

        # determine speed with which to move slider
        if self.cmd_pos in self.mapping.keys():
            speed = self.mapping[self.cmd_pos][self.dir]
        else:
            speed = 175

        # move slider so next move will be forward, don't add these slides to log
        self.move_slider(speed, log=False)
        if self.dir == FORWARD:
            self.move_slider(speed, log=False)

    def run_all(self, num_slides):
        """
        iterate through all angels in angle_order and move slider num_slides number of times (round trips)

        :param num_slides: number of round trips slider will make at each angle
        :return:
        """
        idx = len(self.log)
        num = len(self.angle_order)*2*num_slides
        for pos in self.angle_order:
            self.move_stepper_to(pos)
            self.move_slider(num_slides=num_slides)
        self.add_note('This and following ' + str(num-1) + ' trials run together', idx)

    def engage_stepper(self):
        """
        send command to Arduino that will engage the stepper motor

        :return:
        """
        self.send_cmd('e')

    def move_stepper_to(self, pos):
        """
        move stepper to nearest possible angle to specified angle

        :param pos: angle in degrees
        :return:
        """
        if pos != self.cmd_pos:
            while pos > 90:
                pos -= 180
            while pos < -90:
                pos += 180

            cmd = 'm' + ' ' + str(pos)
            self.send_cmd(cmd)

            self.cmd_pos = pos
            self.steps = int(pos * (STEPS_PER_REVOLUTION / 360.0))
            self.real_pos = self.steps * (360.0 / STEPS_PER_REVOLUTION)
            time.sleep(1.5)

    def stepper_next(self, deg=None):
        """
        move stepper to next angle in angle_order
        if an angle is passed in, stepper will rotate forward that number of degrees

        :param deg: (optional and not recommended) move stepper forward this many degrees
        :return:
        """

        if deg is None:
            if self.angle_order:
                self.angle_idx += 1
                if self.angle_idx == len(self.angle_order):
                    self.angle_idx = 0
                angle = self.angle_order[self.angle_idx]
                self.move_stepper_to(angle)
            else:
                print('No angle_order stored. Pass in degrees to increment or set mapping.')
        else:
            self.move_stepper_to(self.cmd_pos + deg)

        print('New angle: %d' % self.cmd_pos)

    def stepper_prev(self, deg=None):
        """
        move stepper to previous angle in angle_order
        if an angle is passed in, stepper will rotate backward that number of degrees

        :param deg: (optional and not recommended) move stepper backward this many degrees
        :return:
        """

        if deg is None:
            if self.angle_order:
                self.angle_idx -= 1
                if self.angle_idx == -len(self.angle_order):
                    self.angle_idx = -1
                angle = self.angle_order[self.angle_idx]
                self.move_stepper_to(angle)
            else:
                print('No angle_order stored. Pass in degrees to increment or set mapping.')
        else:
            self.move_stepper_to(self.cmd_pos + deg)

        print('New angle: %d' % self.cmd_pos)

    def move_slider(self, cmd_speed=None, num_slides=0, log=True):
        """
        move slider to other end of track, writes new data to file after each time slider is moved

        :param speed: (optional) speed with which to move, if None passed in uses mapped speed
        :param num_slides: number of round trips slider should make (pass in 0 to just move to other end of track)
        :param log: (bool) add slides to log
        :return:
        """
        if num_slides:
            num_slides *= 2
        else:
            num_slides = 1
            
        speed = cmd_speed

        for _ in range(num_slides):
            data = {}
            for k in KEY_LIST:
                data[k] = None

            if cmd_speed is None:
                if self.cmd_pos in self.mapping.keys():
                    speed = self.mapping[self.cmd_pos][self.dir]
                else:
                    print('There is no speed mapping for current angle. Pass in a '
                          'speed or move stepper to a mapped position.')
                    return

            # TODO figure out what this does. Why didn't I just use time.sleep()? why is this in beginning?
            # most likely should just be removed
            while(time.time() - self.last_time) < self.delay:
                pass

            data['stepper_ang'] = self.real_pos
            data['cmd_speed'] = speed
            data['direction'] = self.dir

            self.send_cmd(DIR_STR[self.dir] + ' ' + str(speed))
            data['start_time'] = get_time_str()

            print('Angle: %d\tSpeed: %d\tDirection: %s' % (self.cmd_pos, speed, DIR_STR[self.dir]))

            while not self.ser.in_waiting:
                pass
            data['end_time'] = get_time_str()
            self.last_time = time.time()
            time.sleep(.25)
            last_char = self.ser.read().decode()
            while last_char == '\r' or last_char == '\n':
                last_char = self.ser.read().decode()

            if last_char == '1':
                data['result'] = 'pass'
            elif last_char == '!':
                data['result'] = 'fail'
            else:
                data['result'] = 'comm_error'

            if self.dir:
                self.dir = REVERSE
            else:
                self.dir = FORWARD

            if log:
                self.log = self.log.append(data,ignore_index=True)
                self.write_data_to_csv()

            time.sleep(1)

    def add_note(self, note, index=-1):
        """
        add a note to 'notes' column of data, saves changes

        :param note: text note to add
        :param index: trial to which note should be added. default is to add to last trial run
        :return:
        """
        self.log.iloc[index, self.log.columns.get_loc('notes')] = note
        self.write_data_to_csv()

    def send_cmd(self, cmd):
        """
        send string command to Arduino

        :param cmd: (string) command to send
        :return:
        """
        # print(cmd)
        for c in cmd:
            self.ser.write(c.encode())
            time.sleep(.01)
        self.ser.write('\r'.encode())
        time.sleep(.01)

    def print_slider_pos(self):
        """
        use sparingly, high error rate
        query Arduino for current slider position and print to screen

        :return:
        """
        self.send_cmd('read')
        print(struct.unpack('H', self.ser.read(2)))
        print(' %')

    def print_stepper_pos(self):
        """
        print full stepper info to screen, commanded position, real position, and current step number

        :return:
        """
        print('Target pos:\t' + str(self.cmd_pos) + '\tdeg')
        print('Real pos:\t' + str(self.real_pos) + ' \tdeg')
        print('Current steps:\t' + str(self.steps))

    def set_angle_idx(self, idx=0):
        """
        go to angle at idx in angle_order

        :param idx: index of desired angle in angle order
        :return:
        """
        self.angle_idx = idx - 1
        self.stepper_next()

    def test_all(self, num_slides=5):
        """
        runs test_slider speed for all angles using current speed mapping
        errors a lot, may be better off running one at a time then resetting everything

        :param num_slides: number of slider round trips to take at each angle
        :return:
        """

        idx = len(self.log)
        num = len(self.angle_order) * 2 * num_slides
        for pos in self.angle_order:
            self.move_stepper_to(pos)
            f_speed = self.mapping[self.cmd_pos][FORWARD]
            b_speed = self.mapping[self.cmd_pos][REVERSE]
            self.test_slider_speed(forward_speed=f_speed, backward_speed=b_speed, num_trials=num_slides)
        self.add_note('This and following ' + str(num - 1) + ' trials run together as part of the startup routine', idx)
        self.analyze_test_log()

    def test_slider_speed(self, forward_speed=150, backward_speed=150, num_trials=1):
        """
        NOTE: this fails a lot after the angle is changed. also fails if slider stalls.
                all successful slides were recorded, you may still analyze these. start new session to clear error.
        record the average speed of the slider through the middle third of the track in meters per second
        call analyze_test_log() to print average speeds to screen

        :param motor_speed: value 0 - 255
        :param num_trials: number of round trips slider should take
        :return: timestamps_fwd, pos_fwd, timestamps_bck, pos_bck, f_speed, b_speed from last trial
        """

        # make sure correct direction set
        if self.dir:
            self.move_slider(150, log=False)
        time.sleep(1)
        # enter debug mode
        self.send_cmd('d')
        while not self.ser.in_waiting:
            pass
        time.sleep(.25)
        while self.ser.in_waiting:
            self.ser.read()

        timestamps_fwd = []
        pos_fwd = []
        timestamps_bck = []
        pos_bck = []

        for _ in range(num_trials):

            last_char = ' '
            read_str = ''

            self.move_slider(backward_speed)

            while not self.ser.in_waiting:
                pass
            # read timestamps
            last_char = self.ser.read().decode()
            while last_char == '\r' or last_char == '\n':
                last_char = self.ser.read().decode()
            while last_char != '\r' and last_char != '\n':
                read_str += last_char
                last_char = self.ser.read().decode()

            # remove leading and trailing spaces commas etc
            read_str = read_str.strip()
            read_str = read_str.strip(',')

            timestamps_bck = [int(i) for i in read_str.split(', ')]

            read_str = ''
            last_char = self.ser.read().decode()
            while last_char == '\r' or last_char == '\n':
                last_char = self.ser.read().decode()
            while last_char != '\r' and last_char != '\n':
                read_str += last_char
                last_char = self.ser.read().decode()

            read_str = read_str.strip()
            read_str = read_str.strip(',')

            pos_bck = [int(i) for i in read_str.split(', ')]

            read_str = ''

            self.add_note('testing')

            self.move_slider(forward_speed)

            while not self.ser.in_waiting:
                pass
            # read timestamps
            last_char = self.ser.read().decode()
            while last_char == '\r' or last_char == '\n':
                last_char = self.ser.read().decode()
            while last_char != '\r' and last_char != '\n':
                read_str += last_char
                last_char = self.ser.read().decode()

            # remove leading and trailing spaces commas etc
            read_str = read_str.strip()
            read_str = read_str.strip(',')

            timestamps_fwd = [int(i) for i in read_str.split(', ')]

            read_str = ''
            last_char = self.ser.read().decode()
            while last_char == '\r' or last_char == '\n':
                last_char = self.ser.read().decode()
            while last_char != '\r' and last_char != '\n':
                read_str += last_char
                last_char = self.ser.read().decode()

            read_str = read_str.strip()
            read_str = read_str.strip(',')

            pos_fwd = [int(i) for i in read_str.split(', ')]

            tf = timestamps_fwd.copy()
            pf = pos_fwd.copy()
            tb = timestamps_bck.copy()
            pb = pos_bck.copy()

            while pf[0] < 1024 / 3 or pf[0] > 1024.0 * (2 / 3):
                pf.pop(0)
                tf.pop(0)
            while pf[-1] < 1024 / 3 or pf[-1] > 1024 * (2 / 3):
                pf.pop(-1)
                tf.pop()
            while pb[0] < 1024 / 3 or pb[0] > 1024.0 * (2 / 3):
                pb.pop(0)
                tb.pop(0)
            while pb[-1] < 1024 / 3 or pb[-1] > 1024 * (2 / 3):
                pb.pop(-1)
                tb.pop()

            f_time = tf[-1] - tf[0]
            f_pos = abs(pf[-1] - pf[0])
            f_time = f_time / 1000000.0
            f_pos = f_pos / 1024.0 * .10
            f_speed = f_pos / f_time

            b_time = tb[-1] - tb[0]
            b_pos = abs(pb[-1] - pb[0])
            b_time = b_time / 1000000.0
            b_pos = b_pos / 1024.0 * .10
            b_speed = b_pos / b_time

            print("fwd setting: %d" % forward_speed)
            print("bck setting: %d" % backward_speed)
            print("forward speed: %.4f m/s" % f_speed)
            print("backward speed: %.4f m/s" % b_speed)
            

            self.add_note('testing')
            self._update_test_log(self.cmd_pos, forward_speed, backward_speed, f_speed, b_speed)

        self.send_cmd('d')
        time.sleep(1)
        while self.ser.in_waiting:
            self.ser.read()

        return timestamps_fwd, pos_fwd, timestamps_bck, pos_bck, f_speed, b_speed

    def _update_test_log(self, angle, forward_speed, backward_speed, fwd, rev):
        """
        records angle, comanded motor speed, and slider speeds from last test trial run to self.test_log

        :param angle: stepper angle
        :param speed: commanded motor speed
        :param fwd: forward velocity
        :param rev: reverse velocity
        :return:
        """
        if angle not in self.test_log.keys():
            self.test_log[angle] = {'forward': {}, 'backward': {}}

        if forward_speed not in self.test_log[angle]['forward'].keys():
            self.test_log[angle]['forward'][forward_speed] = [fwd]
        else:
            self.test_log[angle]['forward'][forward_speed].append(fwd)
        if backward_speed not in self.test_log[angle]['backward'].keys():
            self.test_log[angle]['backward'][backward_speed] = [rev]
        else:
            self.test_log[angle]['backward'][backward_speed].append(rev)

    def analyze_test_log(self):
        """
        print fwd and bwd mean velocity and stdev for each angle/motor_speed combination that was tested

        :return:
        """
        for k, speed_dict in self.test_log.items():
            print("angle: %d" % k)
            for speed, v in speed_dict['forward'].items():
                print("\tFORWARD speed: %d\tnum_trials: %d" %(speed, len(v)))
                try:
                    print("\t\tfwd_vel\tmean: %.4f m/s\tstdev: %.6f"
                          % (statistics.mean(v), statistics.stdev(v)))
                except:
                    print('Too few trials to analyze.')
            for speed, v in speed_dict['backward'].items():
                print("\tBACKWARD speed: %d\tnum_trials: %d" %(speed, len(v)))
                try:
                    print("\t\tbck_vel\tmean: %.4f m/s\tstdev: %.6f"
                          % (statistics.mean(v), statistics.stdev(v)))
                except:
                    print('Too few trials to analyze.')

    def store_test_log(self, title=None, dest_folder=None):
        """
        write test log to file
        defaults to prepending 'testLog_' to standard title and storing in standard data directory
        just a wrapper around pickle dump

        :param title: (optional) name of test_log file
        :param dest_folder: directory in which to store test_log
        :return:
        """
        if not title:
            title = 'testLog_' + self.title_string
        if not dest_folder:
            dest_folder = self.dest_folder

        filestr = os.path.join(dest_folder, title)

        with open(filestr, 'wb') as fid:
            pickle.dump(self.test_log, fid)

        print('stored: ' + filestr)

    def write_data_to_csv(self, file_path=None):
        """
        write trial log to file (note this is called anytime the slider is moved or a note is added)
        if no argument is passed, will store using values passed in when object was created:
            dest_folder/animalName_expTag_dateTime.scv

        :param file_path: full path (including file name) to write data
        :return:
        """
        if not file_path:
            file_path = os.path.join(self.dest_folder, self.title_string+'.csv')

        if '.csv' not in file_path:
            file_path += '.csv'

        self.log.to_csv(file_path)


def get_datetime_str():
    """
    returns current datetime as a string with format yyyy-mm-dd-HH-MM-SS

    :return: returns current datetime as a string with format yyyy-mm-dd-HH-MM-SS
    """
    date_time = datetime.datetime.now()
    time_str = str(date_time.time()).split('.')[0]  # Get string with just hh:mm:ss
    time_str = time_str.replace(':', '-')  # Now hh-mm-ss
    date_str = str(date_time.date())  # Date in format yyyy-mm-dd
    date_time_str = '-'.join([date_str, time_str])
    return date_time_str


def get_time_str():
    """
    returns current time as a string with format HH-MM-SS

    :return: returns current datetime as a string with format HH-MM-SS
    """
    date_time = datetime.datetime.now()
    time_str = str(date_time.time())
    return time_str


def store_mapping(d, name=None, directory='mappings'):
    """
    stores mapping as a pickle file
    just a wrapper around pickle dump

    :param d: mapping dictionary
    :param name: name of mapping
    :param directory: directory to store mapping
    :return:
    """
    if not directory:
        directory = os.getcwd()
    if not name:
        name = 'noName' + '_' + get_datetime_str()

    filestr = os.path.join(directory, name + '.pckl')

    with open(filestr, 'wb') as fid:
        pickle.dump(d, fid)

    print('stored: ' + filestr)
    print(d)


def load_mapping(name, directory='mappings'):
    """
    returns the mapping 'name' stored in 'directory'
    just a wrapper around pickle load

    :param name:
    :param directory:
    :return:
    """
    if '.pckl' not in name:
        name += '.pckl'
    if directory:
        filestr = os.path.join(directory, name)
    else:
        filestr = name
    with open(filestr, 'rb') as fid:
        return pickle.load(fid)

