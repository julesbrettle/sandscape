import serial
import time
import math
import threading
import queue
import json
import signal
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Union
from enum import Enum

from local.local_constants import *
from utils import *
from parse_grbl_status import *
from state import *

# --- GRBL COMMANDS ---
class GrblCmd(Enum):
    EMPTY = bytes("",  'utf-8')
    PING = bytes("\n",  'utf-8')
    STATUS = bytes("?",  'utf-8')
    HOLD = bytes("!\n",  'utf-8')
    RESUME = bytes("~\n",  'utf-8')
    SOFT_RESET = bytes([0x18])
    UNLOCK = bytes("$X\n",  'utf-8')
    HOME = bytes("$H\n",  'utf-8')
    GET_SETTINGS = bytes("$$\n",  'utf-8')
    
    # HARD_RESET = bytes([0x85]) ???? maybe? i don't know what this does

class GrblSendMsgType(Enum):
    EMPTY = "empty"
    CMD = "cmd"
    MOVE = "move"
    SETTING = "setting"
    
class GrblRespType(Enum):
    NONE = "none"
    RESP_OTHER = "resp_other"
    RESP_STATUS = "resp_status"
    RESP_OK = "resp_ok"
    RESP_ALARM = "resp_alarm"
    RESP_ERROR = "resp_error"
    RESP_STARTUP = "resp_startup"
    RESP_CHECKLIMITS = "resp_checklimits"
    RESP_NEEDUNLOCK = "resp_needunlock"
    RESP_UNLOCKED = "resp_unlocked"
    RESP_SETTING = "resp_setting"
    
class GrblStatusOptions(Enum):
    IDLE = "Idle"
    RUN = "Run"
    HOLD = "Hold"
    ALARM = "Alarm"

# --- GRBL SETTINGS ---
GRBL_SETTINGS = {
    0: 10,  # Step pulse, microseconds
    1: 25,  # Step idle delay, milliseconds
    2: 0,  # Step port invert, XYZmask*
    3: 4,  # Direction port invert, XYZmask*
    4: 0,  # Step enable invert, (0=Disable, 1=Invert)
    5: 0,  # Limit pins invert, (0=N-Open. 1=N-Close)
    6: 0,  # Probe pin invert, (0=N-Open. 1=N-Close)
    10: 255,  # Status report, '?' status contents
    11: 0.010,  # Junction deviation, mm
    12: 0.002,  # Arc tolerance, mm
    13: 0,  # Report in inches, (0=mm. 1=Inches)**
    20: 0,  # Soft limits, (0=Disable. 1=Enable, Homing must be enabled)
    21: 1,  # Hard limits, (0=Disable. 1=Enable)
    22: 1,  # Homing cycle, (0=Disable. 1=Enable)
    23: 3,  # Homing direction invert, XYZmask* Sets which corner it homes to.
    24: 100.000,  # Homing feed, mm/min
    25: 3000.000,  # Homing seek, mm/min
    26: 250,  # Homing debounce, milliseconds
    27: 8.000,  # Homing pull-off, mm
    30: 1000,  # Max spindle speed, RPM
    31: 0,  # Min spindle speed, RPM
    32: 0,  # Laser mode, (0=Off, 1=On)
    100: 40.000,  # Number of X steps to move 1mm
    101: 40.000,  # Number of Y steps to move 1mm
    102: 22.222,  # Number of Z steps to move 1mm (degree)
    110: 10000.000,  # X Max rate, mm/min
    111: 10000.000,  # Y Max rate, mm/min
    112: 10000.000,  # Z Max rate, mm/min
    120: 50.000,  # X Acceleration, mm/sec^2
    121: 50.000,  # Y Acceleration, mm/sec^2
    122: 10.000,  # Z Acceleration, mm/sec^2
    130: 550.000,  # X Max travel, mm Only for Homing and Soft Limits.
    131: 345.000,  # Y Max travel, mm Only for Homing and Soft Limits.
    132: 200.000  # Z Max travel, mm Only for Homing and Soft Limits.
}

@dataclass
class GrblSendMsg:
    msg_type: GrblSendMsgType = GrblSendMsgType.EMPTY
    msg: bytes = GrblCmd.EMPTY.value
    move: Move = field(default_factory=lambda: Move())
    sent: bool = False
    response: str = ''
    received: bool = False
    def __repr__(self):
        if type(self.msg) == GrblCmd:
            msg_str = self.msg.name
        else:
            msg_str = self.msg
        return f"GrblSendMsg(msg_type={self.msg_type.name}, msg={msg_str}, move={self.move}, sent={self.sent}, response={self.response}, received={self.received})"
    
class GrblRespMsg:
    msg_type: GrblRespType = GrblRespType.NONE
    msg: str = ""
    handled: bool = False
    def __repr__(self):
        return f"GrblRespMsg(msg_type={self.msg_type.name}, msg={self.msg}, handled={self.handled})"

def grbl_resp_msg_txt_to_obj(msg_txt):
    msg_obj = GrblRespMsg()
    msg_obj.msg = msg_txt
    # determine response type
    if "<" in msg_txt and ">" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_STATUS
    elif "ok" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_OK
    elif "ALARM" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_ALARM
    elif "error" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_ERROR
    elif "Grbl 1.1h ['$' for help]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_STARTUP
    elif "[MSG:Check Limits]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_CHECKLIMITS
    elif "[MSG:'$H'|'$X' to unlock]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_NEEDUNLOCK
    elif "[MSG:Caution: Unlocked]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_UNLOCKED
    elif "=" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_SETTING
    else:
        msg_obj.msg_type = GrblRespType.RESP_OTHER
    return msg_obj

def format_move(move):
    return bytes(f"G1 X{move.r:.2f} Z{move.t_grbl:.2f} F{move.s:.2f}\n",  'utf-8')

@dataclass
class GrblCommunicator(SerialCommunicator):
    state: State = field(default_factory=lambda: State())
    port_name: str = UNO_SERIAL_PORT_NAME
    baud_rate: int = UNO_BAUD_RATE
    display_name: str = "Uno"

    run_comm_timeout: float = 1.0

    need_reset: bool = False
    need_ping: bool = False
    need_status: bool = False
    need_unlock: bool = False
    need_send_setting: bool = False
    need_get_settings: bool = False
    need_send_next_move: bool = False
    expecting_extra_msg: bool = False

    curr_grbl_settings: dict = field(default_factory=dict)
    last_grbl_resp: GrblRespMsg = field(default_factory=lambda: GrblRespMsg())
    next_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())
    prev_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())

    def flags_to_str(self, indent=0):
        s = ""
    
        to_show = {"need_reset", "need_ping", "need_status", "need_unlock", "need_send_setting", "need_get_settings", "need_send_next_move", "expecting_extra_msg"}
        gc_dict = self.__dict__
        for x in to_show:
            new = f"grbl_comm.{x}={str(gc_dict[x])}, "
            # new = new.replace("True", green('T'))
            # new = new.replace("False", red('F'))
            # new = new.rjust(30+indent)
            s += indent*' ' +new
        return s[0:-2]
    
    def startup(self):
        self.serial_connect()
        
        global uno_serial_port
        uno_serial_port = self.serial_port

        pprint("Resetting and unlocking GRBL...")
        self.need_reset = True
        self.need_unlock = True
        success = self.run()
        return success
    
    def reset(self):
        self.need_reset = True
        success = self.run()
        return success

    def unlock(self):
        self.need_unlock = True
        success = self.run()
        return success
        
    def status(self):
        pprint("Checking GRBL status...")
        self.need_status = True
        success = self.run()
        return success

    def ping(self):
        pprint("Pinging GRBL...")
        self.need_ping = True
        success = self.run()
        if not success:
            pprint(f"{red('ERROR')}: GRBL did not respond to ping.")
        return success

    def send_next_move(self):
        self.need_send_next_move = True
        success = self.run()
        return success

    def sync_settings(self, grbl_settings=GRBL_SETTINGS):
        pprint("Syncing GRBL settings...")
        self.need_get_settings = True
        success = self.run()
        if not success:
            pprint(f"{red('ERROR')}: Could not get settings.")
            return False
        if self.curr_grbl_settings == grbl_settings:
            pprint(f"Settings are already up to date.")
            return True
        else:
            settings_diff = set(grbl_settings.items()) ^ set(self.curr_grbl_settings.items())
            pprint(f"Settings are out of date. Recieved settings: {self.curr_grbl_settings}")
            pprint(f"Settings Diff: {settings_diff}")
            pprint(f"Updating GRBL settings...")
            for key, value in grbl_settings.items():
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.SETTING, msg=bytes(f"${key}={value}\n",  'utf-8'))
                self.need_send_setting = True
                success = self.run()
                if not success:
                    pprint(f"{red('ERROR')}: Could not update setting {self.next_grbl_msg}")
                    return False
            self.need_get_settings = True
            success = self.run()
            if not success:
                pprint(f"{red('ERROR')}: Could not get settings after updating.")
                return False
            if self.curr_grbl_settings == grbl_settings:
                pprint(f"Settings are now up to date.")
                return True
            else:
                pprint(f"{red('ERROR')}: GRBL settings are still out of date.")
                return False

    def grbl_home(self):
        pprint("Starting GRBL's built-in homing...")
        self.state.flags.need_homing = True
        success = self.run()
        if self.state.flags.run_control_loop:
            self.state.flags.run_control_loop = success
        return success
    
    def hard_reset(self):
        """Disconnect and recconnect serial to zero MPos values"""
        self.serial_disconnect()
        old_timeout = self.run_comm_timeout
        self.run_comm_timeout = 10
        self.serial_connect()
        self.run_comm_timeout = old_timeout
        return True
    
    def run(self):
        """Sends messages to grbl and manages self based on response. Should be the only function that main control loop runs to communicate with grbl."""
        # global data_queue, serial_port
        # assume all previous msgs are handled
        pprint("Starting GRBL communicator...")
        if self.state.flags.need_grbl_hard_reset and not self.need_ping:
                pprint(f"    GC.run:1 - self.state.flags.need_grbl_hard_reset={self.state.flags.need_grbl_hard_reset} and not self.need_ping={self.need_ping} -> hard_reset()")
                self.state.flags.need_grbl_hard_reset = False
                self.hard_reset()
        timeout = self.run_comm_timeout
        while True:
            # pprint(self)
            if self.expecting_extra_msg:
                pprint(f"    GC.run:2 - self.expecting_extra_msg={self.expecting_extra_msg} -> sleep")
                time.sleep(0.1)
            # handle missed messages
            while not self.data_queue.empty():
                pprint(f"    GC.run:3 - self.data_queue.empty()={self.data_queue.empty()} -> self.data_queue.get()")
                msg_txt = self.data_queue.get()
                self.data_queue.task_done()
                self.last_grbl_resp = grbl_resp_msg_txt_to_obj(msg_txt)
                pprint(f"    GC.run:4 - self.last_grbl_resp={self.last_grbl_resp} -> handle_grbl_response()")
                self.handle_grbl_response()
            # generate self.next_grbl_msg
            self.generate_msg()
            # if there is still a message to send
            if (self.next_grbl_msg.msg_type != GrblSendMsgType.EMPTY):
                pprint(f"    GC.run:5 - self.next_grbl_msg={self.next_grbl_msg} -> self.grbl_write_next_msg()")
                # wait extra long if we're running homing
                if self.next_grbl_msg.msg == GrblCmd.HOME.value:
                    timeout = 60
                self.grbl_write_next_msg() # send self.next_grbl_msg
                self.next_grbl_msg.sent = True
                start_time = time.time()
                end_time = start_time + timeout
                while True:
                    if time.time() > end_time:
                        print_error(f"    GC.run:6 - Grbl communicator timed out after {time.time()-start_time} seconds.")
                        exit()
                        self.last_grbl_resp = GrblRespMsg()
                        return False
                    if self.data_queue.empty():
                        pprint(f"    GC.run:7 - self.data_queue.empty()=True -> sleep")
                        time.sleep(0.02)
                    else:
                        pprint(f"    GC.run:8 - self.data_queue.empty()=False -> self.data_queue.get()")
                        msg_txt = self.data_queue.get()
                        self.data_queue.task_done()
                        self.last_grbl_resp = grbl_resp_msg_txt_to_obj(msg_txt)
                        break
                pprint(f"    GC.run:9 - self.last_grbl_resp={self.last_grbl_resp} -> handle_grbl_response()")
                self.handle_grbl_response()
                pprint(f"    GC.run:10 - -> run self.generate_msg()")
                self.prev_grbl_msg = self.next_grbl_msg
                self.generate_msg() # generates empty msg if no further com needed
                pprint(f"    GC.run:11 - self.next_grbl_msg={self.next_grbl_msg}")
            if (self.next_grbl_msg.msg_type == GrblSendMsgType.EMPTY 
                and not self.expecting_extra_msg):
                pprint(f"    GC.run:12 - self.next_grbl_msg.msg_type={self.next_grbl_msg.msg_type} and not self.expecting_extra_msg={self.expecting_extra_msg} -> finish")
                pprint(f"    Grbl communicator finished sucessfully.")
                return True
    
    def handle_grbl_response(self):
        """Sets self in self based on self.last_grbl_resp."""
        # self.expecting_extra_msg = False
        pprint(f"    Handling GRBL response (HGR): {self.last_grbl_resp} ...")
        isgood = False
        self.next_grbl_msg.response = self.last_grbl_resp.msg
        if self.last_grbl_resp.msg_type == GrblRespType.RESP_SETTING:
            msg = self.last_grbl_resp.msg.split("$")[1]
            key = int(msg.split("=")[0])
            value = float(msg.split("=")[1])
            self.curr_grbl_settings[key] = value
            self.need_get_settings = False
            isgood = True
            self.expecting_extra_msg = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_STATUS:
            if self.next_grbl_msg.msg == GrblCmd.STATUS.value:
                isgood = True
                self.need_status = False
                self.update_state_from_grbl_msg(self.last_grbl_resp.msg)
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_OK:
            if (self.next_grbl_msg.msg == GrblCmd.PING.value
                or self.next_grbl_msg.msg == GrblCmd.HOME.value
                or self.next_grbl_msg.msg == GrblCmd.HOLD.value
                or self.next_grbl_msg.msg == GrblCmd.RESUME.value
                or self.next_grbl_msg.msg == GrblCmd.UNLOCK.value
                or self.next_grbl_msg.msg_type == GrblSendMsgType.MOVE
                or self.next_grbl_msg.msg_type == GrblSendMsgType.SETTING
                or self.next_grbl_msg.msg_type == GrblSendMsgType.EMPTY):
                isgood = True
                if self.next_grbl_msg.msg == GrblCmd.HOME.value:
                    self.state.flags.need_homing = False
            self.expecting_extra_msg = False
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_ALARM:
            self.state.grbl.status = GrblStatusOptions.ALARM.value
            self.need_reset = True
            self.need_unlock = True
            self.need_status = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_ERROR:
            self.need_reset = True
            self.need_unlock = True
            self.need_status = True
            self.run_control_loop = False
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_STARTUP:
            self.need_reset = False
            self.need_unlock = True
            self.need_status = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_CHECKLIMITS:
            # self.expecting_extra_msg = True # TODO: Check if this is needed
            self.need_unlock = True
            self.need_status = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_NEEDUNLOCK:
            self.need_unlock = True
            self.need_status = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_UNLOCKED:
            self.need_unlock = False
            self.need_status = True
            self.expecting_extra_msg = True
        elif self.last_grbl_resp.msg_type == GrblRespType.RESP_OTHER:
            self.need_status = True
        else:
            print_error(f"Unrecognized GRBL response: '{self.last_grbl_resp.msg}'")
            self.need_reset = True
            self.need_unlock = True
            self.need_status = True
            # self.run_control_loop = False
        if isgood:
            self.next_grbl_msg.received = True
            self.next_grbl_msg.response = self.last_grbl_resp.msg
            if self.next_grbl_msg.msg_type == GrblSendMsgType.MOVE:
                self.state.next_move.received = True
                self.need_send_next_move = False
            if self.next_grbl_msg.msg == GrblCmd.UNLOCK.value:
                self.need_unlock = False
            if self.next_grbl_msg.msg == GrblCmd.SOFT_RESET.value:
                self.need_reset = False
            if self.next_grbl_msg.msg_type == GrblSendMsgType.SETTING:
                self.need_send_setting = False

        pprint(f"        HGR results: isgood={isgood}, {self.flags_to_str()}")
        # pprint(f"            isgood: {isgood}")
        # pprint(f"{self.flags_to_str(indent=12)}")


    def update_state_from_grbl_msg(self, grbl_msg):
        """Take uno serial msg and integrate into state."""
        pprint("Updating status from grbl msg...")
        status_dict = parse_grbl_status(grbl_msg)
        self.state.grbl.status = status_dict["State"]
        self.state.grbl.mpos_r = status_dict["MPos"][0]
        self.state.grbl.mpos_t = status_dict["MPos"][1]
        self.state.grbl.feed_rate = status_dict["FeedRate"]
        self.state.grbl.planner_buffer = status_dict["PlannerBuffer"]
        self.state.grbl.rx_buffer = status_dict["RxBuffer"]
        if "Pn" in status_dict:
            self.state.grbl.pnX = "X" in status_dict["Pn"]
            self.state.grbl.pnY = "Y" in status_dict["Pn"]
        else:
            self.state.grbl.pnX = False
            self.state.grbl.pnY = False
        self.state.limits_hit.soft_r_min = self.state.grbl.mpos_r <= R_MIN
        self.state.limits_hit.soft_r_max = self.state.grbl.mpos_r >= R_MAX
        self.state.limits_hit.hard_r_min = self.state.grbl.pnX
        self.state.limits_hit.hard_r_max = self.state.grbl.pnY
        
        pprint(f"Updated status: {self}")

    def next_move_to_msg(self):
        """Check move validity based on limits and grbl status, then make next msg next_move."""
        self.set_t_grbl()
        if self.state.next_move != None and not self.state.next_move.is_empty():
            if self.state.check_move(self.state.next_move):
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.MOVE, msg=format_move(self.state.next_move), move=self.state.next_move)
                pprint(f"Next msg: {self.next_grbl_msg}")
            else:
                pprint(print_error("Tried to send invalid move."))
                self.need_send_next_move = False
        else:
            self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)
            self.need_send_next_move = False


    def homing_next_msg(self):
        if self.state.limits_hit.hard_r_min:
            self.state.next_move = Move(r=self.state.grbl.mpos_r+30, t=self.state.grbl.mpos_t, s=3000)
            self.set_t_grbl()
        elif self.state.limits_hit.hard_r_max:
            self.state.next_move = Move(r=self.state.grbl.mpos_r-30, t=self.state.grbl.mpos_t, s=3000)
            self.set_t_grbl()
        else:
            if GRBL_HOMING_ON and not CUSTOM_HOMING_ON:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.HOME.value)
        self.next_move_to_msg()

    def generate_msg(self):
        """Uses self and internal logic to determine which messages to send to GRBL. 
        Sets self.next_grbl_msg"""
        pprint("    Generating GRBL message...")
        if self.need_ping:
            self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.PING.value)
            pprint(f"        GM:1 - self.need_ping={self.need_ping} -> self.next_grbl_msg={self.next_grbl_msg}")
            self.need_ping = False
        
        if self.state.phase == Phase.SETUP:
            if self.need_reset:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.SOFT_RESET.value)
                pprint(f"        GM:2 - self.need_reset={self.need_reset} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_status:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.STATUS.value)
                pprint(f"        GM:3 - self.need_status={self.need_status} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_unlock:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.UNLOCK.value)
                pprint(f"        GM:4 - self.need_unlock={self.need_unlock} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_get_settings:
                pprint(f"        GM:5 - self.need_get_settings={self.need_get_settings} -> self.next_grbl_msg={self.next_grbl_msg}")
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.GET_SETTINGS.value)
            elif self.need_send_setting:
                pprint(f"        GM:6 - self.need_send_setting={self.need_send_setting} -> self.next_grbl_msg={self.next_grbl_msg}")
                if self.next_grbl_msg.msg_type != GrblSendMsgType.SETTING:
                    pprint(f"{red('ERROR')}: self.need_send_setting={self.need_send_setting} but self.next_grbl_msg={self.next_grbl_msg} \n       Please format self.next_grbl_msg.msg_type")
            elif self.state.flags.need_homing and GRBL_HOMING_ON:
                pprint(f"        GM:7 - self.state.flags.need_homing={self.state.flags.need_homing} and GRBL_HOMING_ON={GRBL_HOMING_ON} -> self.next_grbl_msg={self.next_grbl_msg}")
                self.homing_next_msg()
            else:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)
                pprint(f"        GM:8 - self.next_grbl_msg={self.next_grbl_msg}")

        if self.state.phase == Phase.SENSE:
            if self.need_reset:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.SOFT_RESET.value)
                pprint(f"        GM:9 - self.need_reset={self.need_reset} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_status:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.STATUS.value)
                pprint(f"        GM:10 - self.need_status={self.need_status} -> self.next_grbl_msg={self.next_grbl_msg}")
            else:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)
                pprint(f"        GM:11 - self.next_grbl_msg={self.next_grbl_msg}")

        if self.state.phase == Phase.ACT:
            if self.need_reset:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.SOFT_RESET.value)
                pprint(f"        GM:12 - self.need_reset={self.need_reset} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_status:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.STATUS.value)
                pprint(f"        GM:13 - self.need_status={self.need_status} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_unlock:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.UNLOCK.value)
                pprint(f"        GM:14 - self.need_unlock={self.need_unlock} -> self.next_grbl_msg={self.next_grbl_msg}")
            elif self.need_send_next_move:
                self.next_move_to_msg()
                pprint(f"        GM:15 - self.need_send_next_move={self.need_send_next_move} -> self.next_grbl_msg={self.next_grbl_msg}")
            else:
                self.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)
                pprint(f"        GM:16 - self.next_grbl_msg={self.next_grbl_msg}")


    def grbl_write_next_msg(self):
        """Writes self.next_msg.msg to serial port."""
        x = self.next_grbl_msg.msg
        # if type(x) == GrblCmd:
        #     pprint(f"{time.time():.5f} | writing to grbl: {x.value}")
        #     serial_port.write(x.value)
        # elif type(x) == str:
        #     pprint(f"{time.time():.5f} | writing to grbl: {bytes(x, 'utf-8')}")
        #     serial_port.write(bytes(x,  'utf-8'))
        if type(x) == bytes:
            pprint(blue(f"{time.time():.3f} | Writing to grbl: {x}"))
            self.serial_port.write(x)
        else:
            print_error(f"self.next_grbl_msg.msg{self.next_grbl_msg.msg} is of the type {type(self.next_grbl_msg.msg)} not bytes.")

    def set_t_grbl(self):
        # standard case
        if (self.state.prev_move.t_grbl != None 
            and self.state.prev_move.t_grbl != None 
            and self.state.next_move.t != None):
            delta = self.state.next_move.t - self.state.prev_move.t # pyright: ignore[reportOperatorIssue]
            pprint(f"set_t_grbl delta={delta}")
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            pprint(f"set_t_grbl modified delta={delta}")
            pprint(f"set_t_grbl self.state.prev_move.t_grbl={self.state.prev_move.t_grbl}")
            self.state.next_move.t_grbl = self.state.prev_move.t_grbl + delta
            pprint(f"set_t_grbl self.state.next_move.t_grbl={self.state.next_move.t_grbl}")
            return True
        elif self.state.next_move.t == None:
            return False
        # if  self.prev_move.t == None:
        #     if self.prev_move.t_grbl == None:
        #         self.prev_move.t_grbl = self.grbl.mpos_t
        #     self.prev_move.t = self.prev_move.t_grbl % 360
        # if  self.prev_move.t_grbl == None:
        #     if self.prev_move.t == None:
        #         self.prev_move.t_grbl = 0
        #     else:
        #         self.prev_move.t = self.prev_move.t_grbl

    def direct_write(self, msg : bytes):
        """For testing purposes only, not for use in main loop."""
        self.serial_port.write(msg)

    