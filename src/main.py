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

import debugger_window

from parse_grbl_status import *
from parse_svg import SVGParser, create_polar_plot, create_cartesian_plot

from local.local_constants import *
from utils import *
from state import *
from grbl import *
from nano_communicator import *
from mode import *

uno_serial_port = serial.Serial() # global context for sig_handler grbl stop on Ctrl+C

@dataclass
class Sandscape:
    modes_playlist: list = field(default_factory=list)
    state: State = field(default_factory=lambda: State())
    grbl_comm: GrblCommunicator = field(default_factory=lambda: GrblCommunicator())
    nano_comm: NanoCommunicator = field(default_factory=lambda: NanoCommunicator())
    mode_index: int = 0

    _mode: Mode = field(default_factory=lambda: Mode())
    @property
    def mode(self):
        return self._mode
    
    @mode.setter
    def mode(self, v: Mode):
        self._mode.cleanup() # clean up old mode before starting new
        self._mode = v
        self._mode.state = self.state
        self._mode.done = False
        self._mode.startup()
        self.state.flags.input_change = True
        pprint(f"Mode set to: {self._mode.mode_name}")
    
    def stop(self):
        self.grbl_comm.serial_port.write(bytes([0x18])) # no-checks immediate stop
        pprint("Stopped GRBL.")

        self.state.phase = Phase.SETUP
        self.grbl_comm.run_comm_timeout = 0.1
        success = self.grbl_comm.status()
        if not success:
            self.grbl_comm.hard_reset() # grbl stops movement when serial connection is established

        if LOG_PATH or LOG_COMMANDS:
            pprint("Saving data...")
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"../logs/table_log_{timestamp}.json"

            data_to_save = {
                "path_history": self.state.path_history,
                "grbl_command_log": self.state.grbl_command_log
            }

            with open(filename, 'w') as f:
                json.dump(data_to_save, f, indent=4)

            pprint(f"Data saved to {filename}")
    
    def start_next_mode(self):
        if type(self.mode) != HomingSequence:
            self.mode_index = (self.mode_index + 1) % len(self.modes_playlist)
        self.mode = self.modes_playlist[self.mode_index]

    def startup(self):
        if CONNECT_TO_NANO:
            self.nano_comm = NanoCommunicator(state=self.state)
            self.nano_comm.serial_connect()
        else:
            pprint("Not connecting to Arduino Nano.")

        if CONNECT_TO_UNO:
            self.grbl_comm = GrblCommunicator(state=self.state)
            self.grbl_comm.startup()

            if SYNC_GRBL_SETTINGS:
                success = self.grbl_comm.sync_settings()
                if not success:
                    pprint(f"{red('ERROR')}: GRBL settings sync failed. Ending program.")
                    return

            self.grbl_comm.status()

            if CUSTOM_HOMING_ON and CONNECT_TO_NANO: # queue up our theta and r homing sequence to be run first in control loop
                self.mode = HomingSequence()
            else:
                self.mode = self.modes_playlist[self.mode_index]
                if GRBL_HOMING_ON: # run GRBL's built in homing sequence (r homing only)
                    success = self.grbl_comm.grbl_home()
                    if not success:
                        pprint(f"{red('ERROR')}: GRBL's built-in homing sequence failed. Stopping GRBL, getting status, and ending program.")
                        self.grbl_comm.serial_port.write(bytes([0x18])) # no-checks immediate stop
                        self.grbl_comm.status()
                        return
                    self.grbl_comm.status()

        else:
            pprint("Not connecting to Arduino Uno.")

    def sense(self):
        pprint("Sensing...")
        self.state.phase = Phase.SENSE
        if CONNECT_TO_NANO:
            if self.mode.need_control_panel:
                pprint("Checking control panel...")
            else:
                pprint("Ignoring control panel.")
            
            # pprint(f">>>> mode.need_sensors: {self.mode.need_sensors}")
            if self.mode.need_sensors:
                pprint("Checking sensors...")
                self.nano_comm.update_state()
            else:
                pprint("Ignoring sensors.")
        else:
            pprint("Not connected to Arduino Nano - ignoring control panel and touch sensors.")
            if self.mode.need_control_panel or self.mode.need_sensors:
                pprint(f"Skipping mode: {self.mode.mode_name}")
                self.start_next_mode()
        
        if CONNECT_TO_UNO:
            if self.mode.need_grbl:
                self.grbl_comm.status()
                
                if LOG_PATH:
                    self.state.path_history.append([time.time(), self.state.grbl.mpos_r, self.state.grbl.mpos_t])
            else:
                pprint("Ignoring GRBL status.")
        else:
            pprint("Not connected to Arduino Uno - ignoring GRBL status.")
            if self.mode.need_grbl:
                pprint(f"Skipping mode: {self.mode.mode_name}")
                self.start_next_mode()

    def think(self):
        pprint("Thinking...")
        self.state.phase = Phase.THINK

        self.mode.update() # Allow mode to stay up to date even if next move is not needed this loop

        self.state.think_check_if_input_changed()

        self.state.think_check_if_buffer_space()
        
        # If the mode is done, cycle to next mode
        if (self.mode.is_done() 
            and self.state.grbl.status == GrblStatusOptions.IDLE.value 
            and self.state.grbl.planner_buffer == 15):
            pprint(f"Mode {self.mode.mode_name} is done.")
            self.start_next_mode()
        
        # If an input has changed, plan to send reset 
        # and calc next move from current position
        if self.state.flags.input_change:
            self.grbl_comm.need_reset = True
            self.grbl_comm.need_send_next_move = True
            pprint("Calculating next move from current position...")
            self.state.prev_move = Move(r=self.state.grbl.mpos_r, t_grbl=self.state.grbl.mpos_t, s=0, t=self.state.grbl.mpos_t % 360, received=None)
            self.state.next_move = self.mode.next_move(self.state.prev_move)
            if self.state.next_move.s == None:
                self.mode.set_next_speed()
            pprint(f"Next move: {self.state.next_move}")
        
        # If input has not changed and buffer is low, 
        # calc next move from last sent move. Otherwise, do nothing.
        elif self.state.flags.buffer_space:
            self.grbl_comm.need_send_next_move = True
            pprint("Calculating next move from last sent move...")
            self.state.next_move = self.mode.next_move(self.state.prev_move)
            self.mode.set_next_speed()
            pprint(f"Next move: {self.state.next_move}")

    def act(self):
        pprint("Acting...")
        self.state.phase = Phase.ACT
        if self.state.flags.run_control_loop and CONNECT_TO_UNO:
            if self.grbl_comm.need_send_next_move and LOG_COMMANDS:
                self.state.grbl_command_log.append([time.time(), self.state.next_move.r, self.state.next_move.t, self.state.next_move.s])
            self.grbl_comm.run()
        else:
            pprint("Not performing grbl tasks because run_control_loop is set to False.")

    def run(self):
        self.startup()
        self.state.iterate()
        while self.state.flags.run_control_loop:
            pprint(f"Loop Start --------------- moves sent: {self.state.moves_sent} | loop_count: {self.state.loop_count}")
            pprint(f"Current mode: {self.mode.mode_name}")
            pprint(self.mode)
            self.sense()
            self.think()
            self.act()
            self.state.iterate()
            time.sleep(self.state.loop_sleep_time)
        self.stop()


def main():
    signal.signal(signal.SIGINT, sig_handler)
    sandscape = Sandscape(modes_playlist=Mode.get_playlist_geometric_patterns())
    sandscape.run()

def sig_handler(sig, frame):
    if CONNECT_TO_UNO:
        pprint(f"Program terminated by user. No GRBL stop implemented at this time.")
    else:
        pprint(f"Program terminated by user. CONNECT_TO_UNO={CONNECT_TO_UNO} so no GRBL stop command required.")
    exit(0)

if __name__ == "__main__":
    main()