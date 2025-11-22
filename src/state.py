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

from utils import *

class Phase(Enum):
    SETUP = "setup"
    SENSE = "sense"
    THINK = "think"
    ACT = "act"
    ITERATE = "iterate"

@dataclass
class LimitsHit:
    soft_r_min: bool = True
    soft_r_max: bool = True
    hard_r_min: bool = True
    hard_r_max: bool = True
    theta_zero: bool = False

@dataclass
class ControlPanel:
    speed: float = 1
    brightness: float = 1

@dataclass
class Grbl:
    status: str = ""
    mpos_r: float = 0
    mpos_t: float = 0
    feed_rate: float = 0
    planner_buffer: int = 15
    rx_buffer: float = 128
    pnX: bool = False
    pnY: bool = False

@dataclass
class Flags:
    # flags that reset per loop
    input_change: bool = True
    buffer_space: bool = False
    need_homing: bool = False
    need_grbl_hard_reset: bool = False
    stop_on_theta_switch: bool = False
    need_calc_next_move: bool = False
    run_control_loop: bool = True
    sharp_compensation_on: bool = False

@dataclass
class State:
    phase: Phase = Phase.SETUP
    limits_hit: LimitsHit = field(default_factory=lambda: LimitsHit())
    control_panel: ControlPanel = field(default_factory=lambda: ControlPanel())
    touch_sensors: list = field(default_factory=list)
    grbl: Grbl = field(default_factory=lambda: Grbl())
    flags: Flags = field(default_factory=lambda: Flags())
    next_move: Move = field(default_factory=lambda: Move(r=0,t=0))

    prev_limits_hit: LimitsHit = field(default_factory=lambda: LimitsHit())
    prev_control_panel: ControlPanel = field(default_factory=lambda: ControlPanel())
    prev_touch_sensors: list = field(default_factory=list)
    prev_grbl: Grbl = field(default_factory=lambda: Grbl())
    prev_flags: Flags = field(default_factory=lambda: Flags())
    prev_move: Move = field(default_factory=lambda: Move(r=0,t=0))
    # prev_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())
    desired_linspeed: int = 3000 #mm/min
    moves_sent: int = 0
    loop_sleep_time: float = 0.1
    loop_count: int = 0
    sharp_compensation_factor: float = DEFAULT_SHARP_COMPENSATION_FACTOR_MM

    # last_grbl_resp: GrblRespMsg = field(default_factory=lambda: GrblRespMsg())
    # next_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())

    theta_correction: float = 0 # TODO
    path_history: list = field(default_factory=list)
    grbl_command_log: list = field(default_factory=list)
    # curr_grbl_settings: dict = field(default_factory=dict)
        
    def __post_init__(self):
        self.phase = Phase.SETUP
        self.flags.run_control_loop = True
        self.prev_limits_hit.soft_r_min = False
        self.prev_move = Move(r=0,t=0,t_grbl=0)
        self.next_move = Move(r=0,t=0,t_grbl=0)
    
    def __repr__(self):
        return (
            f"State(\n"
            f"  phase={self.phase},\n"
            f"  limits_hit={self.limits_hit},\n"
            f"  control_panel={self.control_panel},\n"
            f"  touch_sensors={self.touch_sensors},\n"
            f"  grbl={self.grbl},\n"
            f"  flags={self.flags},\n"
            f"  next_move={self.next_move},\n"
            f"  prev_limits_hit={self.prev_limits_hit},\n"
            f"  prev_control_panel={self.prev_control_panel},\n"
            f"  prev_grbl={self.prev_grbl},\n"
            f"  prev_flags={self.prev_flags},\n"
            f"  prev_move={self.prev_move},\n"
            # f"  desired_linspeed={self.desired_linspeed},\n"
            # f"  moves_sent={self.moves_sent},\n"
            # f"  last_grbl_resp={self.last_grbl_resp},\n"
            # f"  next_grbl_msg={self.next_grbl_msg}\n"
            # f"  path_history={self.path_history},\n"
            # f"  grbl_command_log={self.grbl_command_log},\n"
            # f"  sharp_compensation_factor={self.sharp_compensation_factor},\n"
            f")"
        )

    def iterate(self):
        self.phase = Phase.ITERATE
        self.prev_limits_hit = copy.deepcopy(self.limits_hit)
        self.prev_control_panel = copy.deepcopy(self.control_panel)
        self.prev_touch_sensors = copy.deepcopy(self.touch_sensors)
        self.prev_grbl = copy.deepcopy(self.grbl)
        self.prev_flags = copy.deepcopy(self.flags)
        if self.next_move.received:
            self.prev_move = copy.deepcopy(self.next_move)
        self.flags.input_change = False
        self.flags.buffer_space = False
        self.loop_count += 1

    def think_check_if_input_changed(self):
        # Check if input_change has already been set by mode.update()
        if self.flags.input_change:
            pprint(orange(f"INPUT CHANGE before think_check_if_input_changed()"))

        # Check if limits_hit have just been hit
        if self.prev_limits_hit != self.limits_hit:
            if self.flags.stop_on_theta_switch and self.limits_hit.theta_zero:
                self.flags.input_change = True
            if self.limits_hit.soft_r_min or self.limits_hit.soft_r_max:
                self.flags.input_change = True
            if self.limits_hit.hard_r_min or self.limits_hit.hard_r_max:
                self.flags.input_change = True
                self.flags.need_homing = True
        
        # Check if input (sensors or dials) has changed
        if (self.prev_control_panel != self.control_panel 
            and self.prev_touch_sensors != self.touch_sensors):
            self.flags.input_change = True
        
        if self.flags.input_change:
            pprint(orange(f"INPUT CHANGE!"))

    def think_check_if_buffer_space(self):
        # Check if grbl's buffer has space
        if self.grbl.planner_buffer >= 1:
            # PlannerBuffer is 0 when full, 15 when empty
            # RxBuffer is 0 when full, 128 when empty
            self.flags.buffer_space = True

    def check_move(self, move):
        """Check move validity based on limits and grbl status."""
        # If r limit has been hit, next move needs to be in oppsite direction
        move.r = round(move.r, 3)
        move.t = round(move.t, 3)

        pprint(f"Checking move {move}...")
        if self.limits_hit.hard_r_min and move.r < self.grbl.mpos_r:
            print_warning(f"Requested move {move} is into the hard_r_min limit switch.")
            return False
        if self.limits_hit.hard_r_max and move.r > self.grbl.mpos_r:
            print_warning(f"Requested move {move} is into the hard_r_max limit switch.")
            return False

        if not self.flags.need_homing and move.r != self.grbl.mpos_r: # Assuming switches have not been hit and position can be trusted
            if self.limits_hit.soft_r_min and move.r < self.grbl.mpos_r:
                print_warning(f"Requested move {move} is into the soft_r_min limit switch.")
                return False
            elif self.limits_hit.soft_r_max and move.r > self.grbl.mpos_r:
                print_warning(f"Requested move {move} is into the soft_r_max limit switch.")
                return False
            elif move.r > R_MAX or move.r < R_MIN:
                print_warning(f"Requested move {move} is out of bounds.")
                return False
            
        if move.t == None and move.t_grbl == None:
            print_warning(f"Requested move {move} is not filled correctly.")
            return False
        
        pprint(f"Checks passed.")
        return True