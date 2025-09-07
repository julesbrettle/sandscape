# type: ignore
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

from parse_svg import SVGParser, create_polar_plot, create_cartesian_plot
from utils import *
from state import *

@dataclass
class Mode:
    """
    Base class for different operational modes.
    Subclasses define specific behaviors and configurations.
    """
    state: Optional[State] = None
    mode_name: str = "base"  # Should be overridden by subclasses
    segment_length: int = 5
    base_linspeed: int = 9000 #mm/min
    r_dir: int = 1
    theta_dir: int = 1
    pitch: int = 13 
    waypoints_xy: list = field(default_factory=list)
    waypoints_rt: list = field(default_factory=list)
    waypoints_i: int = 0
    done: bool = False

    need_touch_sensors: bool = False
    need_control_panel: bool = False
    need_grbl: bool = True

    # do_sharp_compensation: bool = False
    # sharp_compensation_factor: float = DEFAULT_SHARP_COMPENSATION_FACTOR_MM
        
    def set_next_speed(self):
        if self.state.next_move.r != None and self.state.next_move.s == None:
            self.state.next_move.s = (-2*math.pi*self.state.next_move.r + self.base_linspeed + 360) * self.state.control_panel.speed
    
    def startup(self):
        """To be called once when the mode is started, for operations like path-precalculation or other initialization. Override in subclass if needed."""
        pass
    
    def next_move(self, move_from):
        """Calculate and return the next move for this mode. Calulate based on the move_from.r and move_from.t in the Move() object given, assuming that all other state parameters will stay the same."""
        print("Base next_move method called. Override in subclass.")
        return Move()

    def is_done(self):
        """
        Check if the mode is done if this mode has an end. Here in case any instantanious checks need to be made to determine if done.
        """
        return self.done
    
    def update(self):
        """
        To be called once per loop regardless if move is needed. 
        Override in subclass if mode needs to track position
        """
        pass

    def cleanup(self):
        """
        To be called after the mode is ended
        """
        pass

    @classmethod
    def get_playlist_geometric_patterns(self):
        return [
            SpiralMode(mode_name="spiral out"), 
            SVGMode(svg_file_name="pentagon_fractal"),
            SpiralMode(mode_name="spiral in", r_dir=-1),
            SpiralMode(mode_name="spiral out"), 
            SVGMode(svg_file_name="hex_gosper_d4"),
            SpiralMode(mode_name="spiral in", r_dir=-1),
            SVGMode(svg_file_name="dither_wormhole"),
            SpiralMode(mode_name="spiral in", r_dir=-1),
            SpiralMode(mode_name="spiral out"), 
            SVGMode(svg_file_name="hilbert_d5"),
            SpiralMode(mode_name="spiral in", r_dir=-1),
        ]

@dataclass
class Sleep(Mode):
    """Sleep main loop"""
    mode_name: str = "sleep"
    sleep_time: float = 1000000
    def next_move(self, move_from):
        time.sleep(self.sleep_time)
        return Move()

class Wait(Mode):
    mode_name: str = "wait"
    wait_time: float = 1000000
    start_time: float = 0.0
    def startup(self):
        self.start_time = time.time()
    def is_done(self):
        return time.time() - self.start_time > self.wait_time
    def next_move(self, move_from):
        return Move()

@dataclass
class HomingSequence(Mode):
    mode_name: str = "homing sequence"
    r_zeroing_done: bool = False
    t_zeroing_done: bool = False
    need_touch_sensors: bool = True

    def startup(self):
        self.state.flags.stop_on_theta_switch = True
        self.r_zeroing_done = False
        self.t_zeroing_done = False

    def cleanup(self):
        self.state.flags.stop_on_theta_switch = False

    def next_move(self, move_from : Move):
        print(cyan(f"Running HomingSequence.next_move(move_from={move_from})"))
        if self.state.limits_hit.hard_r_min:
            print(cyan(f"1 - self.state.limits_hit.hard_r_min={self.state.limits_hit.hard_r_min}"))
            self.r_zeroing_done = True
        if self.state.limits_hit.theta_zero:
            print(cyan(f"2 - self.state.limits_hit.theta_zero={self.state.limits_hit.theta_zero}"))
            self.t_zeroing_done = True
        if not self.r_zeroing_done:
            print(orange(f"4 - self.r_zeroing_done={self.r_zeroing_done}"))
            self.state.flags.need_homing = True
            move = Move(r=move_from.r-10, t=move_from.t, s=1000)
            print(cyan(f"    move: {move}"))
            return move
        if not self.t_zeroing_done:
            print(cyan(f"5 - self.t_zeroing_done={self.t_zeroing_done}"))
            move = Move(r=move_from.r, t=move_from.t-5.0, s=400)
            print(cyan(f"    move: {move}"))
            return move
        if self.r_zeroing_done and self.t_zeroing_done:
            print(cyan(f"6 - self.r_zeroing_done={self.r_zeroing_done} and self.t_zeroing_done={self.t_zeroing_done}"))
            if self.state.grbl.mpos_r != 0 or self.state.grbl.mpos_t != 0:
                print(cyan(f"7 - self.state.grbl.mpos_r={self.state.grbl.mpos_r} and self.state.grbl.mpos_t={self.state.grbl.mpos_t}"))
                self.state.flags.need_grbl_hard_reset = True
                return Move()
            else:
                print(cyan(f"8 - self.state.grbl.mpos_r={self.state.grbl.mpos_r} and self.state.grbl.mpos_t={self.state.grbl.mpos_t} -> done"))
                self.done = True
                return Move()



@dataclass
class SpiralMode(Mode):
    """Mode for the marble to draw a spiral outwards from its current location."""
    mode_name: str = "spiral"

        # Behavioral flags are inherited from Mode base.
        # Original `become_spiral` flags matched these defaults.
        
    def next_move(self, move_from):
        r = move_from.r
        theta = move_from.t
        if self.is_done():
            return Move()
        else:
            if r>30:
                seg_angle = 60 * self.segment_length/r*2*math.pi
            else:
                seg_angle = 60

            new_theta = theta + seg_angle*self.theta_dir
            new_r = r + self.pitch*seg_angle/360*self.r_dir

            # grbl_assumed_dist = math.sqrt((r-new_r)**2 + (theta-new_theta)**2)
            # compensation_ratio = grbl_assumed_dist/self.segment_length
            # new_speed = self.linspeed*compensation_ratio

            pi=math.pi
            r_ave = (r+new_r)/2
            new_speed = (-2*pi*r_ave + self.base_linspeed + 360) * self.state.control_panel.speed
            new_move = Move(r=new_r, t=new_theta, s=new_speed)
            if self.state.check_move(new_move):
                self.done = False
                return new_move
            else:
                self.done = True
                return Move()

    def is_done(self):
        if self.state.limits_hit.soft_r_max == True and self.r_dir == 1:
            self.done = True
        elif self.state.limits_hit.soft_r_min == True and self.r_dir == -1:
            self.done = True
        return self.done

SENSOR_THETA_MASK = [i*22.5 for i in range(16)]

@dataclass
class ReactiveOnlyDirectMode(Mode):
    """Mode where the marble only moves due to touch sensor activation.
    Marble goes directly towards hand, as opposed to in the cardinal direction
    of the touch."""
    mode_name: str = "reactive only"
    need_touch_sensors: bool = True

    def next_move(self, move_from):
        r = move_from.r
        theta = move_from.t

        speed = 10 # TODO: this is a placeholder

        selected_thetas = [j for i,j in zip(self.state.touch_sensors, SENSOR_THETA_MASK) if i==1]
        print(selected_thetas)
        if selected_thetas == []: # no touch, so no move
            return Move()
        
        avg_theta = sum(selected_thetas) / len(selected_thetas)
        r1, t1 = (280, avg_theta)
        print(r1, t1)

        x0, y0 = polar_to_cartesian_non_object(r, theta)
        x1, y1 = polar_to_cartesian_non_object(r1, t1)

        (dir_x, dir_y)  = ((x1 - x0), (y1 - y0))
        len_dir_vector = math.sqrt(dir_x**2 + dir_y**2)
        (x_to_travel, y_to_travel) = (dir_x*speed/len_dir_vector, dir_y*speed/len_dir_vector)
        (x_next, y_next) = (x0 + x_to_travel, y0 + y_to_travel)

        print(x_next, y_next)

        r_next, t_next = cartesian_to_polar_non_object(x_next, y_next)

        t_next = t_next % 360

        print(r_next, t_next)
        
        return Move(r=r_next, t=t_next, s=3000)

@dataclass
class ReactiveSpiralRippleMode(Mode):
    """
    Mode for the marble to draw a spiral that can be affected by touch sensor activation.
    (Behavioral flags for this mode were not fully defined in the original implementation)
    """
    mode_name: str = "reactive spiral ripple"

# @dataclass
# class SpikyBallMode(Mode):
#     mode_name: str = "spiky ball"
#     width_step: int = 4

#     def next_move(self, move_from):
#         # TODO: update when main loop handling of crossing 360 is updated
#         # this is all janky as fuck but doesn't require more previous steps and is continuous
#         if move_from.r == 0 and move_from.t % (self.width_step*2) == 0:
#             r_next = R_MAX 
#             t_next = move_from.t
#         if move_from.r != 0 and move_from.t % (self.width_step*2) == 0:
#             r_next = R_MAX 
#             t_next = move_from.t + self.width_step
#         if move_from.r != 0 and move_from.t % (self.width_step*2) != 0:
#             r_next = 0
#             t_next = move_from.t
#         if move_from.r == 0 and move_from.t % (self.width_step*2) != 0:
#             r_next = 0
#             t_next = move_from.t + self.width_step
        
#         return Move(r=r_next, t=t_next, s=4000)

@dataclass
class SVGMode(Mode):
    polar_pts: list[PolarPt] = field(default_factory=list)
    mode_name: str = "svg"
    svg_file_name: str = "hilbert_d6"
    pt_index: int = 0

    def __repr__(self):
        return f"SVGMode: {self.svg_file_name}"

    def startup(self):
        svg_parser = SVGParser()
        svg_file_path = self.get_svg_filepath()
        pts = svg_parser.get_pts_from_file(svg_file_path)
        pts = svg_parser.center(pts)
        self.polar_pts = svg_parser.convert_to_table_axes(pts)
        self.polar_pts = svg_parser.scale(self.polar_pts)
        # create_polar_plot(self.polar_pts)
        self.pt_index = 0
        
        # if we are already on the outside, go to the correct theta before starting the svg to avoid spiralling
        if self.state.grbl.mpos_r > R_MAX-2:
            first_t = self.polar_pts[0].t
            self.polar_pts.insert(0, PolarPt(r=self.state.grbl.mpos_r, t=first_t))
    
    def get_svg_filepath(self):
        return f"../svgs_production/{self.svg_file_name}.svg"
    
    def next_move(self, move_from):
        if self.pt_index >= len(self.polar_pts):
            self.done = True
            return Move()
        next_pt = self.polar_pts[self.pt_index]
        self.pt_index += 1
        return Move(r=next_pt.r, t=next_pt.t, s=2000)
