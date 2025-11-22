import serial
import time
import math
import threading
import queue
import json
import signal
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Union
from enum import Enum
import copy
import re

UNO_BAUD_RATE = 115200
NANO_BAUD_RATE = 9600

# --- SAND TABLE INFORMATION ---
TICKS_PER_MM_R = 40 # not used here, set in GRBL $100=40
TICKS_PER_DEG_THETA = 22.222 # not used here, set in GRBL $102=22.222
DISH_RADIUS_MM = 280
MARBLE_DIAMETER_MM = 14
MARBLE_WAKE_PITCH_MM = 13
R_MIN = 0
R_MAX = DISH_RADIUS_MM - MARBLE_DIAMETER_MM/2
DEFAULT_SHARP_COMPENSATION_FACTOR_MM = 5.0

def resume_color(text, color):
    # replace all instances of "\033[0m" with the target color
    return text.replace("\033[0m", color)

def red(text):
    color = "\033[91m"
    return f"{color}{resume_color(text, color)}\033[0m"

def orange(text):
    color = "\033[38;5;208m"
    return f"{color}{resume_color(text, color)}\033[0m"

def yellow(text):
    color = "\033[93m"
    return f"{color}{resume_color(text, color)}\033[0m"

def green(text):
    color = "\033[92m"
    return f"{color}{resume_color(text, color)}\033[0m"

def blue(text):
    color = "\033[94m"
    return f"{color}{resume_color(text, color)}\033[0m"

def purple(text):
    color = "\033[95m"
    return f"{color}{resume_color(text, color)}\033[0m"

def cyan(text):
    color = "\033[96m"
    return f"{color}{resume_color(text, color)}\033[0m"

def grey(text):
    color = "\033[90m"
    return f"{color}{resume_color(text, color)}\033[0m"

def print_error(text="Undefined error"):
    pprint(f"{red('ERROR')}: {text}")

def print_warning(text="Undefined warning"):
    pprint(f"{yellow('WARNING')}: {text}")

def pprint(x):
    """Print pretty with formatting and colors."""
    # bypass all formatting
    # print(x)
    # return

    string = str(x)

    # hide stuff I don't want to see rn
    HIDDEN_REP = blue("•")
    string = string.replace("curr_grbl_settings={0: 10.0, 1: 25.0, 2: 0.0, 3: 4.0, 4: 0.0, 5: 0.0, 6: 0.0, 10: 255.0, 11: 0.01, 12: 0.002, 13: 0.0, 20: 0.0, 21: 1.0, 22: 1.0, 23: 3.0, 24: 100.0, 25: 3000.0, 26: 250.0, 27: 8.0, 30: 1000.0, 31: 0.0, 32: 0.0, 100: 40.0, 101: 40.0, 102: 22.222, 110: 10000.0, 111: 10000.0, 112: 10000.0, 120: 50.0, 121: 50.0, 122: 10.0, 130: 550.0, 131: 345.0, 132: 200.0}", HIDDEN_REP)
    string = re.sub(r"port_name=.*?reader_thread=.*?>,\s*", HIDDEN_REP+", ", string, flags=re.DOTALL)
    string = string.replace("  control_panel=ControlPanel(speed=1, brightness=1),\n", HIDDEN_REP+",")
    string = string.replace("  touch_sensors=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\n", HIDDEN_REP+",")

    # collapse defaults
    string = string.replace("Move(r=None, t=None, s=None, t_grbl=None, received=False)", grey("Move()"))
    

    # color and shorten
    string = string.replace("True", green('T'))
    string = string.replace("False", red('F'))
    string = string.replace("None", blue('N'))

    
    print(string)

@dataclass
class Move:
    r: Optional[float] = None
    t: Optional[float] = None
    s: Optional[float] = None
    t_grbl: Optional[float] = None
    received: Optional[bool] = False

    def is_empty(self):
        if self.r == None and self.t == None and self.s == None:
            return True
        else:
            return False
        
    def __repr__(self):
        r_str = "None" if self.r == None else f"{self.r:.3f}"
        t_str = "None" if self.t == None else f"{self.t:.3f}"
        s_str = "None" if self.s == None else f"{self.s:.3f}"
        tgrbl_str = "None" if self.t_grbl == None else f"{self.t_grbl:.3f}"
        return f"Move(r={r_str}, t={t_str}, s={s_str}, t_grbl={tgrbl_str}, received={self.received})"
    
    @property
    def x(self):
        if self.r != None:
            if self.t_grbl != None:
                return polar_to_cartesian_non_object(self.r, self.t_grbl)[0]
            elif self.t != None:
                return polar_to_cartesian_non_object(self.r, self.t)[0]
            else: 
                print_error(f"{self} is not filled properly to return x.")
    
    @property
    def y(self):
        if self.r != None:
            if self.t_grbl != None:
                return polar_to_cartesian_non_object(self.r, self.t_grbl)[1]
            elif self.t != None:
                return polar_to_cartesian_non_object(self.r, self.t)[1]
            else: 
                print_error(f"{self} is not filled properly to return y.")

    @property
    def xy(self):
        return (self.x, self.y)
    @xy.setter
    def xy(self, new_xy):
        if new_xy[0] != None and new_xy[1] != None:
            self.r, self.t = cartesian_to_polar_non_object(new_xy[0], new_xy[1])

    @property
    def rt(self):
        return (self.r, self.t)
    @rt.setter
    def rt(self, new_rt):
        self.r = new_rt[0]
        self.t = new_rt[1]

@dataclass
class CartesianPt:
    x: float
    y: float
    def to_tuple(self):
        return (self.x, self.y)
    @property
    def r(self):
        return cartesian_to_polar_non_object(self.x, self.y)[0]
    @property
    def t(self):
        return cartesian_to_polar_non_object(self.x, self.y)[1]
    
    @property
    def xy(self):
        return (self.x, self.y)
    @xy.setter
    def xy(self, new_xy):
        self.x = new_xy[0]
        self.y = new_xy[1]

    @property
    def rt(self):
        return (self.r, self.t)
    @rt.setter
    def rt(self, new_rt):
        if self.r != None and self.t != None:
            x, y = polar_to_cartesian_non_object(new_rt[0], new_rt[1])
            self.x = x
            self.y = y
        else:
            print_error(f"Could not set rt in {self} with r={new_rt[0]} and t={new_rt[1]}.")

@dataclass
class PolarPt:
    r: float
    t: float
    def to_tuple(self):
        return (self.r, self.t)
    @property
    def x(self):
        return polar_to_cartesian_non_object(self.r, self.t)[0]
    @property
    def y(self):
        return polar_to_cartesian_non_object(self.r, self.t)[1]
    
    @property
    def xy(self):
        return (self.x, self.y)
    @xy.setter
    def xy(self, new_xy):
        if self.x != None and self.y != None:
            self.r, self.t = cartesian_to_polar_non_object(new_xy[0], new_xy[1])
        else:
            print_error(f"Could not set xy in {self} with x={new_xy[0]} and y={new_xy[1]}.")

    @property
    def rt(self):
        return (self.r, self.t)
    @rt.setter
    def rt(self, new_rt):
        self.r = new_rt[0]
        self.t = new_rt[1]

@dataclass
class Point:
    x: Optional[float] = None
    y: Optional[float] = None
    r: Optional[float] = None
    t: Optional[float] = None
    
    def __post_init__(self):
        if self.x != None and self.y != None:
            self.r, self.t = cartesian_to_polar_non_object(self.x, self.y)
        elif self.r != None and self.t != None:
            self.x, self.y = polar_to_cartesian_non_object(self.r, self.t)
        else:
            self.make_empty() # make sure parameters are not incorrectly set like Point(r=1,x=3)
    
    def is_empty(self):
        if (self.x == None or self.y == None) and (self.r == None or self.t == None):
            return True
        else:
            return False
    
    def make_empty(self):
        self.x = None
        self.y = None
        self.r = None
        self.t = None

    @property
    def xy(self):
        return (self.x, self.y)
    @xy.setter
    def xy(self, new_xy):
        self.x = new_xy[0]
        self.y = new_xy[1]
        if self.x != None and self.y != None:
            self.r, self.t = cartesian_to_polar_non_object(new_xy[0], new_xy[1])
        else:
            print_error(f"Could not set xy in {self} with x={new_xy[0]} and y={new_xy[1]}.")

    @property
    def rt(self):
        return (self.r, self.t)
    @rt.setter
    def rt(self, new_rt):
        self.r = new_rt[0]
        self.t = new_rt[1]
        if self.r != None and self.t != None:
            self.x, self.y = polar_to_cartesian_non_object(new_rt[0], new_rt[1])
        else:
            print_error(f"Could not set rt in {self} with r={new_rt[0]} and t={new_rt[1]}.")

def move_to_point(move: Move):
    if move.r != None and move.t_grbl != None:
        point = Point(r=move.r, t=move.t_grbl)
    elif move.r != None and move.t != None:
        point = Point(r=move.r, t=move.t)
    else:
        point = Point()
    return point

def point_to_move(point: Point):
    return Move(r=point.r, t=point.t)

def normalize_vector(x, y):
    length = math.sqrt(x**2 + y**2)
    if length == 0:
        return [0, 0]
    return [x/length, y/length]

def get_direction(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return normalize_vector(dx, dy)

def cartesian_to_polar_non_object(x, y):
    r = math.sqrt(x**2 + y**2)
    t = math.atan2(y, x)*180/math.pi
    return float(r), float(t)

def polar_to_cartesian_non_object(r, t):
    t = t*math.pi/180
    x = r * math.cos(t)
    y = r * math.sin(t)
    return float(x), float(y)

def sharp_compensate(new_point, prev_point, correction_factor_mm=DEFAULT_SHARP_COMPENSATION_FACTOR_MM):
    """
    Create sharp corners on lagging path by adjusting new_point by correction_factor_mm in the direction of prev_point.
    Args:
        new_point: a Move, Point, or other object with a fetchable x and y attribute and an xy.setter function.
        prev_point: a Move, Point, or other object with a fetchable x and y attribute.

    """
    direction = get_direction(prev_point.x, prev_point.y, new_point.x, new_point.y)
    offset_x = direction[0] * correction_factor_mm
    offset_y = direction[1] * correction_factor_mm
    return_point = copy.deepcopy(new_point)
    return_point.xy = (new_point.x + offset_x, new_point.y + offset_y)
    return return_point

def sharp_compensate_all(pts, correction_factor_mm=DEFAULT_SHARP_COMPENSATION_FACTOR_MM):
    compensated_pts = [pts[0]]
    for i in range(1, len(pts)):
        compensated_pts.extend(sharp_compensate(pts[i], pts[i-1], correction_factor_mm))
    return compensated_pts

@dataclass
class SerialCommunicator:
    port_name: str
    baud_rate: int
    serial_port: serial.Serial = field(default_factory=lambda: serial.Serial())
    data_queue: queue.Queue = field(default_factory=lambda: queue.Queue())
    stop_event: threading.Event = field(default_factory=lambda: threading.Event())
    reader_thread: threading.Thread = field(default_factory=lambda: threading.Thread())
    connected: bool = False
    display_name: str = "<untitled device>"
    print_header: bool = True

    def serial_connect(self, do_ping=True):
        pprint(f"Establishing serial connection to {self.display_name}...")
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            time.sleep(2) # Wait for connection to establish!
            self.data_queue = queue.Queue()
            self.stop_event = threading.Event()
            self.reader_thread = threading.Thread(target=self.read_from_port)
            self.reader_thread.daemon = True
            self.reader_thread.start()
            time.sleep(2)
            if do_ping:
                success = self.ping()
                if success == None:
                    print_warning(f"    Ping is not implemented for {self.display_name}. Connection assumed successful.")
                    self.connected = True
                elif success == True:
                    pprint(f"    Ping successful for {self.display_name}.")
                    self.connected = True
                else:
                    print_error(f"    Ping failed for {self.display_name}.")
                    self.connected = False
                    return False
                pprint(f"    Serial connection to {self.display_name} established.")
                return True
            else:
                pprint(f"    Skipping ping for {self.display_name}.")
        except Exception as e:
            print_error(f"    Failed to establish serial connection to {self.display_name}. {e}")
            return False

    def serial_disconnect(self):
        pprint(f"Ending serial connection to {self.display_name}...")
        self.stop_event.set()
        self.reader_thread.join()
        self.serial_port.close()
        pprint(f"    Serial connection to {self.display_name} ended.")

    def ping(self) -> bool: # pyright: ignore[reportReturnType]
        pass

    def read_from_port(self):
        """
        Reads data from the serial port line by line in a dedicated thread.

        Args:
            ser (serial.Serial): The initialized serial port object.
            stop_event (threading.Event): The event to signal the thread to stop.
        """
        last_msg_timestamp = time.time()
        time_since_last_msg = time.time() - last_msg_timestamp
        last_msg_timestamp = time.time()
        pprint("Reader thread started.")
        while not self.stop_event.is_set():
            try:
                # The readline() function will block until a newline character
                # is received, or until the timeout (set during port initialization)
                # is reached.
                line = self.serial_port.readline()

                # If a line was actually read (i.e., not a timeout)
                if line:
                    # Decode the bytes into a string, using UTF-8 encoding.
                    # 'errors='ignore'' will prevent crashes on decoding errors.
                    # .strip() removes leading/trailing whitespace, including the newline.
                    time_since_last_msg = time.time() - last_msg_timestamp
                    last_msg_timestamp = time.time()
                    decoded_line = line.decode('utf-8', errors='ignore').strip()
                    print_str = ""
                    if self.print_header:
                        print_str += f"{time.time():.3f} | "
                        print_str += f"{time_since_last_msg:.3f}s | "
                        print_str += f"Received"
                        if self.display_name != "":
                            print_str += f" from {self.display_name}: "
                        else:
                            print_str += ": "
                    print_str += decoded_line
                    if len(decoded_line) == 17 and int(decoded_line[16]) == 1:
                        pprint(green(print_str))
                    else: pprint(purple(print_str))
                    if len(decoded_line) > 0:
                        self.data_queue.put(decoded_line)

            except serial.SerialException as e:
                # Handle cases where the serial port is disconnected or an error occurs
                print_error(f"Serial port error: {e}. Stopping thread.")
                exit()
                break
            except Exception as e:
                # Handle other potential exceptions
                print_error(f"An unexpected error occurred: {e}")
                if not self.stop_event.is_set():
                    # Avoid flooding the console with error messages
                    time.sleep(1)
                exit()

        pprint("Reader thread finished.")