# type: ignore
import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
import re
from matplotlib.animation import FuncAnimation
import bezier
import numpy as np
import math
from utils import *

SEG_LENGTH = 3

class SVGParser:
    # Class attribute for curve types configuration
    curve_types_to_expected_length = {
        'm': (2, False),
        'M': (2, False),
        'l': (2, True),
        'L': (2, False),
        'h': (1, False),
        'H': (1, True),
        'v': (1, False),
        'V': (1, True),
        'c': (6, True)
    }

    @dataclass
    class Curve:
        marker: str
        body: List[int]

        def __post_init__(self):
            # Access the parent class's curve_types_to_expected_length
            expected_length = SVGParser.curve_types_to_expected_length[self.marker][0]
            multiples_expected = SVGParser.curve_types_to_expected_length[self.marker][1]

            if multiples_expected:
                if not (len(self.body)%expected_length) == 0:
                    raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")
            else: 
                if not len(self.body) == expected_length:
                    raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")

    def __init__(self):
        #TODO: maybe encoded curve types and metadata in a better object than this - include description of what the curve actually is/does
        # tuple: (expected_length, multiples_expected)
        self.parsing_inkscape_file = True

    def split_raw_curves(self, raw):
        split_at_letter = re.findall(r'[a-zA-Z][^a-zA-Z]*', raw)
        curves = []

        for curve_block in split_at_letter:
            marker = curve_block[0]

            if marker == 'z': #discard end markers
                continue
            elif marker not in self.curve_types_to_expected_length:
                raise RuntimeError(f"Encountered unsupported curve type in SVG: {marker}")

            body = [float(x) for x in re.findall(r'-?\d+\.?\d*', curve_block[1:])]

            curve_length = self.curve_types_to_expected_length[marker][0] 
            single_curves = [body[i:i + curve_length] for i in range(0, len(body), curve_length)]
            for single_curve in single_curves:
                curves.append(self.Curve(
                    marker=marker,
                    body=single_curve
                ))
        return curves

    def discretize_bezier(self, node_set, prev_pt):
        nodes = np.asfortranarray([
            [prev_pt.x, prev_pt.x+node_set[0], prev_pt.x+node_set[2], prev_pt.x+node_set[4]],
            [prev_pt.y, prev_pt.y+node_set[1], prev_pt.y+node_set[3], prev_pt.y+node_set[5]],
        ])
        bezier_curve_obj = bezier.Curve(nodes, degree=3)

        num_pts_in_curve = bezier_curve_obj.length / SEG_LENGTH
        s_vals = np.linspace(0.0, 1.0, math.ceil(num_pts_in_curve))
        evaluator_output = bezier_curve_obj.evaluate_multi(s_vals)
        
        pts = [CartesianPt(x=float(x), y=float(y)) for x, y in zip(evaluator_output[0], evaluator_output[1])]
        
        return pts
    
    def get_dist(self, p0, p1):
        return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

    def interpolate_single(self, p0, p1):
        total_distance = self.get_dist(p0, p1)
        num_pts = math.floor(total_distance / SEG_LENGTH)
        
        pts_to_add = []
        # Add the interpolated pts
        for j in range(1, num_pts + 1):
            t = j * SEG_LENGTH / total_distance
            x = p0.x + t * (p1.x - p0.x)
            y = p0.y + t * (p1.y - p0.y)
            pts_to_add.append(CartesianPt(x, y))
        
        pts_to_add.append(p1)
        return pts_to_add

    def parse_multiple_curves(self, curves_raw):
        curves = self.split_raw_curves(curves_raw)
        
        pts = [CartesianPt(x=0, y=0)]
        first_pt = True
        for curve in curves:
            prev_pt = pts[-1]
            if curve.marker=='M' or curve.marker=="L": # moveto, lineto (absolute)
                # pts.append(CartesianPt(x=curve.body[0], y=curve.body[1])) # TODO: what is getting an interpolated line is improperly mapped
                pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=curve.body[0], y=curve.body[1])))
            elif curve.marker=='m' or curve.marker=="l": # moveto, lineto (relative)
                if first_pt:
                    pts.append(CartesianPt(x=curve.body[0], y=curve.body[1]))
                else:
                    pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=prev_pt.x+curve.body[0], y=prev_pt.y+curve.body[1])))
            elif curve.marker=='h': # horizontal line
                pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=prev_pt.x+curve.body[0], y=prev_pt.y)))
            elif curve.marker=='H': # horizontal line
                pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=curve.body[0], y=prev_pt.y)))
            elif curve.marker=='V': # vertical line, absolute
                pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=prev_pt.x, y=curve.body[0])))
            elif curve.marker=='v': # vertical line
                pts.extend(self.interpolate_single(prev_pt, CartesianPt(x=prev_pt.x, y=prev_pt.y+curve.body[0])))
            elif curve.marker=='c': # bezier curve
                pts.extend(self.discretize_bezier(curve.body, prev_pt))
            elif curve.marker=='z': # end of curve
                pass
            else:
                print(f"Encountered unexpected curve marker: {curve.marker}")
            
            if first_pt:
                first_pt = False
        
        return pts[1:] # remove artificial (0,0) point
            

    def get_layer_above_meat(self,layer):

        if self.parsing_inkscape_file:
            if layer.tag == "{http://www.w3.org/2000/svg}svg":
                for child in layer:
                    if child.tag == "{http://www.w3.org/2000/svg}g":
                        return child
            raise RuntimeError("Inkscape file not in expected format")
        
        else: 
            print(f"current layer.attrib: {layer.attrib}")
            if layer.attrib == {}:
                return None

            for child in layer:
                print(f"evaluating child: {child.attrib}")
                if child.attrib.get('d', None):
                    print("it's this one!")
                    return layer

            for child in layer:
                layer_below_evaluation = self.get_layer_above_meat(child)
                if layer_below_evaluation:
                    return layer_below_evaluation
            
            raise RuntimeError("oops, shouldn't have gotten here")

    def get_pts_from_file(self, file):
        tree = et.parse(file)
        root = tree.getroot()
        if root.attrib['version'] != '1.1':
            print(f"Warning! Unsupported svg version: {root.attrib['version']}")

        layer_above_meat = self.get_layer_above_meat(root)
        print(layer_above_meat.attrib)

        all_pts = []
        for child in layer_above_meat:
            curves_raw = child.attrib['d']
            all_pts.extend(self.parse_multiple_curves(curves_raw))
        
        return all_pts
    
    def convert_to_table_axes(self, pts):
        '''
        - flip y axis
        - convert to polar
        '''
        converted_pts = []
        for pt in pts:
            x = pt.x
            y = -1*pt.y
            polar_pt = self.cartesian_to_polar(CartesianPt(x=x, y=y))
            converted_pts.append(polar_pt)
        return converted_pts
    
    def scale_and_center(self, pts):
        min_x, max_x = min(pt.x for pt in pts), max(pt.x for pt in pts)
        min_y, max_y = min(pt.y for pt in pts), max(pt.y for pt in pts)
        
        width, height = max_x - min_x, max_y - min_y
        max_size = 546 - 2 * 5 # 5 = margin
        scale = min(max_size / width if width > 0 else 1, max_size / height if height > 0 else 1)
        
        center_x, center_y = 273, 273
        current_center_x, current_center_y = (min_x + max_x) / 2, (min_y + max_y) / 2
        
        scaled_pts = [CartesianPt(
            x=(pt.x - current_center_x) * scale + center_x,
            y=(pt.y - current_center_y) * scale + center_y
        ) for pt in pts]
        
        return scaled_pts
    
    def scale(self, pts):
        desired_max_r = 273 - 5
        max_r = max(pt.r for pt in pts)

        scaled_pts = [PolarPt(
            r=pt.r * desired_max_r / max_r,
            t=pt.t
        ) for pt in pts]

        return scaled_pts

    def center(self, pts):
        min_x, max_x = min(pt.x for pt in pts), max(pt.x for pt in pts)
        min_y, max_y = min(pt.y for pt in pts), max(pt.y for pt in pts)

        center_x, center_y = (min_x + max_x) / 2, (min_y + max_y) / 2

        centered_pts = [CartesianPt(
            x=pt.x - center_x,
            y=pt.y - center_y
        ) for pt in pts]

        return centered_pts
    
    def cartesian_to_polar(self, pt: CartesianPt) -> PolarPt:
        r = math.sqrt(pt.x**2 + pt.y**2)
        t = math.atan2(pt.y, pt.x)*180/math.pi % 360
        return PolarPt(float(r), float(t))

# def create_cartesian_plot(pts): 
#     pts_decoded = [pt.to_tuple() for pt in pts]
#     # print(pts_decoded)
#     x_coords = [pt[0] for pt in pts_decoded]
#     y_coords = [pt[1] for pt in pts_decoded]
#     plt.figure()
#     plt.plot(x_coords, y_coords, 'k-')  # 'b-' means blue line
#     # plt.scatter(x_coords, y_coords, c='red', s=5)  # Add points as red dots
#     # plt.plot(x_coords, y_coords, ".-", c='red', markersize=10)
#     plt.scatter(x_coords, y_coords, c=np.linspace(0,1,len(x_coords)), s=20)
#     plt.set_cmap("gist_rainbow") 
#     plt.gca().yaxis.set_inverted(True)
#     plt.grid(True)
#     plt.axis('equal')
#     plt.title('SVG Path Visualization')
#     plt.show()

def create_cartesian_plot(pts, pts2=None, highlight_pt=None):
    plt.figure(figsize=(8, 8))
    
    # Create rainbow color gradient
    num_pts = len(pts)
    colors = plt.cm.rainbow(np.linspace(1, 0, num_pts))
    
    # Plot each segment with its own color
    for i in range(len(pts)-1):
        xs = [pts[i].x, pts[i+1].x]
        ys = [pts[i].y, pts[i+1].y]
        plt.plot(xs, ys, color=colors[i], linewidth=2)
    
    # Plot individual pts with their colors
    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    plt.scatter(xs, ys, c=colors, s=30, zorder=5)

    if pts2 != None:
        # Plot each segment with its own color
        for i in range(len(pts)-1):
            xs = [pts2[i].x, pts2[i+1].x]
            ys = [pts2[i].y, pts2[i+1].y]
            plt.plot(xs, ys, "o-k", linewidth=1)
        
        # Plot individual pts with their colors
        xs = [p.x for p in pts2]
        ys = [p.y for p in pts2]
        # plt.scatter(xs, ys, "k") #, c=colors, s=15, zorder=5)

    
    if highlight_pt != None:
        # Highlight the specified point
        plt.scatter(highlight_pt.x, highlight_pt.y, c='red', s=100, zorder=10, 
                    edgecolors='black', linewidth=2, marker='o')
    else:
        plt.scatter(pts[-1].x, pts[-1].y, c='red', s=100, zorder=10, 
                    edgecolors='black', linewidth=2, marker='o')
    
    # Add arrows to show direction
    for i in range(len(pts)-1):
        mid_x = (pts[i].x + pts[i+1].x) / 2
        mid_y = (pts[i].y + pts[i+1].y) / 2
        
        # Calculate the direction vector
        dx = pts[i+1].x - pts[i].x
        dy = pts[i+1].y - pts[i].y
        
        # Normalize the direction vector
        dx, dy = normalize_vector(dx, dy)
        
        # Plot the arrow
        plt.arrow(mid_x - dx*2, mid_y - dy*2, 
                 dx*4, dy*4,
                 head_width=.5, head_length=.5, fc='blue', ec='blue')
    
    plt.grid(True)
    plt.axis('equal')  # This ensures the plot is circular
    plt.title('Path in Cartesian Coordinates')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def animate_cartesian_plot(pts, pts2=None):
    fig, ax = plt.subplots(figsize=(8, 8))
    
    all_pts = pts + (pts2 if pts2 is not None else [])
    if not all_pts:
        return

    min_x = min(p.x for p in all_pts)
    max_x = max(p.x for p in all_pts)
    min_y = min(p.y for p in all_pts)
    max_y = max(p.y for p in all_pts)
    
    ax.set_xlim(min_x - 10, max_x + 10)
    ax.set_ylim(min_y - 10, max_y + 10)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.set_title('Animated Path in Cartesian Coordinates')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    num_repeats = 30
    colors = plt.cm.rainbow(np.linspace(num_repeats, 0, len(pts)) % 1)
    head, = ax.plot([], [], 'ro')

    def init():
        # Clear previous lines
        for l in ax.get_lines():
            if l != head:
                l.remove()
        head.set_data([], [])
        return [head]

    def update(frame):
        if frame > 0:
            x_segment = [pts[frame-1].x, pts[frame].x]
            y_segment = [pts[frame-1].y, pts[frame].y]
            ax.plot(x_segment, y_segment, color=colors[frame], linewidth=2)
        
        head.set_data([pts[frame].x], [pts[frame].y])
        return [head] + ax.get_lines()

    ani = FuncAnimation(fig, update, frames=len(pts),
                        init_func=init, blit=False, interval=100, repeat=False)
    plt.show()

def animate_polar_plot(pts, pts2=None):
    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw={'projection': 'polar'})

    all_pts = pts + (pts2 if pts2 is not None else [])
    if not all_pts:
        return

    max_r = max(p.r for p in all_pts) if all_pts else 1
    ax.set_rmax(max_r * 1.1)
    ax.grid(True)
    ax.set_title('Animated Path in Polar Coordinates')

    num_repeats = 30
    colors = plt.cm.rainbow(np.linspace(num_repeats, 0, len(pts)) % 1)
    head, = ax.plot([], [], 'ro')

    def init():
        # Clear previous lines
        for l in ax.get_lines():
            if l != head:
                l.remove()
        head.set_data([], [])
        return [head]

    def update(frame):
        if frame > 0:
            theta_segment = [pts[frame-1].t * np.pi / 180, pts[frame].t * np.pi / 180]
            r_segment = [pts[frame-1].r, pts[frame].r]
            ax.plot(theta_segment, r_segment, color=colors[frame], linewidth=2)

        head.set_data([pts[frame].t * np.pi / 180], [pts[frame].r])
        return [head] + ax.get_lines()

    ani = FuncAnimation(fig, update, frames=len(pts),
                        init_func=init, blit=False, interval=100, repeat=False)
    plt.show()


def create_polar_plot(pts: List[PolarPt]):
    # Convert theta from degrees to radians for plotting
    ts_rad = [p.t * math.pi/180 for p in pts]
    rs = [p.r for p in pts]

    # Create polar plot
    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, projection='polar')
    
    # Create rainbow color gradient based on point position
    colors = plt.cm.rainbow(np.linspace(0, 1, len(pts)))
    
    # Plot each segment with its corresponding color
    for i in range(len(pts) - 1):
        ax.plot([ts_rad[i], ts_rad[i+1]], [rs[i], rs[i+1]], color=colors[i], linewidth=2)
    
    # Plot points with rainbow colors
    scatter = ax.scatter(ts_rad, rs, c=np.linspace(0, 1, len(pts)), cmap='rainbow', s=30)
    
    ax.set_rmax(300)  # Set maximum radius to 300
    ax.set_rticks([0, 100, 200, 300])  # Set radius ticks
    ax.set_thetagrids(np.arange(0, 360, 45))  # Set theta grid lines every 45 degrees
    ax.grid(True)
    ax.set_title('Path in Polar Coordinates (Rainbow)')
    plt.show()


if __name__ == "__main__":
    # svg_file = "../svgs_production/dither_cells_2.svg"
    # svg_file = "../svgs_production/pentagon_fractal.svg"
    # svg_file = "../svgs_production/hex_gosper_d4.svg"
    # svg_file = "../svgs_production/dither_wormhole.svg"
    svg_file = "../svgs_production/hilbert_d5.svg"
    svg_parser = SVGParser()
    pts = svg_parser.get_pts_from_file(svg_file)
    pts = svg_parser.center(pts)
    polar_pts = svg_parser.convert_to_table_axes(pts)
    polar_pts = svg_parser.scale(polar_pts)

    # create_cartesian_plot(pts)
    adjusted_points = sharp_compensate_all(polar_pts)
    # print(polar_pts)
    # create_cartesian_plot(polar_pts, adjusted_points)
    # create_cartesian_plot(adjusted_points, polar_pts)
    animate_cartesian_plot(adjusted_points, polar_pts)
    animate_polar_plot(adjusted_points, polar_pts)
    # create_polar_plot(polar_pts)