# type: ignore
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
from utils import *
from svgpathtools import svg2paths, svg2paths2

SEG_LENGTH = 3

class SVGParser:
    def __init__(self):
        self.viewbox = None

    def get_artboard_size(self, file_path):
        paths, path_attributes, svg_attributes = svg2paths2(file_path)
        
        viewbox_string = svg_attributes.get('viewBox')
        if viewbox_string:
            parts = viewbox_string.split()
            if len(parts) == 4:
                vb_width = float(parts[2])
                vb_height = float(parts[3])
                return vb_width, vb_height
        
        return 546.0, 546.0

    def get_pts_from_file(self, file_path):
        paths, attributes = svg2paths(file_path)
        width, height = self.get_artboard_size(file_path)
        adjusted_seg_length = SEG_LENGTH/(DISH_RADIUS_MM*2) * width

        all_xy_coordinates = []

        for path in paths:
            path_coords = []
            
            length = path.length()
            if length == 0:
                continue
                
            samples_per_path = max(2, math.ceil(length / adjusted_seg_length))
            
            if samples_per_path <= 1:
                points_t = [0.0]
            else:
                points_t = [i / float(samples_per_path - 1) for i in range(samples_per_path)]

            for t in points_t:
                point = path.point(t)
                x, y = point.real, point.imag
                path_coords.append((x, y))
            
            all_xy_coordinates.append(path_coords)

        scaled_coordinates = []
        for path_coords in all_xy_coordinates:
            for x, y in path_coords:
                new_x = (x / width) * (2 * DISH_RADIUS_MM) - DISH_RADIUS_MM
                new_y = -1*((y / height) * (2 * DISH_RADIUS_MM) - DISH_RADIUS_MM)
                scaled_coordinates.append(CartesianPt(x=new_x, y=new_y))

        return scaled_coordinates
    
    def convert_to_table_axes(self, pts):
        '''
        - convert to polar
        '''
        converted_pts = []
        for pt in pts:
            polar_pt = self.cartesian_to_polar(pt)
            converted_pts.append(polar_pt)
        return converted_pts
    
    def scale_and_center(self, pts):
        return pts
    
    def scale(self, pts):
        return pts

    def center(self, pts):
        return pts
    
    def cartesian_to_polar(self, pt: CartesianPt) -> PolarPt:
        r = math.sqrt(pt.x**2 + pt.y**2)
        t = math.atan2(pt.y, pt.x)*180/math.pi % 360
        return PolarPt(float(r), float(t))

def create_cartesian_plot(pts, pts2=None, highlight_pt=None):
    plt.figure(figsize=(8, 8))

    # Add a grey circle representing the dish
    circle = plt.Circle((0, 0), DISH_RADIUS_MM, color='grey', fill=False, linestyle='--', zorder=0)
    plt.gca().add_patch(circle)
    
    
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
        for i in range(len(pts2)-1):
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
    plt.xlim(-DISH_RADIUS_MM - 10, DISH_RADIUS_MM + 10)
    plt.ylim(-DISH_RADIUS_MM - 10, DISH_RADIUS_MM + 10)
    plt.axis('equal')  # This ensures the plot is circular
    plt.title('Path in Cartesian Coordinates')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def animate_cartesian_plot(pts, pts2=None):
    fig, ax = plt.subplots(figsize=(8, 8))

    min_x = min(min(p.x for p in pts), -DISH_RADIUS_MM)
    max_x = max(max(p.x for p in pts), DISH_RADIUS_MM)
    min_y = min(min(p.y for p in pts), -DISH_RADIUS_MM)
    max_y = max(max(p.y for p in pts), DISH_RADIUS_MM)
    
    ax.set_xlim(min_x - 10, max_x + 10)
    ax.set_ylim(min_y - 10, max_y + 10)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    # Add a grey circle representing the dish
    circle = plt.Circle((0, 0), DISH_RADIUS_MM, color='grey', fill=False, linestyle='--', zorder=0)
    ax.add_patch(circle)

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
        
        if pts2 != None:
            # Plot each segment
            for i in range(len(pts2)-1):
                xs = [pts2[i].x, pts2[i+1].x]
                ys = [pts2[i].y, pts2[i+1].y]
                plt.plot(xs, ys, "-k", linewidth=1)

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

        if pts2 != None:
            ts_rad = [p.t * math.pi/180 for p in pts2]
            rs = [p.r for p in pts2]
            for i in range(len(pts2) - 1):
                ax.plot([ts_rad[i], ts_rad[i+1]], [rs[i], rs[i+1]], "-k", linewidth=1)
        
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
    # svg_file = "../svgs_production/hilbert_d5.svg"
    # svg_file = "../svgs_production/flowers.svg"
    # svg_file = "../svgs_production/field.svg"
    svg_file = "../svgs_production/hand_eye_4.svg"
    # svg_file = "../svgs_production/woman_with_sunglasses.svg"
    # svg_file = "../svgs_production/ocean.svg"
    # svg_file = "../svgs_production/possum.svg"
    svg_parser = SVGParser()
    pts = svg_parser.get_pts_from_file(svg_file)
    polar_pts = svg_parser.convert_to_table_axes(pts)
    polar_pts = remove_repeated_pts(polar_pts)
    adjusted_points = sharp_compensate_pts(polar_pts)
    # print(polar_pts)
    create_cartesian_plot(polar_pts, adjusted_points)
    # create_cartesian_plot(adjusted_points, polar_pts)
    # animate_cartesian_plot(adjusted_points, polar_pts)
    # animate_polar_plot(adjusted_points, polar_pts)
    # create_polar_plot(polar_pts)