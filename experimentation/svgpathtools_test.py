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
from svgpathtools import svg2paths
from svgpathtools import svg2paths2

def get_artboard_size(file_path):
    # svg2paths2 returns three values; the third contains the root attributes
    paths, path_attributes, svg_attributes = svg2paths2(file_path)
    
    # Extract width, height, and viewBox using the .get() method 
    # to avoid KeyErrors if the attributes are missing.
    # width = svg_attributes.get('width')
    # height = svg_attributes.get('height')
    viewbox_string = svg_attributes.get('viewBox')
    # Split the "min-x min-y width height" string
    parts = viewbox_string.split()
    
    if len(parts) == 4:
        vb_width = float(parts[2])
        vb_height = float(parts[3])
        return vb_width, vb_height
    
    return vb_width, vb_height

# Usage
# width, height, viewbox = get_artboard_size('your_file.svg')
# print(f"Width: {width}, Height: {height}, ViewBox: {viewbox}")

def get_svg_coordinates(file_path, samples_per_path=10):
    # Load the SVG file and extract paths
    paths, attributes = svg2paths(file_path)

    all_xy_coordinates = []

    for path in paths:
        path_coords = []
        
        # Avoid division by zero if samples_per_path is 1
        if samples_per_path <= 1:
            points_t = [0.0]
        else:
            points_t = [i / float(samples_per_path - 1) for i in range(samples_per_path)]

        # Sample points along the path
        for t in points_t:
            point = path.point(t)
            
            # Convert complex number representation to (X, Y) tuple
            x, y = point.real, point.imag
            path_coords.append((x, y))
        
        all_xy_coordinates.append(path_coords)

    # scale and center
    width, height = get_artboard_size(file_path)
    scaled_coordinates = []
    for path_coords in all_xy_coordinates:
        scaled_path = []
        for x, y in path_coords:
            new_x = (x / width) * (2 * DISH_RADIUS_MM) - DISH_RADIUS_MM
            new_y = -1*((y / height) * (2 * DISH_RADIUS_MM) - DISH_RADIUS_MM)
            scaled_path.append((new_x, new_y))
        scaled_coordinates.append(scaled_path)

    return scaled_coordinates

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

# Usage
# coordinates = get_svg_coordinates("../svgs_production/hand_eye_4.svg", samples_per_path=1000)
# coordinates = get_svg_coordinates("../svgs_production/flowers_4.svg", samples_per_path=2000)
coordinates = get_svg_coordinates("../svgs_production/hilbert_d5.svg", samples_per_path=6000)
# print(coordinates)
# Convert the nested lists of (x, y) tuples into a flat list of Point objects
points = [Point(x=x, y=y) for path in coordinates for x, y in path]
print(points)
create_cartesian_plot(pts=points)
