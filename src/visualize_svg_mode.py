from main import *

# mode = SVGMode(svg_file_name="flowers", auto_center=False)
# mode = SVGMode(svg_file_name="wreath1", sharp_compensation_factor=3.0)
mode = SVGMode(svg_file_name="maze2", sharp_compensation_factor=3.0)
mode.startup()

animate_cartesian_plot(mode.polar_pts, mode.pre_compensation_pts)
# animate_polar_plot(mode.polar_pts, mode.pre_compensation_pts)