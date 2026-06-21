from main import *

# mode = SVGMode(svg_file_name="woman_with_sunglasses", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="possum", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="hand_eye", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="ocean", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="field", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="flowers", auto_center=False)
# mode = SVGMode(svg_file_name="hex_gosper_d4", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="dither_wormhole", sharp_compensation_on=False)
# mode = SVGMode(svg_file_name="hilbert_d5")
# mode = SVGMode(svg_file_name="pentagon_fractal", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="hummingbird", sharp_compensation_factor=3.0)
# mode = SVGMode(svg_file_name="scalloped_spiral", sharp_compensation_factor=3.0)
mode = SVGMode(svg_file_name="moreflowers", sharp_compensation_factor=3.0)
mode.startup()

animate_cartesian_plot(mode.polar_pts, mode.pre_compensation_pts)
# animate_polar_plot(mode.polar_pts, mode.pre_compensation_pts)