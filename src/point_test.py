from utils import *

cart_point = CartesianPt(1,1)
print(cart_point.to_tuple())
print(cart_point.r)
print(cart_point.t)

polar_point = PolarPt(cart_point.r, cart_point.t)
print(polar_point.to_tuple())
print(polar_point.x)
print(polar_point.y)

point = Point(x=1, y=1)
point.xy = (2,2)
print(point.xy)
print(point.rt)