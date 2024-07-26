import math

def cart_to_polar(x, y, z):
    rx = math.sqrt(x**2 + y**2 + z**2)
    ry = math.atan2(y, x)
    len = math.acos(z / rx)
    return rx, ry, len

# Beispielkoordinaten
x = 1
y = 1
z = 1

rx, ry, len = cart_to_polar(x, y, z)

print("Polarkoordinaten:")
print("rx =", rx)
print("ry =", ry)
print("len =", len)

def polar_to_cart(rx, ry, len):
    x = rx * math.cos(ry) * math.sin(len)
    y = rx * math.sin(ry) * math.sin(len)
    z = rx * math.cos(len)
    return x, y, z

# Beispiel Polarkoordinaten
#rx = 1
#ry = math.pi / 4
#len = math.pi / 6

x, y, z = polar_to_cart(rx, ry, len)

print("Kartesische Koordinaten:")
print("x =", x)
print("y =", y)
print("z =", z)