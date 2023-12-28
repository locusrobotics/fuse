import numpy as np
import quaternion
from math import sin, cos, tan

p1x = 0.0
p1y = 0.0
p1z = 0.0
r1 = -2.490
p1 = -0.206
y1 = 3.066
v1x = 0.1
v1y = 0.2
v1z = 0.1
w1x = 1.570796327
w1y = 1.570796327
w1z = -1.570796327
a1x = -0.5
a1y = 1.0
a1z = 1.0
dt = 0.1

def get_quaternion(roll, pitch, yaw):
  w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/ 2) * sin(pitch / 2) * sin(yaw / 2)
  x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
  y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
  z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
  q = np.quaternion()
  q.w = w
  q.x = x
  q.y = y
  q.z = z
  q = q.normalized()
  return q
  

def main():
  p2x = p1x + \
    ((cos(y1)*cos(p1))*v1x + (cos(y1)*sin(p1)*sin(r1) - sin(y1)*cos(r1))*v1y + (cos(y1)*sin(p1)*cos(r1) + sin(y1)*sin(r1))*v1z) * dt + \
    ((cos(y1)*cos(p1))*a1x + (cos(y1)*sin(p1)*sin(r1) - sin(y1)*cos(r1))*a1y + (cos(y1)*sin(p1)*cos(r1) + sin(y1)*sin(r1))*a1z) * dt * dt * 0.5

  p2y = p1y + \
    ((sin(y1)*cos(p1))*v1x + (sin(y1)*sin(p1)*sin(r1) + cos(y1)*cos(r1))*v1y + (sin(y1)*sin(p1)*cos(r1) - cos(y1)*sin(r1))*v1z) * dt + \
    ((sin(y1)*cos(p1))*a1x + (sin(y1)*sin(p1)*sin(r1) + cos(y1)*cos(r1))*a1y + (sin(y1)*sin(p1)*cos(r1) - cos(y1)*sin(r1))*a1z) * dt * dt * 0.5 

  p2z = p1z + \
    ((-sin(p1))*v1x + (cos(p1)*sin(r1))*v1y + (cos(p1)*cos(r1))*v1z) * dt + \
    ((-sin(p1))*a1x + (cos(p1)*sin(r1))*a1y + (cos(p1)*cos(r1))*a1z) * dt * dt * 0.5
  
  r2 = r1 + \
    (w1x + (sin(r1)*tan(p1))*w1y + (cos(r1)*tan(p1))*w1z) * dt

  p2 = p1 + \
    ((cos(r1))*w1y + (-sin(r1))*w1z) * dt

  y2 = y1 + \
    ((sin(r1)/cos(p1))*w1y + (cos(r1)/cos(p1))*w1z) * dt
  
  q2 = get_quaternion(r2, p2, y2)
  
  v2x = v1x + a1x * dt
  v2y = v1y + a1y * dt
  v2z = v1z + a1z * dt

  w2x = w1x
  w2y = w1y
  w2z = w1z

  a2x = a1x
  a2y = a1y
  a2z = a1z

  print("p2x: ", p2x)
  print("p2y: ", p2y)
  print("p2z: ", p2z)
  print("r2: ", r2)
  print("p2: ", p2)
  print("y2: ", y2)
  print("q2w: ", q2.w)
  print("q2x: ", q2.x)
  print("q2y: ", q2.y)
  print("q2z: ", q2.z)
  print("v2x: ", v2x)
  print("v2y: ", v2y)
  print("v2z: ", v2z)
  print("w2x: ", w2x)
  print("w2y: ", w2y)
  print("w2z: ", w2z)
  print("a2x: ", a2x)
  print("a2y: ", a2y)
  print("a2z: ", a2z)
    
if __name__ == '__main__':
  main()