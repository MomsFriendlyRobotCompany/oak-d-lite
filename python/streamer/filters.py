# import cv2
import numpy as np
from squaternion import Quaternion
# from collections import deque
# from tqdm import tqdm


class CF:
  """
  Performance increases if the bias is removed and any scaling performed
  on the sensor readings before using the filter.
  
  For accelerometers, there is no need to remove gravity since the 
  values will be normalized.
  
  https://ahrs.readthedocs.io/en/latest/filters/complementary.html
  """
  def __init__(self,aa=0.02):
    self.q = Quaternion()
    self.aa = aa
      
  def update(self, a, g, dt, m=None):
    """
    Update attitude quaternion with accelerometer and gyroscope 
    readings. Optionally, a magnitometer can be used to better
    recover the yaw orientation.
    
    a: accel [doesn't matter], gets normalized
    g: gyro [rad/sec]
    m: mag [doesn't matter], gets normalized
    dt: time step [sec]
    """
    dt = float(dt)
    qw = self.q + 0.5*dt*self.q*Quaternion(0,g.x, g.y, g.z)

    a = np.array([a.x, a.y, a.z])
    a = a / np.linalg.norm(a)
    
    # https://academicworks.cuny.edu/cgi/
    #    viewcontent.cgi?referer=&httpsredir=1&article=1247&context=cc_pubs
    # Fig 2, pg 19315
    # rot = self.q.to_rot() # value?
    # a = rot @ a           # value?

    ax,ay,az = a
    roll = np.arctan2(ay,az)
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
    
    if m is None:
      # qq = qw.conjugate
      # r,p,y = qw.to_euler(degrees=False)
      # yaw = y
      yaw = 0.0
    else:
      mx,my,mz = m / np.linalg.norm(m)
      x = mx*np.cos(pitch)+mz*np.sin(pitch)
      y = mx*np.sin(roll)*np.sin(pitch)+my*np.cos(roll)-mz*np.sin(roll)*np.cos(pitch)
      yaw = np.pi/2 - np.arctan2(y, x) # or np.atan2(x,y) is the same thing
        
    qam = Quaternion.from_euler(roll, pitch, yaw)
    
    self.q = (1-self.aa) * qw + self.aa * qam
    return self.q

