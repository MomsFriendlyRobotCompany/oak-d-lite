
![](https://shop.luxonis.com/cdn/shop/files/f_beb639a7-3c06-40e7-80e9-eb72b4be76bc_360x.png?v=1683804105)

# Oak-D Lite Camera and IMU

| Parameter          | Value           |
|:-------------------|:----------------|
| Sensor             | OV7251          |
| `dai.CameraBoardSocket` | `CAM_B` `CAM_C` |
| Type               | Monochrome      |
| DFOV / HFOV / VFOV | 86° / 73° / 58° |
| Size               | 1/7.5           |
| Shutter            | Global          |
| Focus              | Fixed           |

| Parameter          | Value           |
|:-------------------|:----------------|
| Sensor             | IMX214          |
| `dai.CameraBoardSocket` | `CAM_A` |
| Type               | Color           |
| DFOV / HFOV / VFOV | 81° / 69° / 54° |
| Size               | 1/3.06          |
| Shutter            | Rolling         |
| Focus              | Fixed           |


BMI270 IMU:

- Accelerometer:
  - ODR: 25Hz, 50Hz, 100Hz, 200Hz, 250Hz
  - Noise $mG/\sqrt{Hz}$: 0.16
  - Noise mG-RMS: 1.51
- Gyroscope:
  - ODR: 25Hz, 50Hz, 100Hz, 200Hz, 250Hz
  - Noise $dps/\sqrt{Hz}$: 0.010
  - Noise dps-RMS: 0.09

> Note that BMI270 "rounds down" the input frequency to the next available frequency. For example, if you set the frequency to 99 it will round it to 50Hz. Additionally, the current max frequency of ~250 Hz is set when the input is >400Hz.

- [Camera Spec ref](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)
- [Datasheet](https://github.com/luxonis/depthai-hardware/tree/master/DM9095_OAK-D-LITE_DepthAI_USB3C/Datasheet)
- [Luxonis pypi.org](https://pypi.org/user/luxonis/)
- [depthai examples](https://docs.luxonis.com/software/depthai/examples/)

```
python3 -m venv venvs/oak
changeenv oak
pip install -U pip setuptools
pip install depthai
pip install opencv-contrib-python
```

## Acceleration Notes

For an IMU (Inertial Measurement Unit), when the z-axis is oriented upward (aligned with the direction opposite to gravity), the acceleration due to gravity is typically read as +1G. Here’s why:

- IMUs measure acceleration, including the effect of gravity. The standard convention is that the accelerometer outputs a positive value when the acceleration is in the same direction as the axis.
- When the z-axis is pointing upward, gravity acts downward (opposite to the z-axis direction). However, the accelerometer measures the reaction to gravity as an upward acceleration of the sensor itself (Newton’s third law in play: the normal force or constraint keeping the IMU stationary against gravity).
- By common convention, when the z-axis is up and the IMU is stationary, the accelerometer will read +1G along the z-axis, indicating an acceleration of approximately +9.81 m/s² (the standard gravitational acceleration).

If the z-axis were pointing downward (aligned with gravity), the reading would be -1G, or -9.81 m/s², because the acceleration due to gravity would be in the negative direction of the axis.

So, with the z-axis up, you should expect +1G. Note that this assumes a typical IMU configuration—some specific devices or software might flip the sign based on their coordinate system, but the standard convention follows this logic.

## Optional

```
pip install opencv-contrib-python
pip install depthai-viewer
```

## Protobuf

```
protoc --python_out=../msgs  AllMsgs.proto
```

# MIT License

**Copyright (c) 2024 Mom's Friendly Robot Company**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
