# GL-3 (Mechanical scanning 2D LiDAR)

## Guide
- Add files to your project
- Include header file
```
#include "gl_driver.h"
```
- Initialize UDP or Serial
```
// UDP : Set local IP as 10.110.1.3
std::string gl_ip = "10.110.1.2";
int gl_port = 2000;
int pc_port = 3000;
SOSLAB::GL* gl;
gl = new SOSLAB::GL(gl_ip, gl_port, pc_port);
```
```
// Serial
std::string gl_port = "/dev/ttyUSB0";
int gl_baudrate = 921600;
SOSLAB::GL* gl;
gl = new SOSLAB::GL(gl_port, gl_baudrate);
```
- Get Serial Number of GL
```
std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
```
- Get lidar data
```
// Start stream
gl->SetFrameDataEnable(true);

// Get data
SOSLAB::GL::framedata_t frame_data;
gl->ReadFrameData(frame_data);

//Stop stream and terminate
gl->SetFrameDataEnable(false);
delete gl;
```
- Data format
```
struct framedata_t
{
    std::vector<double> angle;
    std::vector<double> distance;
    std::vector<double> pulse_width;
};
```

## Test environment
- Windows 10
- Ubuntu 18.04 LTS

## Application demo
- [GL-3, Demo] 2D LiDAR, Mapping (https://youtu.be/AfsqlU_f-Go)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (https://youtu.be/_HBWe95GqXM)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (pulse-width version) (https://youtu.be/q4MeeS9eP4c)
- [GL-3, Demo] Human Tracking Demo of Multiple Mobile Robots (https://youtu.be/ZzEvm8gMPaA)
- [GL-3, Demo] 2D LiDAR Object Detection (https://youtu.be/V29QzU9AcQo)
