# UKF-based Multi-Object Tracker on FPGA  


A fully hardware-accelerated multi-object tracking (MOT) system implemented in **Vivado HLS (C++)** and targeted for Xilinx FPGA platforms.

The core of the tracker is an **Unscented Kalman Filter (UKF)** with constant velocity motion model, completely optimized and pipelined for maximum throughput on FPGA.

Real-time performance: **~15 FPS** on 1080p video streams (xc7k410t-ffg900-2).

### Demo Video  
A full tracking demo on real traffic footage (processed in MATLAB for visualization, actual tracking runs 100% on FPGA):

[Watch Traffic Demo (traffic.avi)]

## Key Features
- Unscented Kalman Filter with sigma-point propagation
- Custom high-performance Cholesky decomposition 
- Correlation-based data association 
- Complete C/RTL co-simulation testbench

## Resource Utilization 
| Resource | Used   |
|----------|--------|
| BRAM     | 700    |
| DSP      | 567    |
| FF       | 30k    |
| LUT      | 92k    |

## Performance
- Throughput: ~15 FPS (360*640) 





<img width="2012" height="1185" alt="output" src="https://github.com/user-attachments/assets/13b4ce81-bd73-4e80-b2f8-7c5df87e0217" />
*Real traffic sequence – Frame 161*  
- **Green circles**: Active & confirmed tracks  
- **Yellow circles**: Newly detected objects (before association)  
- **Red lines**: UKF predicted trajectory using sigma points
