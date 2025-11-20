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





## algoritm

* **det** – Moving corner detection
* **predict** – Position prediction using a Kalman Filter
* **corr** – Corner matching using Normalized Cross-Correlation (NCC)


### 1. **det (Detection)**

This stage identifies moving corner points in the frame.

### 2. **predict (Kalman Filter Prediction)**

Estimates the expected location of each tracked point in the next frame using a Kalman filter.

### 3. **corr (Correlation / NCC)**

Searches for the corresponding moving corner in the current frame by measuring similarity using the NCC metric.

---

## Initial Latency Analysis

The detection and correlation stages dominate the processing time:

* **det:** 6,403k cycles
* **corr:** 9,566k cycles

Running at 20 MHz:

```
(6,403k + 9,566k) × 50 ns = 796 ms
```

This results in a throughput of approximately **1 frame per second**.

---

## Latency Optimization Techniques

### **1. Motion Filtering Before Corner Detection**

Before checking whether a point is a corner, its motion is evaluated.
Only **5%** of all points are identified as moving:

```
6,403k × 0.05 = 320k cycles
```

### **2. Correlation Row Pruning Through Early Rejection**

During NCC evaluation, if the similarity of a point in one row drops below 50%, subsequent points in that row are skipped, since none of them can exceed 90% correlation.

This reduces the evaluation to **7%** of the original operations:

```
9,566k × 0.07 = 669k cycles
```

---

## Final Latency After Optimization

```
(320k + 669k) × 50 ns = 50 ms
```

This corresponds to a throughput of approximately **20 frames per second** 

---



