# Automotive Radar 4D Point Cloud Imaging

## About The Project

This project presents a MATLAB-based simulation of a **4D automotive radar system** that estimates **range, velocity, azimuth, and elevation** using a sparse 2D TDM-MIMO array and advanced signal processing algorithms.

It further explores the use of **Doppler Division Multiple Access (DDMA)** to improve orthogonality and transmission efficiency — particularly suited for **ADAS (Advanced Driver Assistance Systems)** and **Short Range Radar (SRR)** applications.

The work is part of an academic research project and includes signal modeling, 2D beamforming, matrix completion, and realistic automotive driving scenarios.

## Built With

- [MATLAB](https://www.mathworks.com/products/matlab.html) — main simulation environment  
- Signal Processing Toolbox  
- Phased Array System Toolbox  
- Driving Scenario Designer  

## Getting Started

To explore or modify the simulations:

### Prerequisites

- MATLAB R2021a or later  
- Signal Processing Toolbox  
- Phased Array Toolbox  
- (Optional) Driving Scenario Designer Toolbox for visual simulations

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/RonBartov/automotive-radar-4d-imaging.git

2. Open Matlab and add the repository path

3. Run the "mimo_ddma_main.m" file inside the "MIMO_DDMA_Waveform_Simulation" folder


## Acknowledgments

This project builds upon the work of several foundational research papers in the automotive radar field:

- [J. Ding et al.](https://doi.org/10.1109/Radar53847.2021.10028211), *"Automotive Radar 4D Point-cloud Imaging with 2D Sparse Array"*, IEEE CIE Radar Conference, 2021.
- [F. Xu, S. A. Vorobyov, and F. Yang], *"Transmit Beamspace DDMA Based Automotive MIMO Radar"*, IEEE Transactions on Vehicular Technology, 2022.
- [S. Sun and A. P. Petropulu], *"A Sparse Linear Array Approach in Automotive Radars Using Matrix Completion"*, ICASSP, 2020.

> © IEEE — All rights reserved to the original authors.  
> This repository references their work for academic, research, and educational purposes only.


