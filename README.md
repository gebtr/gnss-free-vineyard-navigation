# GNSS-Free Vineyard Navigation

Official implementation of the paper:

**A GNSS-free LiDAR-based navigation architecture for autonomous inter-row operation under sparse or absent vegetation conditions**

Smart Agricultural Technology (2026)

DOI: https://doi.org/10.1016/j.atech.2026.102106

---

## Overview

This repository contains a ROS2-based autonomous navigation pipeline for agricultural rovers operating in vineyard inter-rows under sparse or absent vegetation conditions.

The approach relies exclusively on LiDAR perception and robust RANSAC regression to estimate vineyard row boundaries and the inter-row centerline without GNSS support.

## Main Features

- GNSS-free navigation
- LiDAR-only perception
- Robust RANSAC row estimation
- ROS2 Humble compatible
- Real-world vineyard validation
- Gazebo simulation support

## Citation

If you use this software in academic work, please cite:

```bibtex
@article{Betro2026,
  author = {Gerardo Betrò and Simone Pascuzzi and Francesco Paciolla},
  title = {A GNSS-free LiDAR-based navigation architecture for autonomous inter-row operation under sparse or absent vegetation conditions},
  journal = {Smart Agricultural Technology},
  year = {2026},
  doi = {10.1016/j.atech.2026.102106}
}
