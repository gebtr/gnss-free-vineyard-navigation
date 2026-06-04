[![DOI](https://zenodo.org/badge/1259278320.svg)](https://doi.org/10.5281/zenodo.20544591)

# GNSS-Free Vineyard Navigation

Official implementation of the paper:

**A GNSS-free LiDAR-based navigation architecture for autonomous inter-row operation under sparse or absent vegetation conditions**

Smart Agricultural Technology (2026)

DOI: https://doi.org/10.1016/j.atech.2026.102106

---

## Overview

This repository provides the core ROS2 implementation of the LiDAR/RANSAC navigation method described in the associated publication.

The method relies exclusively on LiDAR perception and robust RANSAC regression to estimate vineyard row boundaries and the inter-row centerline without GNSS support.

The code is released to support the reproducibility of the published results, promote further research, and encourage academic use.

---

## Main Features

* GNSS-free inter-row navigation
* LiDAR-only perception
* RANSAC-based row boundary estimation
* Centerline estimation from vineyard posts
* ROS2 Humble compatible

---

## Repository Contents

```text
.
├── LICENSE
├── README.md
├── CITATION.cff
└── row_navigation_node.py
```

---

## Dependencies

* ROS2 Humble
* NumPy

---

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
```

---

## License

GNU General Public License v3.0 (GPL-3.0)

Copyright (C) 2026 Gerardo Betrò
