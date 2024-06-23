# MARG (Magnetic, Angular Rate, and Gravity) orientation estimation

This crate provides MARG IMU orientation estimation based on TRIAD DCM-estimation and Kalman Filtering techniques.
It is aimed at `no_std` environments and type-generic.

---

**Work in progress:** The estimator currently requires an allocator to construct the internal filters and is generally
considered work in progress.
