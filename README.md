# MARG (Magnetic, Angular Rate, and Gravity) orientation estimation

[![Crates.io](https://img.shields.io/crates/v/marg-orientation)](https://crates.io/crates/marg-orientation)
[![Crates.io](https://img.shields.io/crates/l/marg-orientation)](https://crates.io/crates/marg-orientation)
![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/sunsided/marg-orientation/rust.yml)
[![docs.rs](https://img.shields.io/docsrs/marg-orientation)](https://docs.rs/marg-orientation/)
[![codecov](https://codecov.io/gh/sunsided/marg-orientation/graph/badge.svg?token=LTSCCVqdTp)](https://codecov.io/gh/sunsided/marg-orientation)

This crate provides MARG IMU orientation estimation based on TRIAD DCM-estimation and Kalman Filtering techniques.
It is aimed at `no_std` environments and type-generic.

---

**Work in progress:** The estimator currently requires an allocator to construct the internal filters and is generally
considered work in progress.
