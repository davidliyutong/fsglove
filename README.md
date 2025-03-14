<p align="center">
  <h3 align="center"><strong>FSGlove: <br>An Inertial-Based Hand Tracking System with Shape-Aware Calibration</br></strong></h3>
<p align="center">
    <a href="https://github.com/">Yutong Li</a><sup>1</sup><span class="note">*</span>,
    <a href="https://github.com/">Jieyi Zhang</a><sup>1</sup><span class="note">*</span>,
    <a href="https://github.com/">Wenqiang Xu</a><sup>1</sup>,
    <a href="https://github.com/">Tutian Tang</a><sup>1</sup>,
    <a href="https://github.com/">Cewu Lu</a><sup>1†</sup>,
    <br>
    <br>
    <sup>†</sup>Corresponding authors.
    <br>
    </sup><span class="note">*</span>Equal contribution.
    <br>
    <sup>1</sup>Shanghai Jiao Tong University
    <br>
</p>

<p align="center">
  <table>
    <tr>
      <td><img src="assets/imgs/hand_protocol.png" alt="Hand Protocol" style="display: inline-block;" height="350"></td>
      <td><img src="assets/imgs/hand_ok.png" alt="Hand OK" style="display: inline-block;" height="350"></td>
    </tr>
  </table>
</p>

## Contents

- [Contents](#contents)
- [Introduction](#introduction)
- [Build Instructions](#build-instructions)
- [Firmware Setup](#firmware-setup)
- [DiffHCal Setup](#diffhcal-setup)

## Introduction
Accurate hand motion capture (MoCap) is critical for robotics, virtual reality, and biomechanics, yet existing systems often fail to capture high-degree-of-freedom (DoF) joint kinematics and personalized hand shapes. FSGlove addresses these limitations with an inertial-based system that tracks up to 48 DoFs and reconstructs hand shapes using DiffHCal, a novel calibration method. Equipped with IMUs on each finger joint and dorsum, FSGlove achieves high-resolution motion sensing. DiffHCal integrates with the MANO model via differentiable optimization, resolving joint kinematics, shape parameters, and sensor misalignment in a single step.

> **Note:** The source code for FSGlove will be open-sourced after the reviewing process.

## Build Instructions

TBD.

## Firmware Setup

TBD.

## DiffHCal Setup

TBD.
