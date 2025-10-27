Ultra-Low Power SIMD AI Inference Accelerator
Overview

This project implements a SystemVerilog-based ultra-low power AI accelerator designed using a SIMD (Single Instruction, Multiple Data) architecture.
It performs parallel multiply–accumulate (MAC) operations across multiple lanes for efficient edge AI inference.

Originally intended for eSim, the design was successfully simulated on EDA Playground due to better compatibility with SystemVerilog modules.

Key Features

4-lane SIMD MAC architecture

Integrated clock gating and power gating

ReLU activation function per lane

Designed for sub-µW power operation

Simulated using Icarus Verilog (EDA Playground)

Files

design.sv → Main accelerator design

testbench.sv → Testbench for simulation

SMID_Chip.vcd → Generated waveform

simd_waveform.png → Output waveform image
