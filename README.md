# RISCV-Multicycle-Processor
A multi-cycle processor of a cpu designed according to the instruction set (assembly language) of RISC-V using System Verilog HDL.

## Overview
This project implements a multicycle RISC-V processor using SystemVerilog. It is designed to execute a subset of the RISC-V instruction set architecture (ISA) and demonstrates a step-by-step implementation of the fetch, decode, execute, memory, and write-back cycles.

The multicycle approach optimizes resource usage by allowing different stages of the instruction execution to share hardware resources.

## Features
- Multicycle instruction execution.
- Implementation of core RISC-V instructions.
- Modular design for easier debugging and extensibility.
- Support for simulation using industry-standard tools.

## Architecture
The processor follows a classic multicycle architecture with the following components:
- **Instruction Fetch (IF)**: Fetches instructions from memory.
- **Instruction Decode (ID)**: Decodes the instruction and fetches operands.
- **Execute (EX)**: Performs ALU operations or calculates addresses.
- **Memory Access (MEM)**: Reads from or writes to memory.
- **Write Back (WB)**: Writes results back to the register file.

## Dependencies and Tools
- **Hardware Description Language**: SystemVerilog
- **Simulator**: ModelSim, QuestaSim, or other SystemVerilog-compatible simulators
- **Waveform Viewer**: GTKWave or equivalent

## Supported Instructions
This processor supports the following RISC-V instructions:
- **Arithmetic and Logical**: `ADD`, `SUB`, `AND`, `OR`, `XOR`, `SLT`, `SLL`, `SRL`, `SRA` 
- **Memory Access**: `LW`, `SW`
- **Branching**: `BEQ`, `BNE`
- **Immediate Operations**: `ADDI`, `ORI`

