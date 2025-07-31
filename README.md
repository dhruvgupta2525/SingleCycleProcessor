# SingleCycleProcessor
Single-Cycle MIPS CPU (Verilog)
Overview
This project implements a Single-Cycle MIPS CPU in Verilog. It mimics the behavior of a simplified MIPS processor, executing instructions in a single clock cycle. The processor includes components such as an ALU, register file, instruction memory, data memory, and control logic.

Features
Supports a subset of MIPS instructions:

R-type: add, sub, and, or, slt

I-type: lw, sw, beq, addi

J-type: jump

Single-cycle execution model

Word-aligned instruction and data memory

Parameter-free modular structure for extensibility and clarity

Modules
1. SingleCycleCPU
Top-level module integrating all components. Handles program counter (PC) updates based on control signals (branch, jump, etc.).

2. InstrMem
256-word instruction memory

Preloaded with a sample program:

assembly
Copy
Edit
addi $s0, $zero, 5  ; memory[0]
addi $s1, $zero, 3  ; memory[1]
add  $s2, $s0, $s1  ; memory[2]
3. DataMem
256-word data memory

Supports memory read/write using MemRead and MemWrite control signals.

4. RegFile
32 general-purpose 32-bit registers

Supports simultaneous read of two registers and write to one register on the rising edge of the clock.

5. ALU
Performs arithmetic and logical operations based on ALUControl

Outputs result and a Zero flag for conditional branches

6. ALUControl
Generates control signals for ALU based on funct field and ALUOp

7. ControlUnit
Decodes opcode to generate control signals:

RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump, and ALUOp

Instruction Format Support
R-Type: add, sub, and, or, slt

I-Type: lw, sw, beq, addi (manually added in memory)

J-Type: jump

Execution Flow
Fetch: Instruction is fetched from InstrMem using PC

Decode: Instruction fields are extracted, and control signals are generated

Execute:

ALU performs arithmetic/logical operations

PC is updated (normal, branch, or jump)

Memory Access: lw and sw access DataMem

Write Back: Result is written to the destination register

Usage Notes
Program counter increments by 4 unless a branch or jump is taken.

Memory addresses are word-aligned (addr[9:2] used).

$zero register (register 0) is hardcoded as zero by not writing to it.

Simulation Suggestions
To test the CPU:

Simulate the SingleCycleCPU module with a clock input.

Observe register values, memory content, and PC at each cycle.

Extend the instruction memory with more test instructions as needed.

Future Extensions
Support for additional MIPS instructions (ori, andi, bne, etc.)

Pipeline architecture (5-stage or multi-cycle)

Forwarding and hazard detection

Enhanced testbench and waveform generation

License
