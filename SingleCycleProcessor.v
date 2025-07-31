module SingleCycleCPU(input clk);
    reg [31:0] PC = 0;

    wire [31:0] instr;
    wire [5:0] opcode = instr[31:26];
    wire [4:0] rs = instr[25:21];
    wire [4:0] rt = instr[20:16];
    wire [4:0] rd = instr[15:11];
    wire [15:0] imm = instr[15:0];
    wire [5:0] funct = instr[5:0];
    wire [25:0] jumpAddr = instr[25:0];

    wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump;
    wire [1:0] ALUOp;

    wire [31:0] readData1, readData2, writeData;
    wire [31:0] signExtImm = {{16{imm[15]}}, imm};

    wire [2:0] ALUControl;
    wire [31:0] ALUResult;
    wire Zero;
    wire [31:0] memData;

    wire [31:0] srcB = (ALUSrc) ? signExtImm : readData2;
    wire [4:0] writeReg = (RegDst) ? rd : rt;

    InstrMem imem(.addr(PC), .instr(instr));
    ControlUnit cu(.opcode(opcode), .RegDst(RegDst), .ALUSrc(ALUSrc), .MemtoReg(MemtoReg),
                   .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite),
                   .Branch(Branch), .Jump(Jump), .ALUOp(ALUOp));
    RegFile rf(.clk(clk), .regWrite(RegWrite), .rs(rs), .rt(rt), .rd(writeReg),
               .writeData(writeData), .readData1(readData1), .readData2(readData2));
    ALUControl aluctrl(.ALUOp(ALUOp), .funct(funct), .ALUControl(ALUControl));
    ALU alu(.A(readData1), .B(srcB), .ALUControl(ALUControl), .Result(ALUResult), .Zero(Zero));
    DataMem dmem(.clk(clk), .MemRead(MemRead), .MemWrite(MemWrite),
                 .addr(ALUResult), .writeData(readData2), .readData(memData));

    assign writeData = (MemtoReg) ? memData : ALUResult;

    wire [31:0] PCPlus4 = PC + 4;
    wire [31:0] branchAddr = PCPlus4 + (signExtImm << 2);
    wire [31:0] jumpAddrFull = {PCPlus4[31:28], jumpAddr, 2'b00};

    always @(posedge clk) begin
        if (Jump)
            PC <= jumpAddrFull;
        else if (Branch && Zero)
            PC <= branchAddr;
        else
            PC <= PCPlus4;
    end
endmodule

// ALU
module ALU(
    input [31:0] A, B,
    input [2:0] ALUControl,
    output reg [31:0] Result,
    output Zero
);
    always @(*) begin
        case (ALUControl)
            3'b000: Result = A & B;
            3'b001: Result = A | B;
            3'b010: Result = A + B;
            3'b110: Result = A - B;
            3'b111: Result = (A < B) ? 1 : 0;
            default: Result = 0;
        endcase
    end
    assign Zero = (Result == 0);
endmodule

// Register File
module RegFile(
    input clk,
    input regWrite,
    input [4:0] rs, rt, rd,
    input [31:0] writeData,
    output [31:0] readData1, readData2
);
    reg [31:0] registers[31:0];

    assign readData1 = registers[rs];
    assign readData2 = registers[rt];

    always @(posedge clk) begin
        if (regWrite && rd != 0)
            registers[rd] <= writeData;
    end
endmodule

// Control Unit
module ControlUnit(
    input [5:0] opcode,
    output reg RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump,
    output reg [1:0] ALUOp
);
    always @(*) begin
        {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump, ALUOp} = 10'b0;

        case (opcode)
            6'b000000: begin // R-type
                RegDst = 1; RegWrite = 1; ALUOp = 2'b10;
            end
            6'b100011: begin // lw
                ALUSrc = 1; MemtoReg = 1; RegWrite = 1; MemRead = 1; ALUOp = 2'b00;
            end
            6'b101011: begin // sw
                ALUSrc = 1; MemWrite = 1; ALUOp = 2'b00;
            end
            6'b000100: begin // beq
                Branch = 1; ALUOp = 2'b01;
            end
            6'b000010: begin // jump
                Jump = 1;
            end
        endcase
    end
endmodule

// ALU Control
module ALUControl(
    input [1:0] ALUOp,
    input [5:0] funct,
    output reg [2:0] ALUControl
);
    always @(*) begin
        case (ALUOp)
            2'b00: ALUControl = 3'b010; // add
            2'b01: ALUControl = 3'b110; // sub
            2'b10: begin
                case (funct)
                    6'b100000: ALUControl = 3'b010; // add
                    6'b100010: ALUControl = 3'b110; // sub
                    6'b100100: ALUControl = 3'b000; // and
                    6'b100101: ALUControl = 3'b001; // or
                    6'b101010: ALUControl = 3'b111; // slt
                    default:   ALUControl = 3'b010;
                endcase
            end
            default: ALUControl = 3'b010;
        endcase
    end
endmodule

// Instruction Memory
module InstrMem(
    input [31:0] addr,
    output [31:0] instr
);
    reg [31:0] memory [0:255];

    assign instr = memory[addr[9:2]]; // word aligned

    initial begin
        // preload with instructions (example)
        memory[0] = 32'h20100005; // addi $s0, $zero, 5
        memory[1] = 32'h20110003; // addi $s1, $zero, 3
        memory[2] = 32'h02119020; // add  $s2, $s0, $s1
    end
endmodule

// Data Memory
module DataMem(
    input clk,
    input MemRead, MemWrite,
    input [31:0] addr, writeData,
    output [31:0] readData
);
    reg [31:0] memory [0:255];

    assign readData = (MemRead) ? memory[addr[9:2]] : 0;

    always @(posedge clk) begin
        if (MemWrite)
            memory[addr[9:2]] <= writeData;
    end
endmodule
