module top(input  logic clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic MemWrite);

  logic [31:0] ReadData;

  // Instantiate processor and external memory
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, WriteData, ReadData);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule


module riscvmulti(input  logic        clk, reset,
                   output logic        MemWrite,
                   output logic [31:0] DataAdr, WriteData,
                   input  logic [31:0] ReadData);

  logic       ALUSrc, RegWrite, Zero, lt_zero, AdrSrc;
  logic [1:0] ResultSrc, ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic 		  IRWrite, PCWrite;
  logic [3:0] ALUControl;
  logic [31:0] Instr;

  controller ctrl(clk, reset, Instr[6:0], Instr[14:12], Instr[30], Zero, lt_zero,
                ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl,
                IRWrite, PCWrite, RegWrite, MemWrite, ALUSrc);
  datapath dp(clk, reset, ResultSrc,
              ALUSrc, RegWrite,
              ImmSrc, ALUControl,
              Zero, lt_zero, Instr,
              DataAdr, WriteData, ReadData, IRWrite, PCWrite, AdrSrc, ALUSrcA, ALUSrcB);

endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Controller~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
module controller(input  logic clk, reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic funct7b5,
                  input  logic Zero,
                  input  logic lt_zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc,
                  output logic AdrSrc,
                  output logic [3:0] ALUControl,
                  output logic IRWrite, PCWrite,
                  output logic RegWrite, MemWrite,
                  output logic ALUSrc);

  logic [1:0] ALUOp;
  logic       Branch, PCUpdate;
  
  // Instantiate the main FSM
	main_fsm fsm_inst (
	  .clk(clk),
	  .reset(reset),
	  .op(op),
	  .Zero(Zero),
	  .ALUSrcA(ALUSrcA),
	  .ALUSrcB(ALUSrcB),
	  .ALUOp(ALUOp),
	  .ResultSrc(ResultSrc),
	  .AdrSrc(AdrSrc),
	  .IRWrite(IRWrite),
	  .PCUpdate(PCUpdate),
	  .MemWrite(MemWrite), 
	  .RegWrite(RegWrite),
	  .Branch(Branch)
	);


  aludec aludec_inst(op[5], funct3, funct7b5, ALUOp, ALUControl);
  instrdec instrdec_inst(op, ImmSrc);
  
  assign PCWrite = Branch & ((funct3 == 3'b000) ? Zero : lt_zero) | PCUpdate;
endmodule


module main_fsm(
    input  logic       clk,
    input  logic       reset,
    input  logic [6:0] op,
    input  logic       Zero,
    output logic [1:0] ALUSrcA, ALUSrcB, ALUOp, ResultSrc,
    output logic       AdrSrc,
    output logic       IRWrite, PCUpdate, MemWrite, RegWrite, Branch
);


    typedef enum logic [3:0] {
        S0_FETCH, S1_DECODE, S2_MEMADR, S3_MEMREAD, S4_MEMWB,
        S5_MEMWRITE, S6_EXECUTER, S7_ALUWB, S8_EXECUTEI,
        S9_JAL, S10_BEQ
    } state_t;

    state_t state, next_state;

    // State Transition
    always_ff @(posedge clk or posedge reset) begin
        if (reset) 
            state <= S0_FETCH;
        else 
            state <= next_state;
    end

    // Next State Logic and Output Logic
    always_comb begin
        // Default values for outputs to avoid latches
        IRWrite = 0;
        PCUpdate = 0;
        RegWrite = 0;
        MemWrite = 0;
		  Branch = 0;
        AdrSrc = 0;
        ALUOp = 2'b00;
        ALUSrcA = 2'b00;
        ALUSrcB = 2'b00;
        ResultSrc = 2'b00;

        case (state)
            // Fetch state: Fetch instruction from memory, increment PC
            S0_FETCH: begin
                AdrSrc = 0;
					 IRWrite = 1;
                ALUSrcA = 2'b00;
                ALUSrcB = 2'b10;
                ALUOp = 2'b00;
                ResultSrc = 2'b10;
					 PCUpdate = 1;
                next_state = S1_DECODE;
            end
            
            // Decode state: Decode instruction, set ALUSrc for subsequent operations
            S1_DECODE: begin
                ALUSrcA = 2'b01;
                ALUSrcB = 2'b01;
                ALUOp = 2'b00;
                case (op)
                    7'b0000011, 7'b0100011: next_state = S2_MEMADR;  // lw, sw
                    7'b0110011: next_state = S6_EXECUTER;            // R-type
                    7'b0010011: next_state = S8_EXECUTEI;            // I-type ALU
                    7'b1101111: next_state = S9_JAL;                 // jal
                    7'b1100011: next_state = S10_BEQ;                // beq
                    default:    next_state = S0_FETCH;
                endcase
            end

            // Memory Address Computation state
            S2_MEMADR: begin
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b01;
                ALUOp = 2'b00;
					 next_state = (op == 7'b0000011) ? S3_MEMREAD : S5_MEMWRITE;
            end
            
            // Memory Read state
            S3_MEMREAD: begin
                ResultSrc = 2'b00;
					 AdrSrc = 1;
                next_state = S4_MEMWB;
            end
            
            // Memory Write-Back state
            S4_MEMWB: begin
                ResultSrc = 2'b01;
                RegWrite = 1;
                next_state = S0_FETCH;
            end
            
            // Memory Write state
            S5_MEMWRITE: begin
					 ResultSrc = 2'b00;
					 AdrSrc = 1;
                MemWrite = 1;
                next_state = S0_FETCH;
            end
            
            // R-type Execute state
            S6_EXECUTER: begin
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b00;
                ALUOp = 2'b10;
                next_state = S7_ALUWB;
            end
            
            // ALU Write-Back state
            S7_ALUWB: begin
                ResultSrc = 2'b00;
                RegWrite = 1;
                next_state = S0_FETCH;
            end
            
            // I-type Execute state
            S8_EXECUTEI: begin
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b01;
                ALUOp = 2'b10;
                next_state = S7_ALUWB;
            end
            
            // Jump and Link (JAL) state
            S9_JAL: begin
                ALUSrcA = 2'b01;
                ALUSrcB = 2'b10;
                ALUOp = 2'b00;
                ResultSrc = 2'b00;
					 PCUpdate = 1;
                next_state = S7_ALUWB;
            end
            
            // Branch if Equal (BEQ) state
            S10_BEQ: begin
                ALUSrcA = 2'b10;
                ALUSrcB = 2'b00;
                ALUOp = 2'b01;
                ResultSrc = 2'b00;
					 Branch = 1;
                next_state = S0_FETCH;
            end

            // Default case to handle unexpected states
            default: next_state = S0_FETCH;
        endcase
    end
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [3:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 4'b0000; // addition
      2'b01:                ALUControl = 4'b0001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 4'b0001; // sub
                          else          
                            ALUControl = 4'b0000; // add, addi
                 3'b010:    ALUControl = 4'b0101; // slt, slti
                 3'b110:    ALUControl = 4'b0011; // or, ori
                 3'b111:    ALUControl = 4'b0010; // and, andi
					  3'b001:	 ALUControl = 4'b1001; //SRA
					  3'b100:    ALUControl = 4'b0101; // BLT (signed comparison)
                 default:   ALUControl = 4'bxxx; // ???
               endcase
    endcase
endmodule


module instrdec (input logic [6:0] op,
						output logic [1:0] ImmSrc);
 always_comb
 case(op)
	7'b0110011: ImmSrc = 2'b00; // R-type
	7'b0010011: ImmSrc = 2'b00; // I-type ALU
	7'b0000011: ImmSrc = 2'b00; // lw
	7'b0100011: ImmSrc = 2'b01; // sw
	7'b1100011: ImmSrc = 2'b10; // beq
	7'b1101111: ImmSrc = 2'b11; // jal
	default: ImmSrc = 2'b00; // ???
 endcase
endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Datapath~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc, 
                input  logic        ALUSrc, //will never used
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [3:0]  ALUControl,
                output logic        Zero,
					 output logic			lt_zero,
                output logic [31:0] Instr,
                output logic [31:0] DataAdr, WriteData,
                input  logic [31:0] ReadData,
					 input  logic  		IRWrite, PCWrite,
					 input  logic 			AdrSrc,
					 input  logic [1:0] 	ALUSrcA, ALUSrcB);

  logic [31:0] PC, PCNext, OldPC;
  logic [31:0] ImmExt;
  logic [31:0] RD1, RD2, A, SrcA, SrcB, Data;
  logic [31:0] Result, ALUReseult, ALUOut;


  assign PCNext = Result;
  flopenr #(32) pcflop(clk, reset, PCWrite, PCNext, PC);
  mux2 #(32) adrmux(PC, Result, AdrSrc, DataAdr); 
  
  flopenr #(32) memflop1(clk, reset, IRWrite, PC, OldPC);
  flopenr #(32) memflop2(clk, reset, IRWrite, ReadData, Instr);
  flopr #(32) dataflop(clk, reset, ReadData, Data);
  
  regfile rf(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, RD1, RD2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);
  
  flopr #(32) reg_f1(clk, reset, RD1, A);
  flopr #(32) reg_f2(clk, reset, RD2, WriteData);
  
  mux3 #(32)  SrcAMux(PC, OldPC, A, ALUSrcA, SrcA);
  mux3 #(32)  SrcBMux(WriteData, ImmExt, 32'd4, ALUSrcB, SrcB);
  
  alu alu(SrcA, SrcB, ALUControl, ALUReseult, Zero, lt_zero);
  
  flopr #(32) alu_result_out(clk, reset, ALUReseult, ALUOut);
  mux3 #(32)  resultMux(ALUOut, Data, ALUReseult, ResultSrc, Result);
  
endmodule


module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
      default: immext = 32'bx; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
		(input logic clk, reset, en, input logic [WIDTH-1:0] d, output logic [WIDTH-1:0] q);
	
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else if (en) q <= d;

endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule


module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
    initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned 

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [3:0]  alucontrol,
           output logic [31:0] result,
           output logic        Zero,
			  output logic			 lt_zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      4'b0000:  result = sum;                 // add
      4'b0001:  result = sum;                 // subtract
      4'b0010:  result = a & b;               // and
      4'b0011:  result = a | b;       			 // or
      4'b0100:  result = a ^ b;       			 // xor
      4'b0101:  result = (a < b) ? 32'd1 : 32'd0;       // slt
      4'b0110:  result = a << b;       		 // sll
      4'b0111:  result = a >> b;       		 // srl
		4'b1001:  result = a >> b[4:0]; 			 //sra
      default: result = 32'bx;
    endcase

  assign Zero = (result == 32'b0);
  assign lt_zero = result[31];
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule

