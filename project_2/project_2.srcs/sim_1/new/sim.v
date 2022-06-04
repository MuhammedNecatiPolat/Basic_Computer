`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.05.2022 12:52:19
// Design Name: 
// Module Name: sim
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Part1_simulation();
    reg E;
    reg [1:0] FunSel;
    reg [7:0] I;
    reg clock;
    wire [7:0] Q;

    reg_8_bit uut(.E(E), .FunSel(FunSel), .clock(clock), .I(I), .Q(Q) );
    initial begin
        clock = 0;
        E = 1; FunSel = 2'b11; I = 8'd31; #100; //Reset
        E = 0; FunSel = 2'b10; I = 8'd31; #100; //E 0 Load
        E = 1; FunSel = 2'b10; I = 8'd31; #100; //Load 31
        E = 1; FunSel = 2'b00; I = 8'd31; #100; //Decrement
        E = 1; FunSel = 2'b01; I = 8'd35; #100; //Increment
        E = 0; FunSel = 2'b00; I = 8'd35; #100; //E 0 Decrement
        E = 0; FunSel = 2'b01; I = 8'd98; #100; //E 0 Increment
        E = 1; FunSel = 2'b10; I = 8'd98; #100; //Load 98
        E = 1; FunSel = 2'b11; I = 8'd98; #100; //Reset
        //E = 0; FunSel = 2'b10; I = 8'd98; #100; //deneme123
        //E = 1; FunSel = 2'b10; I = 8'd25; #100; //deneme12

        $finish;          
        end
        always begin
        clock = ~ clock ; #25; 
        end
endmodule

module Part1_16bit_simulation();
    reg E;
    reg [1:0] FunSel;
    reg [15:0] I;
    reg clock;
    wire [15:0] Q;

    reg_16_bit uut(.E(E), .FunSel(FunSel), .clock(clock), .I(I), .Q(Q) );
    initial begin
        clock = 0;
        E = 1; FunSel = 2'b11; I = 16'd31; #100; //Reset
        E = 0; FunSel = 2'b10; I = 16'd31; #100; //E 0 Load
        E = 1; FunSel = 2'b10; I = 16'd31; #100; //Load 31
        E = 1; FunSel = 2'b00; I = 16'd31; #100; //Decrement
        E = 1; FunSel = 2'b01; I = 16'd35; #100; //Increment
        E = 0; FunSel = 2'b00; I = 16'd35; #100; //E 0 Decrement
        E = 0; FunSel = 2'b01; I = 16'd98; #100; //E 0 Increment
        E = 1; FunSel = 2'b10; I = 16'd98; #100; //Load 98
        E = 0; FunSel = 2'b10; I = 16'd98; #100; //Load 98
        E = 1; FunSel = 2'b11; I = 16'd98; #100; //Reset
        $finish;          
        end
        always begin
        clock = ~ clock ; #25; 
        end
endmodule

module Part2a_simulation();
    reg clock;
    reg [1:0] FunSel;
    reg [7:0] I;
    reg [1:0] OutASel;
    reg [1:0] OutBSel;
    wire [7:0] OutA;
    wire [7:0] OutB;
    reg [3:0] RegSel; 
    
    RegisterFile part2atest(.clock(clock), .I(I), .OutASel(OutASel), .OutBSel(OutBSel),
                         .FunSel(FunSel), .RegSel(RegSel), .OutA(OutA), .OutB(OutB));
                         
    initial begin
    clock = 0; 
    I = 8'd25; RegSel = 4'b0000; FunSel = 2'b10; OutASel = 2'b10; OutBSel = 2'b11; #100; //Load 25 to every register
    I = 8'd34; RegSel = 4'b0110; FunSel = 2'b10; OutASel = 2'b10; OutBSel = 2'b11; #100; //Load 34 to Reg1 and Reg 4 look Reg 3 and Reg4
    I = 8'd10; RegSel = 4'b0000; FunSel = 2'b00; OutASel = 2'b00; OutBSel = 2'b01; #100; //Decrement Every Register Look Reg1 and Reg2
    I = 8'd42; RegSel = 4'b1001; FunSel = 2'b01; OutASel = 2'b00; OutBSel = 2'b01; #100; //Increment Reg2 and Reg3 Look Reg1 and Reg2
    I = 8'd15; RegSel = 4'b0000; FunSel = 2'b11; OutASel = 2'b10; OutBSel = 2'b01; #100; //Reset
    I = 8'd25; RegSel = 4'b1001; FunSel = 2'b10; OutASel = 2'b00; OutBSel = 2'b10; #100; //Load 25 to Reg 2 and Reg 3 Look Reg 1 and Reg 3
    I = 8'd31; RegSel = 4'b1111; FunSel = 2'b01; OutASel = 2'b10; OutBSel = 2'b00; #100; //Do not change
    I = 8'd31; RegSel = 4'b1111; FunSel = 2'b11; OutASel = 2'b10; OutBSel = 2'b00; #100; //Do not change

    
    
    $finish;
    end
    always begin
    clock = ~ clock ; #25;
    end
endmodule

module Part2b_simulation();
    reg clock;
    reg [1:0] FunSel;
    reg [7:0] I;
    reg [1:0] OutCSel;
    reg [1:0] OutDSel;
    wire [7:0] OutC;
    wire [7:0] OutD;
    reg [2:0] RegSel; 
    
    AddressRegisterFile part2btest(.I(I),.clock(clock), .OutCSel(OutCSel), .OutDSel(OutDSel), .FunSel(FunSel), .RegSel(RegSel), .OutC(OutC), .OutD(OutD));
                         
    initial begin
    clock = 0; 
    I = 8'd10; RegSel = 3'b000; FunSel = 2'b11; OutCSel = 2'b10; OutDSel = 2'b11; #100; //Reset
    I = 8'd15; RegSel = 3'b000; FunSel = 2'b10; OutCSel = 2'b00; OutDSel = 2'b10; #100; //Load 15 to Every Reg
    I = 8'd0; RegSel = 3'b011; FunSel = 2'b00; OutCSel = 2'b00; OutDSel = 2'b10;  #100; //Decrement only PC OutC PC OutD AR
    I = 8'd0; RegSel = 3'b000; FunSel = 2'b01; OutCSel = 2'b00; OutDSel = 2'b10;  #100; //Increment Every Reg OutC PC OutD AR
    I = 8'd25; RegSel = 3'b111; FunSel = 2'b01; OutCSel = 2'b10; OutDSel = 2'b01; #100; //Do not change
    I = 8'd25; RegSel = 3'b000; FunSel = 2'b11; OutCSel = 2'b10; OutDSel = 2'b11; #100; //Reset Every Reg 
    I = 8'd25; RegSel = 3'b100; FunSel = 2'b10; OutCSel = 2'b00; OutDSel = 2'b10; #100; //Load 25 to AR and SP OutC PC OutD AR
    I = 8'd10; RegSel = 3'b011; FunSel = 2'b01; OutCSel = 2'b00; OutDSel = 2'b10; #100; //Only increment PC OutC PC OutD AR
    
    
    $finish;
    end
    always begin
    clock = ~ clock ; #25;
    end
endmodule
    

module Part2c_simulation();
    reg clock;
    reg [1:0] FunSel;
    reg [7:0] I;
    wire [15:0] IR;
    reg lh;
    reg E;
    
    InstructionRegister uut31(.clock(clock), .I(I), .lh(lh), .E(E), .FunSel(FunSel), .IR(IR));
    
    initial begin
    clock = 0; 
    I = 8'd10; E = 1; FunSel = 2'b11; lh = 0; #100; //Reset
    I = 8'd10; E = 0; FunSel = 2'b10; lh = 1; #100; //E 0
    I = 8'b0000_1111; E = 1; FunSel = 2'b10; lh = 1; #100; //Load b0000_1111 to lowest bits
    I = 8'b1111_0000; E = 1; FunSel = 2'b10; lh = 0;  #100; //Load b1111_0000 to highest bits
    I = 8'd31; E = 1; FunSel = 2'b00; lh = 0; #100; //Decrement
    I = 8'd31; E = 1; FunSel = 2'b01; lh = 0; #100; //Increment
    I = 8'd31; E = 0; FunSel = 2'b00; lh = 0; #100; //E = 0
    I = 8'd31; E = 0; FunSel = 2'b01; lh = 0; #100; //E = 0
    I = 8'd31; E = 1; FunSel = 2'b11; lh = 0; #100; //Reset
    
    
    $finish;
    end
    always begin
    clock = ~ clock ; #25;
    end
endmodule

module Part3_ALU_Simulation();
    reg [7:0] A;
    reg [7:0] B;
    reg  [3:0] FunSel;
    
    wire [7:0] OutALU;
    wire [3:0] OutFlag;
    
    ALU uut(A, B, FunSel, OutALU, OutFlag);
    
    initial begin
        FunSel = 4'b0000; A = 8'd15;        B = 8'd12; #50;         // Set A
        FunSel = 4'b0001; A = 8'd15;        B = 8'd12; #50;         // Set B
        FunSel = 4'b0010; A = 4'b1010;      B = 8'd0; #50;          // NOT A
        FunSel = 4'b0011; A = 8'd1;         B = 4'b0101; #50;       // Not B
        FunSel = 4'b0100; A = 8'd15;        B = 8'd17; #50;         // A + B
        FunSel = 4'b0100; A = 8'b0111_1111; B = 8'b0000_0001; #50;  // A + B with overflow
        FunSel = 4'b0101; A = 8'd15;        B = 8'd15; #50;         // A + B + Cin
        FunSel = 4'b0110; A = 8'd15;        B = 8'd10; #50;         // A - B
        FunSel = 4'b0111; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // A AND B
        FunSel = 4'b1000; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // A OR B
        FunSel = 4'b1001; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // A XOR B
        FunSel = 4'b1010; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // LSL A
        FunSel = 4'b1011; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // LSR A
        FunSel = 4'b1100; A = 8'b1101_0011; B = 8'b0011_1001; #50;  // ASL A
        FunSel = 4'b1100; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // ASL A with overflow
        FunSel = 4'b1101; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // ASR A
        FunSel = 4'b1110; A = 8'b1101_0011; B = 8'b0011_1001; #50;  // CSL A
        FunSel = 4'b1110; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // CSL A with overflow
        FunSel = 4'b1111; A = 8'b1001_0011; B = 8'b0011_1001; #50;  // CSR A
        FunSel = 4'b1111; A = 8'b1001_0010; B = 8'b0011_1001; #50;  // CSR A with overflow
        $finish;
    end
endmodule

module Part4_TestBench();
    //Input Registers of ALUSystem
    reg[1:0] RF_OutASel; 
    reg[1:0] RF_OutBSel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RegSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutCSel; 
    reg[1:0] ARF_OutDSel; 
    reg[1:0] ARF_FunSel;
    reg[2:0] ARF_RegSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0]      IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg MuxCSel;
    reg      Clock;
    
    //Test Bench Connection of ALU System
    ALUSystem _ALUSystem(
    .RF_OutASel(RF_OutASel), 
    .RF_OutBSel(RF_OutBSel), 
    .RF_FunSel(RF_FunSel),
    .RF_RegSel(RF_RegSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutCSel), 
    .ARF_OutDSel(ARF_OutDSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RegSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock)
    );
    
    //Test Vector Variables
    reg [31:0] VectorNum, Errors, TotalLine; 
    reg [34:0] TestVectors[10000:0];
    reg Reset, Operation;
    
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
                    
                #1; //remove this line to apply vector immediately, otherwise it will show the results of the last input in the negedge section
                {Operation, RF_OutASel, RF_OutBSel, RF_FunSel, 
                RF_RegSel, ALU_FunSel, ARF_OutCSel, ARF_OutDSel, 
                ARF_FunSel, ARF_RegSel, IR_LH, IR_Enable, IR_Funsel, 
                Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if(~(TestVectors[VectorNum] === 35'bx))begin
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: OutASel: %d, OutBSel: %d, FunSel: %b, Regsel: %b", RF_OutASel, RF_OutBSel, RF_FunSel, RF_RegSel);            
            $display("ALU FunSel: %b", ALU_FunSel);
            $display("Addres Register File: OutCSel: %d, OutDSel: %d, FunSel: %b, Regsel: %b", ARF_OutCSel, ARF_OutDSel, ARF_FunSel, ARF_RegSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %b", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %b, MuxBSel: %b, MuxCSel: %b", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Ouput Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %b, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: COut: %d, DOut (Address): %d", _ALUSystem.ARF_COut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            $display("");
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 35'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                #15; // wait some time to see the last input's effect
                $finish; // End simulation
            end
        end
        end
    
endmodule


module Project2Sim();
    reg      Clock;
    reg reset;
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //reset at start
    initial 
    begin
        reset = 1; #10; reset = 0;
    end
    
    //total operation takes 1085 ns, if testing a code different from given in the hw, delete or change the waiting time
    initial
    begin
        #1100; $finish;
    end
    
    //print after each timerFunSel + 10ns
    //this is not reliable and might show results that will be updated in the next tick, but at that time opcode etc. resets
    //correct results can be seen from the simulation graph
    
    //always @(posedge comp.timerFunSel)
    //begin
    //    #10;
    //    $display("Register File: OutASel: %d, OutBSel: %d, FunSel: %b, Regsel: %b", comp.RF_OutASel, comp.RF_OutBSel, comp.RF_FunSel, comp.RF_RegSel);            
    //    $display("ALU FunSel: %b", comp.ALU_FunSel);
    //    $display("Addres Register File: OutCSel: %d, OutDSel: %d, FunSel: %b, Regsel: %b", comp.ARF_OutCSel, comp.ARF_OutDSel, comp.ARF_FunSel, comp.ARF_RegSel);            
    //    $display("Instruction Register: LH: %d, Enable: %d, FunSel: %b", comp.IR_LH, comp.IR_Enable, comp.IR_Funsel);            
    //    $display("Memory: WR: %d, CS: %d", comp.Mem_WR, comp.Mem_CS);
    //    $display("MuxASel: %b, MuxBSel: %b, MuxCSel: %b", comp.MuxASel, comp.MuxBSel, comp.MuxCSel);
    //    
    //    $display("");
    //    $display("Register File: AOut: %d, BOut: %d", comp.AOut, comp.BOut);            
    //    $display("ALUOut: %d, ALUOutFlag: %b, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", comp.ALUOut, comp.ALUOutFlag, comp.ALUOutFlag[3],comp.ALUOutFlag[2],comp.ALUOutFlag[1],comp.ALUOutFlag[0]);
    //    $display("Address Register File: COut: %d, DOut (Address): %d", comp.ARF_COut, comp.Address);            
    //    $display("Memory Out: %d", comp.MemoryOut);            
    //    $display("IROut: %h", comp.IROut);
    //    $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", comp.MuxAOut, comp.MuxBOut, comp.MuxCOut);
    //    
    //    $display("");
    //    $display("opcode: %h", comp.opcode);
    //    if(comp.mem_ref_op)
    //    begin
    //        $display("adressing: %b", comp.addressing);   
    //        $display("op address: %h", comp.op_address);
    //        $display("regsel: %h", comp.regsel);
    //    end
    //    else
    //    begin
    //        $display("destreg: %b, srcreg1: %b, srcreg2: %b", comp.destreg, comp.srcreg1, comp.srcreg2);
    //    end
    //    $display("R1: %h, R2: %h, R3: %h, R4: %h", comp.alusys.rf.Reg1, comp.alusys.rf.Reg2, comp.alusys.rf.Reg3, comp.alusys.rf.Reg4);
    //    $display("PC: %h, AR: %h, SP: %h", comp.alusys.address_rf.RegPC, comp.alusys.address_rf.RegAR, comp.alusys.address_rf.RegSP);
        
    //    $display("");
    //    $display("");
    //end
    
    BasicComputer comp(Clock, reset);

endmodule