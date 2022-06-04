`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.05.2022 12:51:22
// Design Name: 
// Module Name: modules
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


module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration o?f the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule


module mux1to2_8bit( select, A ,B, Q );
    input wire select;
    input wire [7:0] A,B;
    output reg [7:0]Q;
    
    always @(select or A or B)
    begin
        case(select)
            1'b0:  begin
                Q <= A;
                end
            1'b1   :  begin
                Q <= B;
                end
        endcase
    end

endmodule

module mux2to4_8bit( select, A ,B, C, D, Q );

    input wire [1:0] select;
    input wire [7:0] A,B,C,D;
    output reg [7:0]Q;
    
    always @(select or A or B or C or D)
    begin
        case(select)
            2'b00:  begin
                Q <= A;
                end
            2'b01   :  begin
                Q <= B;
                end
            2'b10   :  begin
                Q <= C;
                end
            2'b11   :  begin
                Q <= D;
                end
        endcase
    end

endmodule

module reg_8_bit(E , clock, FunSel, I, Q);
    input wire clock;
    input wire E;
    input wire [1:0] FunSel;
    input wire [7:0] I;
    output reg [7:0] Q;
    
    always @(posedge clock)
    
    begin
        if(!E)
        begin
            Q <= Q;
        end
        else
        begin
        case(FunSel)
            2'b00:  begin
                        Q <= Q - 1;
                        end
            2'b01   :  begin
                        Q <= Q + 1;
                        end
            2'b10   :  begin
                        Q <= I;
                        end
            2'b11   :  begin
                        Q <= 0;
                        end  
        endcase
        end   
    end                                  
endmodule

module reg_16_bit(E , clock, FunSel, I, Q);
    input wire clock;
    input wire E;
    input wire [1:0] FunSel;
    input wire [15:0] I;
    output reg [15:0] Q;
    
    always @(posedge clock)
    
    begin
        if(!E)
        begin
            Q <= Q;
        end
        else
        begin
        case(FunSel)
            2'b00:  begin
                        Q <= Q - 1;
                        end
            2'b01   :  begin
                        Q <= Q + 1;
                        end
            2'b10   :  begin
                        Q <= I;
                        end
            2'b11   :  begin
                        Q <= 0;
                        end  
        endcase
        end   
    end                                  
endmodule


module RegisterFile(I,clock, OutASel, OutBSel, FunSel, RegSel, OutA, OutB);
    input wire clock;
    input wire [7:0] I;
    input wire [1:0] FunSel;
    input wire [3:0] RegSel;
    input wire [1:0] OutASel;
    input wire [1:0] OutBSel;
    wire [7:0] Reg1;
    wire [7:0] Reg2;
    wire [7:0] Reg3;
    wire [7:0] Reg4;
    output reg [7:0] OutA;
    output reg [7:0] OutB;
    
    
    reg_8_bit reg1(.E(!RegSel[3]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(Reg1));
    reg_8_bit reg2(.E(!RegSel[2]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(Reg2));
    reg_8_bit reg3(.E(!RegSel[1]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(Reg3));
    reg_8_bit reg4(.E(!RegSel[0]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(Reg4));
    
    always @(*)
    begin
    case(OutASel)
       2'b00:  begin
                        OutA = Reg1;
                        end
            2'b01   :  begin
                        OutA = Reg2;
                        end
            2'b10   :  begin
                        OutA = Reg3;
                        end
            2'b11   :  begin
                        OutA = Reg4;
                        end  
       endcase
       case(OutBSel)
       2'b00:  begin
                        OutB = Reg1;
                        end
            2'b01   :  begin
                        OutB = Reg2;
                        end
            2'b10   :  begin
                        OutB = Reg3;
                        end
            2'b11   :  begin
                        OutB = Reg4;
                        end  
       endcase
       end
       
 endmodule
 
 module AddressRegisterFile(I,clock, OutCSel, OutDSel, FunSel, RegSel, OutC, OutD);
    input wire clock;
    input wire [7:0] I;
    input wire [1:0] FunSel;
    input wire [2:0] RegSel;
    input wire [1:0] OutCSel;
    input wire [1:0] OutDSel;
    wire [7:0] RegPC;
    wire [7:0] RegAR;
    wire [7:0] RegSP;
    output reg [7:0] OutC;
    output reg [7:0] OutD;
    
    
    reg_8_bit PC(.E(!RegSel[2]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(RegPC));
    reg_8_bit AR(.E(!RegSel[1]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(RegAR));
    reg_8_bit SP(.E(!RegSel[0]) , .clock(clock), .FunSel(FunSel), .I(I), .Q(RegSP));    
    always @(*)
    begin
    case(OutCSel)
       2'b00:  begin
                        OutC = RegPC;
                        end
            2'b01   :  begin
                        OutC = RegPC;
                        end
            2'b10   :  begin
                        OutC = RegAR;
                        end
            2'b11   :  begin
                        OutC = RegSP;
                        end  
       endcase
       case(OutDSel)
       2'b00:  begin
                        OutD = RegPC;
                        end
            2'b01   :  begin
                        OutD = RegPC;
                        end
            2'b10   :  begin
                        OutD = RegAR;
                        end
            2'b11   :  begin
                        OutD = RegAR;
                        end  
       endcase
       end
       
 endmodule

module InstructionRegister(clock, I, lh, E, FunSel, IR);
    input wire E;
    input wire clock;
    input wire [7:0] I;
    reg [15:0] passthis;
    input wire [1:0] FunSel;
    input wire lh;
    output wire [15:0] IR;
    

    always @(*)
    begin
    case(lh)
    1'b0:   begin
            passthis = {I,IR[7:0]};
            end
    1'b1:    begin
            passthis = {IR[15:8],I};
            end
     endcase
     

     end
     reg_16_bit idunno(.E(E) , .clock(clock), .FunSel(FunSel), .I(passthis), .Q(IR));

 endmodule

//our ALU doesn't use a clock
module ALU(
    input wire [7:0] A,
    input wire [7:0] B,
    input wire [3:0] FunSel,
    output wire [7:0] OutALU,
    output reg [3:0] OutFlag
    );
    reg [7:0] OutALUReg;
 
    always @(*)
    begin
        
        // Function Operations
        case (FunSel)
            4'b0000:    begin // A
                                    OutALUReg = A;
                                end
            4'b0001:    begin // B
                                    OutALUReg = B;
                                end
            4'b0010:    begin // NOT A
                                    OutALUReg = ~A;
                                end
            4'b0011:     begin // NOT B
                                    OutALUReg = ~B;
                                end
            4'b0100:     begin // A + B
                                    {OutFlag[2], OutALUReg} = A + B;
                                    if (((A[7] == 0) & (B[7] == 0) & (OutALUReg[7] == 1)) | ((A[7] == 1) & (B[7] == 1) & (OutALUReg[7] == 0)))
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                end
            4'b0101:     begin // A + B + Carry
                                    {OutFlag[2], OutALUReg} = A + B + OutFlag[2];
                                    if (((A[7] == 0) & (B[7] == 0) & (OutALUReg[7] == 1)) | ((A[7] == 1) & (B[7] == 1) & (OutALUReg[7] == 0)))
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                end
            4'b0110:     begin // A - B
                                    {OutFlag[2], OutALUReg} = A - B;
                                    if (((A[7] == 0) & (B[7] == 1) & (OutALUReg[7] == 1)) | ((A[7] == 1) & (B[7] == 0) & (OutALUReg[7] == 0)))
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                end
            4'b0111:     begin // A AND B
                                    OutALUReg = A & B;
                                end
            4'b1000:     begin // A OR B
                                    OutALUReg = A | B;
                                end
            4'b1001:     begin // A XOR B
                                    OutALUReg = A ^ B;
                                end
            4'b1010:     begin // LSL A
                                    OutFlag[2] = A[7]; // Set C flag to MSB
                                    OutALUReg = A << 1;
                                end
            4'b1011:     begin // LSR A
                                    OutFlag[2] = A[0]; // Set C flag to LSB
                                    OutALUReg = A >> 1;
                                end
            4'b1100:     begin // ASL A
                                    if (A[7] != A[6])
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                    OutALUReg = A <<< 1;
                                end
            4'b1101:     begin // ASR A
                                    OutALUReg = A >>> 1;
                                end
            4'b1110:     begin // CSL A
                                    if (A[7] != A[6])
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                    OutFlag[2] = A[7]; // Set C flag to MSB
                                    OutALUReg = A << 1; // Shift A to Left
                                    OutALUReg[0] = OutFlag[2]; // Set the LSB to the value in carry flag
                                end
            4'b1111:     begin // CSR A
                                    if (A[7] != A[0])
                                    begin
                                        OutFlag[0] = 1;
                                    end
                                    else
                                    begin
                                        OutFlag[0] = 0;
                                    end
                                    OutFlag[2] = A[0]; // Set C flag to LSB
                                    OutALUReg = A >> 1; // Shift A to Right
                                    OutALUReg[7] = OutFlag[2]; // Set the MSB to the value in carry flag
                                end
        endcase
        
        // Set Z flag
        if (OutALUReg == 8'd0) 
        begin
            OutFlag[3] = 1'b1;
        end
        else
        begin
            OutFlag[3] = 1'b0;
        end
        
        // Set N flag
        if ((OutALUReg[7] == 1) & (FunSel != 4'b1101))
        begin
            OutFlag[1] = 1'b1;
        end
        else
        begin
            OutFlag[1] = 1'b0;
        end
    end
    
    assign OutALU = OutALUReg;
endmodule


module ALUSystem(
        RF_OutASel, 
        RF_OutBSel, 
        RF_FunSel,
        RF_RegSel,
        ALU_FunSel,
        ARF_OutCSel, 
        ARF_OutDSel, 
        ARF_FunSel,
        ARF_RegSel,
        IR_LH,
        IR_Enable,
        IR_Funsel,
        Mem_WR,
        Mem_CS,
        MuxASel,
        MuxBSel,
        MuxCSel,
        Clock, 
        AOut,
        BOut,
        ALUOut,
        ALUOutFlag,
        ARF_COut,
        Address,
        MemoryOut,
        IROut,
        MuxAOut,
        MuxBOut,
        MuxCOut
    );
    input wire[1:0] RF_OutASel; 
    input wire[1:0] RF_OutBSel; 
    input wire[1:0] RF_FunSel;
    input wire[3:0] RF_RegSel;
    input wire[3:0] ALU_FunSel;
    input wire[1:0] ARF_OutCSel; 
    input wire[1:0] ARF_OutDSel; 
    input wire[1:0] ARF_FunSel;
    input wire[2:0] ARF_RegSel;
    input wire IR_LH;
    input wire IR_Enable;
    input wire[1:0] IR_Funsel;
    input wire Mem_WR;
    input wire Mem_CS;
    input wire[1:0] MuxASel;
    input wire[1:0] MuxBSel;
    input wire MuxCSel;
    input wire Clock;
       
    output wire [7:0] AOut, BOut;
    output wire [7:0] ALUOut;
    output wire [3:0] ALUOutFlag;
    output wire [7:0] ARF_COut;
    output wire [7:0] Address;
    output wire [7:0] MemoryOut;
    output wire [15:0] IROut;
    output wire [7:0] MuxAOut, MuxBOut, MuxCOut;
    
    mux2to4_8bit muxa(MuxASel,  IROut[7:0], MemoryOut, ARF_COut, ALUOut, MuxAOut);
    
    mux2to4_8bit muxb(MuxBSel, 8'b0, IROut[7:0],MemoryOut, ALUOut ,MuxBOut); //don't care input of B is 0's
    
    mux1to2_8bit muxc(MuxCSel, ARF_COut, AOut, MuxCOut);
    
    ALU alum(MuxCOut, BOut, ALU_FunSel, ALUOut, ALUOutFlag); //ALU doesn't use the clock
    
    Memory mem(Address, ALUOut, Mem_WR, Mem_CS, Clock, MemoryOut);
    
    RegisterFile rf(MuxAOut, Clock, RF_OutASel, RF_OutBSel, RF_FunSel, RF_RegSel, AOut, BOut);
    AddressRegisterFile address_rf(MuxBOut, Clock, ARF_OutCSel, ARF_OutDSel, ARF_FunSel, ARF_RegSel, ARF_COut, Address);
    InstructionRegister ir(Clock, MemoryOut, IR_LH, IR_Enable, IR_Funsel, IROut);
    


endmodule


module clock_4_bit(E, Clock, FunSel, Q);
    input wire Clock;
    input wire E;
    input wire FunSel;
    output reg [3:0] Q;
    
    always @(posedge Clock)
    begin
        if(!E)
        begin
            Q <= Q;
        end
        else
        begin
        case(FunSel)
            1'b0:  begin
                        Q <= Q + 1;
                        end
            1'b1   :  begin
                        Q <= 4'b0;
                        end
        endcase
        end   
    end
endmodule


module HardwiredControlUnit(Clock,
        reset,
        
        IROut, //needed for instruction decoding
        ALUOutFlag,//needed for 0x0f
        
        RF_OutASel, 
        RF_OutBSel, 
        RF_FunSel,
        RF_RegSel,
        ALU_FunSel,
        ARF_OutCSel, 
        ARF_OutDSel, 
        ARF_FunSel,
        ARF_RegSel,
        IR_LH,
        IR_Enable,
        IR_Funsel,
        Mem_WR,
        Mem_CS,
        MuxASel,
        MuxBSel,
        MuxCSel,
        
        opcode,
        addressing,
        regsel, 
        op_address,
        destreg,
        srcreg1,
        srcreg2,
        mem_ref_op,
        
        timerFunSel,
        timerOut
        );
        
    
    input wire Clock;
    input wire reset;
    input wire [15:0] IROut;
    input wire [3:0] ALUOutFlag;
    output reg[1:0] RF_OutASel; 
    output reg[1:0] RF_OutBSel; 
    output reg[1:0] RF_FunSel;
    output reg[3:0] RF_RegSel;
    output reg[3:0] ALU_FunSel;
    output reg[1:0] ARF_OutCSel; 
    output reg[1:0] ARF_OutDSel; 
    output reg[1:0] ARF_FunSel;
    output reg[2:0] ARF_RegSel;
    output reg IR_LH;
    output reg IR_Enable;
    output reg[1:0] IR_Funsel;
    output reg Mem_WR;
    output reg Mem_CS;
    output reg[1:0] MuxASel;
    output reg[1:0] MuxBSel;
    output reg MuxCSel;
    
    output wire [3:0] opcode;
    output wire addressing;
    output wire [7:0] op_address;
    output wire [1:0] regsel;
    output wire [3:0] destreg, srcreg1, srcreg2;
    output wire mem_ref_op;
    


    assign opcode = IROut[15:12];
    assign addressing = IROut[10];
    assign regsel = IROut[9:8];
    assign op_address = IROut[7:0];
    assign mem_ref_op = opcode == 4'h0 || opcode == 4'h1 || opcode == 4'h2 || opcode == 4'hF || opcode == 4'hB || opcode == 4'hC;


    assign destreg = IROut[11:8];
    assign srcreg1 = IROut[7:4];
    assign srcreg2 = IROut[3:0];

    output wire [3:0] timerOut;
    output reg timerFunSel;
    
    //at the end of each instruction, timer and IR should be cleared
    
    always @(*)
    begin
        if(~reset)
        begin
            //default values if inputs are not changed in the section below
            
            RF_FunSel = 2'bZ;
            RF_RegSel = 4'b1111;
            ARF_FunSel = 2'bZ;
            ARF_RegSel = 3'b111;
            IR_LH = 1'bZ;
            IR_Enable = 0;
            IR_Funsel = 1'bZ;
            Mem_WR = 1'bZ;
            Mem_CS = 1'b1;
            //ALU funsel should not be cleared
            //
            
            //first 2 clocks per instruction are instruction fetching, there is no decoding in our hardwired control unit
            //both memory reference and register reference instructions starts directly at t3, since memory are accessible anytime if it is required to be loaded(R1=M[AR] etc.)
            if(timerOut == 4'd0) 
            begin
                //Address = PC
                //Memory Read
                //IR Low = Memory
                //PC = PC +1
                
                ARF_OutDSel= 2'b00; // Address = PC
                ARF_RegSel= 3'b011; // Select PC
                ARF_FunSel= 2'b01; // Increase PC
                timerFunSel = 0; // Timer++
                
                IR_LH= 1; // IR Low
                IR_Enable= 1;
                IR_Funsel= 2'b10; // IR Load
                Mem_CS= 0; //Enable Memory
                Mem_WR= 0; //Read Mode
                
            end
            if(timerOut == 4'd1)
            begin
                //Address = PC
                //Memory Read
                //IR High = Memory
                //PC = PC+1
                ARF_OutDSel= 2'b00; // Address = PC
                ARF_RegSel = 3'b011; // Increase PC
                ARF_FunSel = 2'b01; 
                timerFunSel = 0; // Timer++
                
                IR_LH= 0; // IR High
                IR_Enable= 1;
                IR_Funsel= 2'b10; // IR Load
                Mem_CS= 0; //Enable Memory
                Mem_WR= 0; //Read Mode
                
            end
            if(opcode == 4'h0)
            begin
                if(timerOut == 4'd2)
                begin
                    MuxBSel = 2'b01; //Select Low IR
                    ARF_RegSel = 3'b011; // Select PC
                    ARF_FunSel = 2'b10; // Load Low IR to PC
                    
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end
            end
            if(opcode == 4'h1) // load value to rx
            begin
                if(timerOut == 4'd2)
                begin
                    
                    case(addressing)
                        1'b0:
                        begin
                            Mem_CS= 0; // enable memory
                            Mem_WR= 0; //read mode
                            ARF_OutDSel= 2'b10; // select ar as address
                            MuxASel= 2'b01; // select memory output
                        end
                        1'b1:
                        begin
                            MuxASel= 2'b00; // select IR Low
                        end
                    endcase

                    RF_FunSel= 2'b10; //Load to Rx;

                    case(regsel) //Choose Rx
                        2'b00:  begin
                                RF_RegSel= 4'b0111;
                                end
                        2'b01:  begin
                                RF_RegSel= 4'b1011;
                                end
                        2'b10:  begin
                                RF_RegSel= 4'b1101;
                                end
                        2'b11:  begin
                                RF_RegSel= 4'b1110;
                                end
                       endcase
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end
            end
            if(opcode == 4'h2)//Load rx to memory
            begin
                if(timerOut == 4'd2)
                begin
                    Mem_CS = 0; // Enable memory
                    Mem_WR = 1; // Write Mode
                    ALU_FunSel = 4'b0001; //OUT_ALU is B
                    RF_OutBSel = regsel; //B input will be Rx
                    
                    ARF_OutDSel = 2'b10;
                    
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end    
            end
            if(opcode == 4'hB)
            begin
                if(timerOut == 4'd2)
                begin
                    RF_OutBSel = regsel;
                    ALU_FunSel = 4'b0001; //OUTAlu is B
                    ARF_OutDSel = 2'b11; //SP points memory
                    Mem_CS = 0; // Enable memory
                    Mem_WR = 1; // Write Mode
                    
                    ARF_RegSel = 3'b110; //Enable SP
                    ARF_FunSel = 2'b00; //Decrement ARF
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end
            end
            if(opcode == 4'hC)
            begin  
                if(timerOut == 4'd2)
                begin
                    ARF_OutDSel = 2'b11; //SP points memory
                    Mem_CS = 0; // Enable memory
                    Mem_WR = 0; // Read Mode
                    MuxASel = 2'b01;    //Memory output to RF
                    RF_FunSel = 2'b10;  //Load RF

                    case(regsel) //Choose Rx
                        2'b00:  begin
                            RF_RegSel = 4'b0111;
                                end
                        2'b01:  begin
                                RF_RegSel = 4'b1011;
                                end
                        2'b10:  begin
                                RF_RegSel = 4'b1101;
                                end
                        2'b11:  begin
                                RF_RegSel = 4'b1110;
                                end
                    endcase
                    
                    ARF_FunSel = 2'b01; //Decrement ARF
                    ARF_RegSel = 3'b110;    //Enable SP
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end
            end
            if(opcode == 4'hF)// if z=0 then pc <- value
            begin
                if(timerOut == 4'd2)
                begin
                    if(ALUOutFlag[3] == 0)
                    begin
                        MuxBSel = 2'b01; //Select Low IR
                        ARF_RegSel = 3'b011; // Select PC
                        ARF_FunSel = 2'b10; // Load Low IR to PC
                    end
                    //clear even if z!=0
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end    
                
            end
            

            if(~mem_ref_op)
            begin
                if(timerOut == 4'd2 || (opcode == 4'h8 && timerOut == 4'd3))
                begin
                    //choose destreg
                    case(destreg)
                    //PC
                    4'b0000:    begin
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b011;
                        ARF_FunSel = 2'b10;
                                end
                    4'b0001:    begin
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b011;
                        ARF_FunSel = 2'b10;
                                end
                    //AR
                    4'b0010:    begin
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b101;
                        ARF_FunSel = 2'b10;
                                end
                    //SP    
                    4'b0011:    begin
                        MuxBSel = 2'b11;
                        ARF_RegSel = 3'b110;
                        ARF_FunSel = 2'b10;
                                end
                    
                    //R1
                    4'b0100:    begin
                        MuxASel = 2'b11; 
                        RF_RegSel = 4'b0111;
                        RF_FunSel = 2'b10;
                                end
                    //R2
                    4'b0101:    begin
                        MuxASel = 2'b11; 
                        RF_RegSel = 4'b1011;
                        RF_FunSel = 2'b10;
                                end
                    //R3
                    4'b0110:    begin
                        MuxASel = 2'b11; 
                        RF_RegSel = 4'b1101;
                        RF_FunSel = 2'b10;
                                end
                    //R4
                    4'b0111:    begin
                        MuxASel = 2'b11; 
                        RF_RegSel = 4'b1110;
                        RF_FunSel = 2'b10;
                                end       
                    endcase
                end

                if(timerOut == 4'd2)
                begin
                    RF_OutBSel = srcreg2[1:0];

                    if(srcreg1[2] == 0)
                    begin
                        MuxCSel = 0;//ARF
                        ARF_OutCSel = srcreg1[1:0];
                    end

                    if(srcreg1[2] == 1)
                    begin
                        MuxCSel = 1; //RF
                        RF_OutASel = srcreg1[1:0];
                    end
                    
                    if(opcode == 4'd3)
                    begin
                        ALU_FunSel = 4'b0000; //SrcReg1
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'h4)
                    begin
                        ALU_FunSel = 4'b0111; //AND
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'h5)
                    begin
                        ALU_FunSel = 4'b1000;//OR
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'h6)
                    begin
                        ALU_FunSel = 4'b0010; // ALU NOT srcreg1
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'h7)
                    begin
                        ALU_FunSel = 4'b0100; // ALU A + B
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'h8)
                    begin
                        ALU_FunSel = 4'b0110; // ALU A - B
                        timerFunSel = 0;//have to make NOT(result)+1
                    end
                    if(opcode == 4'h9)
                    begin
                        ALU_FunSel = 4'b1010; // LSL A
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end
                    if(opcode == 4'hA)
                    begin
                        ALU_FunSel = 4'b1011; // LSR A
                        timerFunSel = 1; //clear timer
                        IR_Enable = 1'b1; // enable IR
                        IR_Funsel = 2'b11; // clear IR
                    end  
                    if(opcode == 4'hD || opcode == 4'hE)
                    begin
                        ALU_FunSel = 4'b0000; // ALU A = SrcReg1
                        timerFunSel = 0;//will increment or decrement
                    end
                end
            end
            
            if(opcode == 4'h8)
            begin
                if(timerOut==4'd3)
                begin
                    if(destreg[2] == 0)
                    begin
                        MuxCSel = 0;//ARF
                        ARF_OutCSel = srcreg1[1:0];
                    end
                    if(destreg[2] == 1)
                    begin
                        MuxCSel = 1; //RF
                        RF_OutASel = srcreg1[1:0];
                    end

                    ALU_FunSel = 4'b0010; // Not (A-B)
                    //selecting destreg and enabling load is done above
                    timerFunSel = 0;
                end
                
                if(timerOut==4'd4)
                begin
                    case(destreg)
                    4'b0000:    begin
                        ARF_RegSel = 3'b011;
                        ARF_FunSel = 2'b01; //increment selected Reg
                                end
                    4'b0001:    begin
                        ARF_RegSel = 3'b011;
                        ARF_FunSel = 2'b01; //increment selected Reg
                                end
                    4'b0010:    begin
                        ARF_RegSel = 3'b101;
                        ARF_FunSel = 2'b01; //increment selected Reg
                                end
                    4'b0011:    begin
                        ARF_RegSel = 3'b110;
                        ARF_FunSel = 2'b01; //increment selected Reg
                                end
                    4'b0100:    begin
                        RF_RegSel = 4'b0111;
                        RF_FunSel = 2'b01;
                                end
                    4'b0101:    begin
                        RF_RegSel = 4'b1011;
                        RF_FunSel = 2'b01;
                                end
                    4'b0110:    begin
                        RF_RegSel = 4'b1101;
                        RF_FunSel = 2'b01;
                                end
                    4'b0111:    begin
                        RF_RegSel = 4'b1110;
                        RF_FunSel = 2'b01;
                                end       
                    endcase
                    
                    //ALU A = destreg to get flags from the total result
                    if(destreg[2] == 0)
                    begin
                        MuxCSel = 0;//ARF
                        ARF_OutCSel = srcreg1[1:0];
                    end
                    if(destreg[2] == 1)
                    begin
                        MuxCSel = 1; //RF
                        RF_OutASel = srcreg1[1:0];
                    end
    
                    ALU_FunSel = 4'b0000;
                    
                    timerFunSel = 1; //clear timer
                    IR_Enable = 1'b1; // enable IR
                    IR_Funsel = 2'b11; // clear IR
                end
            end
            
            
            if((opcode == 4'hD || opcode == 4'hE) && timerOut == 4'd3)
            begin
                if(destreg[2] == 1'b0) // if destreg in ARF
                begin
                    if(opcode == 4'hD) //if inc
                    begin
                        ARF_FunSel = 2'b01;
                    end
                    if(opcode == 4'hE) //if dec
                    begin
                        ARF_FunSel = 2'b00;
                    end
                end
                if(destreg[2] == 1'b1) // if destreg in RF
                begin
                    if(opcode == 4'hD) //if inc
                    begin
                        RF_FunSel = 2'b01;
                    end
                    if(opcode == 4'hE) //if dec
                    begin
                        RF_FunSel = 2'b00;
                    end
                end
                
                case(destreg)//enable destreg
                //PC
                4'b0000:    begin
                    ARF_RegSel = 3'b011;
                            end
                //PC
                4'b0001:    begin
                    ARF_RegSel = 3'b011;
                            end
                //AR
                4'b0010:    begin
                    ARF_RegSel = 3'b101;
                            end
                //SP
                4'b0011:    begin
                    ARF_RegSel = 3'b110;
                            end
                //R1
                4'b0100:    begin
                    RF_RegSel = 4'b0111;
                            end
                //R2
                4'b0101:    begin
                    RF_RegSel = 4'b1011;
                            end
                //R3
                4'b0110:    begin
                    RF_RegSel = 4'b1101;
                            end
                //R4
                4'b0111:    begin
                    RF_RegSel = 4'b1110;
                            end       
                endcase
                
                //ALU A = destreg to get flags from the total result
                if(destreg[2] == 0)
                begin
                    MuxCSel = 0;//ARF
                    ARF_OutCSel = srcreg1[1:0];
                end
                if(destreg[2] == 1)
                begin
                    MuxCSel = 1; //RF
                    RF_OutASel = srcreg1[1:0];
                end

                ALU_FunSel = 4'b0000;
                
                timerFunSel = 1; //clear timer
                IR_Enable = 1'b1; // enable IR
                IR_Funsel = 2'b11; // clear IR
            end
        end                   
        else // reset
        begin
            timerFunSel = 1;
            RF_RegSel= 4'b0000;
            RF_FunSel= 2'b11;
            ARF_RegSel= 3'b000;
            ARF_FunSel= 2'b11;
            IR_Enable= 1;
            IR_Funsel= 2'b11;
        end
    
    end
    clock_4_bit timer(1, Clock, timerFunSel, timerOut);

endmodule

module BasicComputer(Clock, reset);
    input wire Clock;
    input wire reset;
    wire[1:0] RF_OutASel; 
    wire[1:0] RF_OutBSel; 
    wire[1:0] RF_FunSel;
    wire[3:0] RF_RegSel;
    wire[3:0] ALU_FunSel;
    wire[1:0] ARF_OutCSel; 
    wire[1:0] ARF_OutDSel; 
    wire[1:0] ARF_FunSel;
    wire[2:0] ARF_RegSel;
    wire IR_LH;
    wire IR_Enable;
    wire[1:0] IR_Funsel;
    wire Mem_WR;
    wire Mem_CS;
    wire[1:0] MuxASel;
    wire[1:0] MuxBSel;
    wire MuxCSel;
       
    wire [7:0] AOut, BOut;
    wire [7:0] ALUOut;
    wire [3:0] ALUOutFlag;
    wire [7:0] ARF_COut;
    wire [7:0] Address;
    wire [7:0] MemoryOut;
    wire [15:0] IROut;
    wire [7:0] MuxAOut, MuxBOut, MuxCOut; 
    
    //wires below are used for debug, they can be deleted from outputs of hcu
    wire [3:0] opcode;
    wire addressing;
    wire [7:0] op_address;
    wire [1:0] regsel;
    wire [3:0] destreg, srcreg1, srcreg2;
    wire mem_ref_op;
    wire timerFunSel;
    wire [3:0] timerOut;
    
    ALUSystem alusys(
            RF_OutASel, 
            RF_OutBSel, 
            RF_FunSel,
            RF_RegSel,
            ALU_FunSel,
            ARF_OutCSel, 
            ARF_OutDSel, 
            ARF_FunSel,
            ARF_RegSel,
            IR_LH,
            IR_Enable,
            IR_Funsel,
            Mem_WR,
            Mem_CS,
            MuxASel,
            MuxBSel,
            MuxCSel,
            Clock, 
            AOut,
            BOut,
            ALUOut,
            ALUOutFlag,
            ARF_COut,
            Address,
            MemoryOut,
            IROut,
            MuxAOut,
            MuxBOut,
            MuxCOut);
    
    HardwiredControlUnit hcu(Clock,
            reset, 
            IROut,
            ALUOutFlag,
            
            RF_OutASel, 
            RF_OutBSel, 
            RF_FunSel,
            RF_RegSel,
            ALU_FunSel,
            ARF_OutCSel, 
            ARF_OutDSel, 
            ARF_FunSel,
            ARF_RegSel,
            IR_LH,
            IR_Enable,
            IR_Funsel,
            Mem_WR,
            Mem_CS,
            MuxASel,
            MuxBSel,
            MuxCSel,
            
            
            opcode,
            addressing,
            regsel, 
            op_address,
            destreg,
            srcreg1,
            srcreg2,
            mem_ref_op,
            
            timerFunSel,
            timerOut);
    
    
    
endmodule