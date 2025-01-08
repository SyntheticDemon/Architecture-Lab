`// ============================================================================
// Copyright (c) 2012 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//
//
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// ============================================================================
//
// Major Functions:	DE2 TOP LEVEL
//
// ============================================================================
//
// Revision History :
// ============================================================================
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Johnny Chen       :| 05/08/19  :|      Initial Revision
//   V1.1 :| Johnny Chen       :| 05/11/16  :|      Added FLASH Address FL_ADDR[21:20]
//   V1.2 :| Johnny Chen       :| 05/11/16  :|		Fixed ISP1362 INT/DREQ Pin Direction.   
//   V1.3 :| Johnny Chen       :| 06/11/16  :|		Added the Dedicated TV Decoder Line-Locked-Clock Input
//													            for DE2 v2.X PCB.
//   V1.5 :| Eko    Yan        :| 12/01/30  :|      Update to version 11.1 sp1.
// ============================================================================
`define WORD_WIDTH 32
`define REG_FILE_DEPTH 4
`define REG_FILE_SIZE 16
`define SIGNED_IMM_WIDTH 24
`define SHIFTER_OPERAND_WIDTH 12

`define CLOCK_PERIOD 20

module ARM
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		//UART_TXD,						//	UART Transmitter
		//UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		//IRDA_TXD,						//	IRDA Transmitter
		//IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		//SD_DAT,							//	SD Card Data
		//SD_WP_N,						   //	SD Write protect 
		//SD_CMD,							//	SD Card Command Signal
		//SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	   TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		TD_CLK27,                  //	TV Decoder 27MHz CLK
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input		   	CLOCK_27;				//	27 MHz
input		   	CLOCK_50;				//	50 MHz
input			   EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	   [3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	  [17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output  [17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
//output			UART_TXD;				//	UART Transmitter
//input			   UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
//output			IRDA_TXD;				//	IRDA Transmitter
//input			   IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	  [15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output  [11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	  [7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output [21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	 [15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output [17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	 [15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output  [1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			   OTG_INT0;				//	ISP1362 Interrupt 0
input			   OTG_INT1;				//	ISP1362 Interrupt 1
input			   OTG_DREQ0;				//	ISP1362 DMA Request 0
input			   OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	  [7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
//inout	 [3:0]	SD_DAT;					//	SD Card Data
//input			   SD_WP_N;				   //	SD write protect
//inout			   SD_CMD;					//	SD Card Command Signal
//output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			   I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	   PS2_DAT;				//	PS2 Data
input			   PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			   ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			   AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			   AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			   AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			   AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	 [7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			   TD_HS;					//	TV Decoder H_SYNC
input			   TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
input          TD_CLK27;            //	TV Decoder 27MHz CLK
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1
  wire clk;
  wire rst;
  wire enableForwarding;

  wire [`WORD_WIDTH-1:0] IF_stage_pc_out;
  wire [`WORD_WIDTH-1:0] IF_stage_instruction_out;
  wire [`WORD_WIDTH-1:0] branch_address;
  wire EXE_stage_B_out;
  wire hazard_detected;

  IF_Stage  IF_Stage_Inst (
   .clk(clk),
   .rst(rst),
   .freeze(hazard_detected),
   .Branch_taken(EXE_stage_B_out),
   .BranchAddr(branch_address),
   .PC(IF_stage_pc_out),
   .Instruction(IF_stage_instruction_out)
  );

  wire [`WORD_WIDTH-1:0] IF_reg_pc_out;
  wire [`WORD_WIDTH-1:0] IF_reg_instruction_out;

  IF_Stage_Reg  IF_Reg_Inst (
   .clk(clk),
   .rst(rst),
   .freeze(hazard_detected),
   .flush(EXE_stage_B_out),
   .PC_in(IF_stage_pc_out),
   .Instruction_in(IF_stage_instruction_out),
   .PC(IF_reg_pc_out),
   .Instruction(IF_reg_instruction_out)
  );

  //wire [`WORD_WIDTH-1:0]              ID_stage_pc_out;
  //wire [`WORD_WIDTH-1:0]              ID_stage_instruction_out;
    wire [`REG_FILE_DEPTH-1:0]              ID_stage_reg_file_dst;
  wire [`REG_FILE_DEPTH-1:0]                ID_stage_reg_file_src1, ID_stage_reg_file_src2;
    wire [`WORD_WIDTH-1:0]                      ID_stage_val_Rn, ID_stage_val_Rm;
    wire [`SIGNED_IMM_WIDTH-1:0]            ID_stage_signed_immediate;
    wire [`SHIFTER_OPERAND_WIDTH-1:0] ID_stage_shifter_operand;
    wire [3:0]                                              ID_stage_EX_command_out;
    wire [3:0]              status;
    wire ID_stage_mem_read_out, ID_stage_mem_write_out,
        ID_stage_WB_en_out,
        ID_stage_Imm_out,
        ID_stage_B_out,
        ID_stage_SR_update_out;

  wire [`REG_FILE_DEPTH-1:0] WB_Stage_dst_out;
  wire [`WORD_WIDTH-1:0] WB_Value;
  wire WB_Stage_WB_en_out;
  wire twoSrc;
  ID_Stage ID_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .hazard(hazard_detected),
    //.PC_IN(IF_reg_pc_out),
    .Instruction(IF_reg_instruction_out),
    .Dest_wb(WB_Stage_dst_out),
      .Result_WB(WB_Value),
      .writeBackEn(WB_Stage_WB_en_out),
    .SR(status),
    //.PC(ID_stage_pc_out),
    //.instruction(ID_stage_instruction_out),
      .Dest(ID_stage_reg_file_dst),
      .Val_Rn(ID_stage_val_Rn), .Val_Rm(ID_stage_val_Rm),
      .Signed_imm_24(ID_stage_signed_immediate),
      .Shift_operand(ID_stage_shifter_operand),
      .EXE_CMD(ID_stage_EX_command_out),
      .MEM_R_EN(ID_stage_mem_read_out), .MEM_W_EN(ID_stage_mem_write_out),
        .WB_EN(ID_stage_WB_en_out),
        .imm(ID_stage_Imm_out),
        .B(ID_stage_B_out),
        .S(ID_stage_SR_update_out),
        .Two_src(twoSrc),
        .src1(ID_stage_reg_file_src1),
    .src2(ID_stage_reg_file_src2)
  );

  wire [`WORD_WIDTH-1:0] ID_reg_pc_out;
  //wire [`WORD_WIDTH-1:0] ID_reg_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] ID_reg_reg_file_dst_out;
  wire [`WORD_WIDTH-1:0] ID_reg_val_Rn_out, ID_reg_val_Rm_out;
  wire [`SIGNED_IMM_WIDTH-1:0] ID_reg_signed_immediate_out;
  wire [`SHIFTER_OPERAND_WIDTH-1:0] ID_reg_shifter_operand_out;
  wire [3:0] ID_reg_SR_out, ID_reg_EX_command_out;
  wire ID_reg_mem_read_out, ID_reg_mem_write_out,
    ID_reg_WB_en_out,
    ID_reg_Imm_out,
    ID_reg_B_out,
    ID_reg_SR_update_out;
  wire [`REG_FILE_DEPTH-1:0] ID_reg_reg_file_src1, ID_reg_reg_file_src2;

  ID_Stage_Reg ID_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .flush(EXE_stage_B_out),
    .PC_IN(IF_reg_pc_out),
    //.instruction_in(ID_stage_instruction_out),
    .Dest_IN(ID_stage_reg_file_dst),
      .Val_Rn_IN(ID_stage_val_Rn), .Val_Rm_IN(ID_stage_val_Rm),
      .Signed_imm_24_IN(ID_stage_signed_immediate),
      .Shift_operand_IN(ID_stage_shifter_operand),
    .EXE_CMD_IN(ID_stage_EX_command_out),
      .MEM_R_EN_IN(ID_stage_mem_read_out), .MEM_W_EN_IN(ID_stage_mem_write_out),
        .WB_EN_IN(ID_stage_WB_en_out),
        .imm_IN(ID_stage_Imm_out),
        .B_in(ID_stage_B_out),
        .S_in(ID_stage_SR_update_out),
    .reg_file_src1_in(ID_stage_reg_file_src1),
    .reg_file_src2_in(ID_stage_reg_file_src2),
    .PC(ID_reg_pc_out),
    //.instruction(ID_reg_instruction_out),
    .Dest(ID_reg_reg_file_dst_out),
      .Val_Rn(ID_reg_val_Rn_out), .Val_Rm(ID_reg_val_Rm_out),
      .Signed_imm_24(ID_reg_signed_immediate_out),
      .Shift_operand(ID_reg_shifter_operand_out),
    .EXE_CMD(ID_reg_EX_command_out),
      .MEM_R_EN(ID_reg_mem_read_out), .MEM_W_EN(ID_reg_mem_write_out),
        .WB_EN(ID_reg_WB_en_out),
        .imm(ID_reg_Imm_out),
        .B_out(ID_reg_B_out),
    .S_out(ID_reg_SR_update_out),
    .SR_in(status),
    .SR_out(ID_reg_SR_out),
    .reg_file_src1_out(ID_reg_reg_file_src1),
    .reg_file_src2_out(ID_reg_reg_file_src2)
  );

  wire [`WORD_WIDTH-1:0] EXE_stage_pc_out;
  wire [`WORD_WIDTH-1:0] EXE_stage_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] EXE_stage_reg_file_dst_out;
  wire [`WORD_WIDTH-1:0] EXE_stage_val_Rm_out;
  wire [3:0] EXE_stage_SR_out;
  wire [`WORD_WIDTH-1:0] ALU_res;
  wire EXE_stage_mem_read_out, EXE_stage_mem_write_out,
    EXE_stage_WB_en_out;

  wire [1:0] EXE_sel_src1, EXE_sel_src2;
  wire [`WORD_WIDTH-1:0] Mem_Stage_ALU_res_out;

  EXE_Stage EXE_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .pc_in(ID_reg_pc_out),
    //.instruction_in(ID_reg_instruction_out),
    .MEM_stage_val(Mem_Stage_ALU_res_out),
    .WB_stage_val(WB_Value),
    .signed_immediate(ID_reg_signed_immediate_out),
    .EX_command(ID_reg_EX_command_out),
    .SR_in(ID_reg_SR_out),
    .shifter_operand(ID_reg_shifter_operand_out),
    .dst_in(ID_reg_reg_file_dst_out),
    .mem_read_in(ID_reg_mem_read_out), .mem_write_in(ID_reg_mem_write_out),
    .imm(ID_reg_Imm_out),
    .WB_en_in(ID_reg_WB_en_out),
    .B_in(ID_reg_B_out),
    .val_Rn_in(ID_reg_val_Rn_out), .val_Rm_in(ID_reg_val_Rm_out),

    .sel_src1(EXE_sel_src1),
    .sel_src2(EXE_sel_src2),

    .dst_out(EXE_stage_reg_file_dst_out),
    .SR_out(EXE_stage_SR_out),
    .ALU_res(ALU_res),
    .val_Rm_out(EXE_stage_val_Rm_out),
    .branch_address(branch_address),
    .mem_read_out(EXE_stage_mem_read_out), .mem_write_out(EXE_stage_mem_write_out),
    .WB_en_out(EXE_stage_WB_en_out),
    .B_out(EXE_stage_B_out),
    .pc(EXE_stage_pc_out)//,
    //.instruction(EXE_stage_instruction_out)
  );

  wire [`WORD_WIDTH-1:0] EXE_reg_pc_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_instruction_out;
  wire [`REG_FILE_DEPTH-1:0] EXE_reg_dst_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_ALU_res_out;
  wire [`WORD_WIDTH-1:0] EXE_reg_val_Rm_out;
  wire EXE_reg_mem_read_out, EXE_reg_mem_write_out, EXE_reg_WB_en_out;

  EXE_Stage_Reg EXE_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .pc_in(EXE_stage_pc_out),
    //.instruction_in(EXE_stage_instruction_out),
    .dst_in(EXE_stage_reg_file_dst_out),
    .mem_read_in(EXE_stage_mem_read_out), .mem_write_in(EXE_stage_mem_write_out),
    .WB_en_in(EXE_stage_WB_en_out),
    .val_Rm_in(EXE_stage_val_Rm_out),
    .ALU_res_in(ALU_res),
    .dst_out(EXE_reg_dst_out),
    .ALU_res_out(EXE_reg_ALU_res_out),
    .val_Rm_out(EXE_reg_val_Rm_out),
    .mem_read_out(EXE_reg_mem_read_out), .mem_write_out(EXE_reg_mem_write_out),
    .WB_en_out(EXE_reg_WB_en_out),
    .pc(EXE_reg_pc_out)//,
    //.instruction(EXE_reg_instruction_out)
  );

  wire [`REG_FILE_DEPTH-1:0] Mem_Stage_dst_out;
  wire [`WORD_WIDTH-1:0] Mem_Stage_mem_out;
  wire Mem_Stage_read_out, Mem_Stage_WB_en_out;

  Mem_Stage Mem_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .dst(EXE_reg_dst_out),
    .ALU_res(EXE_reg_ALU_res_out),
    .val_Rm(EXE_reg_val_Rm_out),
    .mem_read(EXE_reg_mem_read_out),
    .mem_write(EXE_reg_mem_write_out),
    .WB_en(EXE_reg_WB_en_out),

    .dst_out(Mem_Stage_dst_out),
    .ALU_res_out(Mem_Stage_ALU_res_out),
    .mem_out(Mem_Stage_mem_out),
    .mem_read_out(Mem_Stage_read_out),
    .WB_en_out(Mem_Stage_WB_en_out)
  );

  wire [`REG_FILE_DEPTH-1:0] Mem_Reg_dst_out;
  wire [`WORD_WIDTH-1:0] Mem_Reg_ALU_res_out;
  wire [`WORD_WIDTH-1:0] Mem_Reg_mem_out;
  wire Mem_Reg_read_out, Mem_Reg_WB_en_out;

  MEM_Reg Mem_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .dst(Mem_Stage_dst_out),
    .ALU_res(Mem_Stage_ALU_res_out),
    .mem(Mem_Stage_mem_out),
    .mem_read(Mem_Stage_read_out),
    .WB_en(Mem_Stage_WB_en_out),

    .dst_out(Mem_Reg_dst_out),
    .ALU_res_out(Mem_Reg_ALU_res_out),
    .mem_out(Mem_Reg_mem_out),
    .mem_read_out(Mem_Reg_read_out),
    .WB_en_out(Mem_Reg_WB_en_out)
  );

  WB_Stage WB_Stage_Inst(
    .clk(clk),
    .rst(rst),
    .dst(Mem_Reg_dst_out),
    .ALU_res(Mem_Reg_ALU_res_out),
    .mem(Mem_Reg_mem_out),
    .mem_read(Mem_Reg_read_out),
    .WB_en(Mem_Reg_WB_en_out),
    .WB_Dest(WB_Stage_dst_out),
    .WB_en_out(WB_Stage_WB_en_out),
    .WB_Value(WB_Value)
  );

  Status_Reg Status_Reg_Inst(
    .clk(clk),
    .rst(rst),
    .load(ID_reg_SR_update_out),
    .status_in(EXE_stage_SR_out),
    .status(status)
  );

  wire EXE_WB_en = ID_reg_WB_en_out;
  wire MEM_WB_en = EXE_reg_WB_en_out;
  wire[`REG_FILE_DEPTH-1:0] EXE_dest = ID_reg_reg_file_dst_out;
  wire[`REG_FILE_DEPTH-1:0] MEM_dest = EXE_reg_dst_out;

  HazardUnit Hazard_Detection_Unit_Inst(
    .forwardEn(enableForwarding),
    .rn(ID_stage_reg_file_src1),
    .rdm(ID_stage_reg_file_src2),
    .destEx(EXE_dest),
    .destMem(MEM_dest),
    .wbEnEx(EXE_WB_en),
    .wbEnMem(MEM_WB_en),
    .memREn(EXE_stage_mem_read_out),
    .twoSrc(twoSrc),
    .hazard(hazard_detected)
  );

  Forwarding_Unit Forwarding_Unit_Inst(
    .enable(enableForwarding),
    .src1(ID_reg_reg_file_src1),
    .src2(ID_reg_reg_file_src2),
    .MEM_dest(MEM_dest),
    .WB_dest(WB_Stage_dst_out),
    .MEM_WB_en(MEM_WB_en),
    .WB_WB_en(WB_Stage_WB_en_out),
    .sel_src1(EXE_sel_src1),
    .sel_src2(EXE_sel_src2)
  );

assign rst = SW[2];
assign clk = CLOCK_50;
assign enableForwarding = SW[3];
assign LEDR[0] = SW[0] | SW[1];
endmodule