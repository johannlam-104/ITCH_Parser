`timescale 1ns / 1ps

module crc_32_engine(
    input  wire        i_rmii_clk,    // 50 MHz
    input  wire        i_rstn,
    input  wire        i_crc_init,    // reset the crc seed
    input  wire [1:0]  i_rxd,
    
    input  wire        drive_crc,
        
    output reg [31:0]  o_calculated_crc 
    );
    
    localparam CRC_POLY = 32'hEDB8_8320;
    reg [31:0] stored_crc;
    
    // intermediate signals in between each cycle
    wire feedback0;
    wire feedback1;
    wire [31:0] crc_shifted0;
    wire [31:0] crc_shifted1;
    wire [31:0] crc_mid;
    wire [31:0] crc_next;
    
    // --------------------------------------------------
    //     crc-32 algorithm (2 phase calculation)
    // --------------------------------------------------
    assign feedback0        = stored_crc[0] ^ i_rxd[0];
    assign feedback1        = crc_mid[0] ^ i_rxd[1];
    
    assign crc_shifted0     = stored_crc >> 1;
    assign crc_shifted1     = crc_mid >> 1;
    
    assign crc_mid          = (feedback0) ? crc_shifted0 ^ CRC_POLY : crc_shifted0;
    assign crc_next         = (feedback1) ? crc_shifted1 ^ CRC_POLY : crc_shifted1;
    
    // --------------------------------------------------
    // CRC-32 Engine (2 shifts per cycle per RMII Standard)
    // --------------------------------------------------
    always @ (posedge i_rmii_clk) begin
        if (!i_rstn || i_crc_init)begin
            o_calculated_crc    <= 32'hFFFF_FFFF;
            stored_crc          <= 32'hFFFF_FFFF;
        end
        else begin
            if(drive_crc) begin
                o_calculated_crc    <= crc_next;
                stored_crc          <= crc_next;
            end
        end
    end
    
endmodule
