`timescale 1ns / 1ps
/*
Stage 1: RMII Deserializer
-Bit/byte alignment
-Preamble stripping
-SFD detection
-Byte assembly
-CRC accumulator
-Output AXI-Stream of full Ethernet frame without FCS
*/
module packet_recvr
    #(
    parameter [31:0]  FPGA_IP = 32'hC0A80164, 
    parameter [31:0]  HOST_IP = 32'hC0A80165,
    parameter [15:0]  FPGA_PORT = 16'h4567,
    parameter [15:0]  HOST_PORT = 16'h4567,
    parameter [47:0]  FPGA_MAC = 48'he86a64e7e830,
    parameter [47:0]  HOST_MAC = 48'he86a64e7e829,
    parameter CHECK_DESTINATION = 1    
    )
    (
    // RMII side
    input  wire        i_rmii_clk,    // 50 MHz
    input  wire        i_rstn,
    input  wire [1:0]  i_rxd,
    input  wire        i_crs_dv,      // carrier sense/data valid

    // AXI-Stream output
    output reg [7:0]  o_axis_tdata,  
    output reg        o_axis_tvalid,
    input  wire       i_axis_tready,
    output reg        o_axis_tlast,
    output reg [3:0]  o_axis_tuser  
    );
    
   // ====================================
   // double-register received mii data
   //   Generates start/end condition
   // ====================================
   wire packet_start;
   wire packet_done;
   localparam integer MII_WIDTH = 2;
   localparam integer STAGES = 2;
   
   
    reg [MII_WIDTH*STAGES-1:0] rxd_z; // recieve data (4 bits)
    reg [STAGES-1:0]		   rxdv_z; // receive data valid (2 bits)
    reg [7:0]	    first_packet_count;

    localparam	    FIRST_PACKET_IGNORE = 0;
   
    always @(posedge i_rmii_clk)begin                                                                     
        if (!i_rstn)begin                 
             rxd_z <= 0;
             rxdv_z <= 0;
             first_packet_count <= 0;
         end                                                                   
         else begin
            // raw data shifting
             rxd_z[1:0] <= i_rxd; 
             rxd_z[3:2] <= rxd_z[1:0]; 
             // data valid shifting
             rxdv_z[0] <= i_crs_dv; 
             rxdv_z[1] <= rxdv_z[0];
             if (packet_done & first_packet_count < FIRST_PACKET_IGNORE) begin
                 first_packet_count <= first_packet_count + 1;
             end
        end
   end
   
   
   // packet start/end signals
   assign packet_start = (rxdv_z[1] == 0 && rxdv_z[0] == 1); // rising edge of i_crs_dv
   assign packet_done  = (rxdv_z[1] == 1 && rxdv_z[0] == 0); // falling edge of i_crs_dv
    
   // =======================================================    
   //                   Implementation    
   // =======================================================    
   
   // state registers
   reg [2:0] STATE, NEXT_STATE;
   
   // state encoding
   localparam IDLE          = 4'd0,
              PREAMBLE      = 4'd1,
              SFD           = 4'd2,
              DATA          = 4'd3,
              COMPARE       = 4'd4,
              DONE          = 4'd5,
              HEADER_ERROR  = 4'd6,
              CRC_ERROR     = 4'd7;
   
   localparam PREAMBLE_LENGTH = 28,
              SFD_LENGTH      = 4;
  
   // counters / byte assembly
    reg [7:0] stored_byte, next_stored_byte;
    reg [5:0] dibit_cnt;           // 0 ... PREAMBLE_LENGTH-1
    reg [15:0] byte_count_total;   // count all DATA bytes (including FCS)

    // plain "byte ready" pulse (independent of FCS discarding)
    wire byte_ready_any         = i_crs_dv && ((dibit_cnt % 4) == 3);
    wire byte_ready_data        = (STATE == DATA) && byte_ready_any;

    // 4-byte tail buffer: tail[31:24]=oldest, tail[7:0]=newest
    reg [31:0] tail, next_tail;

    // AXI output byte/valid/last (skeleton, no backpressure)
    reg [7:0]  out_byte;
    reg        out_valid;
    reg        out_last;

    // CRC engine signals
    wire [31:0] calculated_crc;
    reg         drive_crc;
    reg         crc_init;
    localparam [31:0] GOOD_RESIDUE = 32'hC704_DD7B;
    
    reg frame_error, next_frame_error;
   
   // ================================================================
   //                   CRC-32 Engine Instatiation
   // ================================================================
   crc_32_engine crc_i (
       .i_rmii_clk          (i_rmii_clk),    
       .i_rstn              (i_rstn),         
       .i_crc_init          (crc_init),         
       .i_rxd               (i_rxd),          
               
       .drive_crc           (drive_crc),      
               
       .o_calculated_crc    (calculated_crc)
   );
   
   // ================================================================
    // Next-state / combinational logic
    // ================================================================
    always @* begin
        NEXT_STATE        = STATE;
        next_stored_byte  = stored_byte;
        next_tail         = tail;
        next_frame_error  = frame_error;

        out_byte          = 8'h00;
        out_valid         = 1'b0;
        out_last          = 1'b0;

        case (STATE)
            IDLE: begin
                if (packet_start) begin
                    NEXT_STATE = PREAMBLE;
                end
            end

            PREAMBLE: begin
                if (rxd_z[1:0] != 2'b10) begin
                    NEXT_STATE       = HEADER_ERROR;
                    next_frame_error = 1'b1;
                end else begin
                    next_stored_byte = {rxd_z[1:0], stored_byte[7:2]};
                    NEXT_STATE = (dibit_cnt == PREAMBLE_LENGTH-1) ? SFD : PREAMBLE;
                end
            end

            SFD: begin
                // when full byte assembled, check for 0xD5
                if (byte_ready_any && (next_stored_byte != 8'hD5)) begin
                    NEXT_STATE       = HEADER_ERROR;
                    next_frame_error = 1'b1;
                end else begin
                    next_stored_byte = {rxd_z[1:0], stored_byte[7:2]};
                    // after 1 byte SFD, go to DATA
                    NEXT_STATE = (dibit_cnt == SFD_LENGTH-1) ? DATA : SFD;
                end
            end

            DATA: begin
                // assemble incoming dibits into a byte
                next_stored_byte = {rxd_z[1:0], stored_byte[7:2]};

                // when a full byte is ready:
                if (byte_ready_data) begin
                    // new byte is the one we just assembled
                    // (using next_stored_byte as the "new_byte")
                    // shift into 4-byte tail buffer
                    next_tail = {next_stored_byte, tail[31:24]};

                    // once we've seen at least 4 bytes, emit the oldest
                    if (byte_count_total >= 4) begin
                        out_byte  = tail[31:24];  // oldest byte in tail
                        out_valid = 1'b1;
                    end
                end

                // End of frame → go to CRC compare
                if (packet_done) begin
                    NEXT_STATE = COMPARE;
                end
            end

            COMPARE: begin
                // CRC residue check AFTER feeding data+FCS
                if (calculated_crc == GOOD_RESIDUE) begin
                    NEXT_STATE = DONE;
                end else begin
                    NEXT_STATE       = CRC_ERROR;
                    next_frame_error = 1'b1;
                end
            end

            HEADER_ERROR: begin
            next_frame_error = 1'b1;
                // flush until end of frame
                if (packet_done) begin
                    NEXT_STATE = DONE;
                    next_frame_error = 1'b0;
                end
            end

            CRC_ERROR: begin
                // single cycle state; go straight to DONE
                NEXT_STATE = DONE;
                next_frame_error = 1'b0;
            end

            DONE: begin
                // one cycle to signal end-of-frame; then back to IDLE
                NEXT_STATE = IDLE;
            end

            default: NEXT_STATE = IDLE;
        endcase
    end

    // ================================================================
    // Sequential logic
    // ================================================================
    always @(posedge i_rmii_clk) begin
        if (!i_rstn) begin
            STATE           <= IDLE;
            stored_byte     <= 8'h00;
            dibit_cnt       <= 2'd0;
            byte_count_total<= 16'd0;
            tail            <= 32'h0;
            frame_error     <= 1'b0;

            o_axis_tdata    <= 8'h00;
            o_axis_tvalid   <= 1'b0;
            o_axis_tlast    <= 1'b0;
            o_axis_tuser    <= 4'b0000;

            drive_crc       <= 1'b0;
            crc_init        <= 1'b0;
        end else begin
            STATE          <= NEXT_STATE;
            stored_byte    <= next_stored_byte;
            tail           <= next_tail;
            frame_error    <= next_frame_error;

            // dibit counter: 0..3 in PREAMBLE/SFD/DATA
            if (STATE != NEXT_STATE) begin
                dibit_cnt <= 2'd0;
            end else if (i_crs_dv) begin
                dibit_cnt <= dibit_cnt + 1'b1;
            end

            // total byte count (includes FCS, used only for FCS discard)
            if (byte_ready_data) begin
                byte_count_total <= byte_count_total + 1'b1;
            end
            if (STATE == IDLE || packet_start) begin
                byte_count_total <= 16'd0;
            end

            // CRC control
            crc_init  <= packet_start;                 // seed at frame start
            drive_crc <= (STATE == DATA) && i_crs_dv;  // feed data+FCS

            // AXI outputs (skeleton: assume tready always 1)
            o_axis_tdata  <= out_valid ? out_byte : o_axis_tdata;
            o_axis_tvalid <= out_valid;       // pulse when a byte is emitted
            o_axis_tlast  <= (STATE == DONE); 

            // tuser: error flag at end of frame
            if (STATE == DONE) begin
                o_axis_tuser <= frame_error ? 4'b0001 : 4'b0000;
            end else begin
                o_axis_tuser <= 4'b0000;
            end
        end
    end

                     
endmodule


