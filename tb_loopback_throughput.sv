`timescale 1ns/1ps

module axis_fifo #(
    parameter int DATA_W = 32,
    parameter int DEPTH  = 16,
    localparam int ADDR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH)
) (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [DATA_W-1:0] s_data,
    input  logic              s_valid,
    output logic              s_ready,
    input  logic              s_last,
    output logic [DATA_W-1:0] m_data,
    output logic              m_valid,
    input  logic              m_ready,
    output logic              m_last,
    output logic [ADDR_W:0]   level
);
    logic [DATA_W-1:0] mem_data [0:DEPTH-1];
    logic              mem_last [0:DEPTH-1];
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] rd_ptr;
    logic [ADDR_W:0]   used;

    wire push = s_valid && s_ready;
    wire pop  = m_valid && m_ready;

    assign s_ready = (used < DEPTH);
    assign m_valid = (used != 0);
    assign m_data  = mem_data[rd_ptr];
    assign m_last  = mem_last[rd_ptr];
    assign level   = used;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            used   <= '0;
        end else begin
            if (push) begin
                mem_data[wr_ptr] <= s_data;
                mem_last[wr_ptr] <= s_last;
                wr_ptr           <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
            end
            if (pop) begin
                rd_ptr <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
            end
            case ({push, pop})
                2'b10: used <= used + 1'b1;
                2'b01: used <= used - 1'b1;
                default: used <= used;
            endcase
        end
    end
endmodule

module tb_loopback_throughput;
    localparam int CLK_PERIOD_NS        = 10;
    localparam int NUM_TRANSACTIONS     = 1000;
    localparam int CLOCK_FREQ_MHZ       = 100;
    localparam bit USE_FIFO             = 1'b1;
    localparam int FIFO_DEPTH           = 16;
    localparam int INPUT_BITS_PER_BEAT  = 32;
    localparam int DECODE_BITS_PER_BEAT = 16;

    logic clk;
    logic rst_n;

    // Encoder slave AXIS stimulus side.
    logic [31:0] enc_s_tdata;
    logic        enc_s_tvalid;
    logic        enc_s_tready;
    logic [3:0]  enc_s_tkeep;
    logic [3:0]  enc_s_tstrb;
    logic [0:0]  enc_s_tlast;

    // Encoder master AXIS / decoder slave AXIS interconnect.
    logic [31:0] enc_m_tdata;
    logic        enc_m_tvalid;
    logic        enc_m_tready;
    logic [3:0]  enc_m_tkeep;
    logic [3:0]  enc_m_tstrb;
    logic [0:0]  enc_m_tlast;

    logic [31:0] link_tdata;
    logic        link_tvalid;
    logic        link_tready;
    logic        link_tlast;

    // Decoder master AXIS output.
    logic [31:0] dec_m_tdata;
    logic        dec_m_tvalid;
    logic        dec_m_tready;

    // Control/status.
    logic enc_ap_start;
    logic enc_ap_done;
    logic enc_ap_idle;
    logic enc_ap_ready;


    longint unsigned cycle_count;
    longint unsigned first_input_cycle;
    longint unsigned first_output_cycle;
    longint unsigned last_output_cycle;
    longint unsigned input_words_sent;
    longint unsigned encoder_words_sent;
    longint unsigned output_words_recv;
    longint unsigned total_input_bits;
    longint unsigned total_encoded_bits;
    longint unsigned total_output_bits;
    longint unsigned active_in_cycles;
    longint unsigned active_mid_cycles;
    longint unsigned active_out_cycles;
    longint unsigned start_to_start_cycle_sum;
    longint unsigned output_gap_cycle_sum;
    longint unsigned input_handshake_prev_cycle;
    longint unsigned output_handshake_prev_cycle;
    int            input_gap_samples;
    int            output_gap_samples;
    bit            seen_first_input;
    bit            seen_first_output;
    bit            test_done;

    real latency_cycles;
    real throughput_bpc;
    real throughput_mbps;
    real steady_state_bpc;
    real input_utilization;
    real mid_utilization;
    real output_utilization;
    real avg_input_ii;
    real avg_output_ii;

    // ------------------------------
    // Clock generation
    // ------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    // ------------------------------
    // Reset generation
    // ------------------------------
    initial begin
        rst_n        = 1'b0;
        enc_ap_start = 1'b0;
        enc_s_tdata  = '0;
        enc_s_tvalid = 1'b0;
        enc_s_tkeep  = 4'hF;
        enc_s_tstrb  = 4'hF;
        enc_s_tlast  = '0;
        dec_m_tready = 1'b1;
        repeat (10) @(posedge clk);
        rst_n        = 1'b1;
        enc_ap_start = 1'b1;
    end

    // ------------------------------
    // DUT instantiation: encoder
    // ------------------------------
    convEncoder_Seq u_encoder (
        .ap_clk       (clk),
        .ap_rst_n     (rst_n),
        .ap_start     (enc_ap_start),
        .ap_done      (enc_ap_done),
        .ap_idle      (enc_ap_idle),
        .ap_ready     (enc_ap_ready),
        .s_axis_TDATA (enc_s_tdata),
        .s_axis_TVALID(enc_s_tvalid),
        .s_axis_TREADY(enc_s_tready),
        .s_axis_TKEEP (enc_s_tkeep),
        .s_axis_TSTRB (enc_s_tstrb),
        .s_axis_TLAST (enc_s_tlast),
        .m_axis_TDATA (enc_m_tdata),
        .m_axis_TVALID(enc_m_tvalid),
        .m_axis_TREADY(enc_m_tready),
        .m_axis_TKEEP (enc_m_tkeep),
        .m_axis_TSTRB (enc_m_tstrb),
        .m_axis_TLAST (enc_m_tlast)
    );

    generate
        if (USE_FIFO) begin : gen_fifo
            logic [$clog2(FIFO_DEPTH):0] fifo_level;
            axis_fifo #(
                .DATA_W(32),
                .DEPTH (FIFO_DEPTH)
            ) u_fifo (
                .clk    (clk),
                .rst_n  (rst_n),
                .s_data (enc_m_tdata),
                .s_valid(enc_m_tvalid),
                .s_ready(enc_m_tready),
                .s_last (enc_m_tlast[0]),
                .m_data (link_tdata),
                .m_valid(link_tvalid),
                .m_ready(link_tready),
                .m_last (link_tlast),
                .level  (fifo_level)
            );
        end else begin : gen_direct
            assign link_tdata   = enc_m_tdata;
            assign link_tvalid  = enc_m_tvalid;
            assign link_tlast   = enc_m_tlast[0];
            assign enc_m_tready = link_tready;
        end
    endgenerate

    // ------------------------------
    // DUT instantiation: decoder
    // ------------------------------
    viterbi_decoder u_decoder (
        .ap_clk            (clk),
        .ap_rst_n          (rst_n),
        .encoded_in_TDATA  (link_tdata),
        .encoded_in_TVALID (link_tvalid),
        .encoded_in_TREADY (link_tready),
        .decoded_out_TDATA (dec_m_tdata),
        .decoded_out_TVALID(dec_m_tvalid),
        .decoded_out_TREADY(dec_m_tready)
    );

    // ------------------------------
    // AXI driver: continuous random input words
    // ------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_s_tvalid <= 1'b0;
            enc_s_tdata  <= '0;
            enc_s_tlast  <= '0;
        end else begin
            if (!test_done && ((enc_s_tvalid == 1'b0) || (enc_s_tvalid && enc_s_tready))) begin
                if (input_words_sent < NUM_TRANSACTIONS) begin
                    enc_s_tvalid <= 1'b1;
                    enc_s_tdata  <= $urandom;
                    enc_s_tlast  <= (input_words_sent == NUM_TRANSACTIONS-1);
                end else begin
                    enc_s_tvalid <= 1'b0;
                    enc_s_tlast  <= 1'b0;
                end
            end
        end
    end

    // ------------------------------
    // AXI monitor + throughput counters
    // ------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count               <= 0;
            first_input_cycle         <= 0;
            first_output_cycle        <= 0;
            last_output_cycle         <= 0;
            input_words_sent          <= 0;
            encoder_words_sent        <= 0;
            output_words_recv         <= 0;
            total_input_bits          <= 0;
            total_encoded_bits        <= 0;
            total_output_bits         <= 0;
            active_in_cycles          <= 0;
            active_mid_cycles         <= 0;
            active_out_cycles         <= 0;
            start_to_start_cycle_sum  <= 0;
            output_gap_cycle_sum      <= 0;
            input_handshake_prev_cycle<= 0;
            output_handshake_prev_cycle<=0;
            input_gap_samples         <= 0;
            output_gap_samples        <= 0;
            seen_first_input          <= 1'b0;
            seen_first_output         <= 1'b0;
            test_done                 <= 1'b0;
        end else begin
            cycle_count <= cycle_count + 1;

            if (enc_s_tvalid && enc_s_tready)
                active_in_cycles <= active_in_cycles + 1;
            if (link_tvalid && link_tready)
                active_mid_cycles <= active_mid_cycles + 1;
            if (dec_m_tvalid && dec_m_tready)
                active_out_cycles <= active_out_cycles + 1;

            if (enc_s_tvalid && enc_s_tready) begin
                input_words_sent <= input_words_sent + 1;
                total_input_bits <= total_input_bits + INPUT_BITS_PER_BEAT;
                if (!seen_first_input) begin
                    seen_first_input  <= 1'b1;
                    first_input_cycle <= cycle_count;
                end else begin
                    start_to_start_cycle_sum <= start_to_start_cycle_sum + (cycle_count - input_handshake_prev_cycle);
                    input_gap_samples        <= input_gap_samples + 1;
                end
                input_handshake_prev_cycle <= cycle_count;
            end

            if (link_tvalid && link_tready) begin
                encoder_words_sent <= encoder_words_sent + 1;
                total_encoded_bits <= total_encoded_bits + 32;
            end

            if (dec_m_tvalid && dec_m_tready) begin
                output_words_recv <= output_words_recv + 1;
                total_output_bits <= total_output_bits + DECODE_BITS_PER_BEAT;
                last_output_cycle <= cycle_count;
                if (!seen_first_output) begin
                    seen_first_output  <= 1'b1;
                    first_output_cycle <= cycle_count;
                end else begin
                    output_gap_cycle_sum       <= output_gap_cycle_sum + (cycle_count - output_handshake_prev_cycle);
                    output_gap_samples         <= output_gap_samples + 1;
                end
                output_handshake_prev_cycle <= cycle_count;

                if (output_words_recv + 1 == NUM_TRANSACTIONS) begin
                    test_done <= 1'b1;
                end
            end
        end
    end

    // ------------------------------
    // Throughput report and finish
    // ------------------------------
    initial begin
        $dumpfile("tb_loopback_throughput.vcd");
        $dumpvars(0, tb_loopback_throughput);

        wait(rst_n === 1'b1);
        wait(test_done === 1'b1);
        repeat (10) @(posedge clk);

        latency_cycles = first_output_cycle - first_input_cycle;
        throughput_bpc = (cycle_count != 0) ? (1.0 * total_output_bits) / cycle_count : 0.0;
        throughput_mbps = throughput_bpc * CLOCK_FREQ_MHZ;
        steady_state_bpc = (last_output_cycle > first_output_cycle) ?
                           (1.0 * ((output_words_recv - 1) * DECODE_BITS_PER_BEAT)) /
                           (last_output_cycle - first_output_cycle) : 0.0;
        input_utilization = (cycle_count != 0) ? (1.0 * active_in_cycles) / cycle_count : 0.0;
        mid_utilization   = (cycle_count != 0) ? (1.0 * active_mid_cycles) / cycle_count : 0.0;
        output_utilization= (cycle_count != 0) ? (1.0 * active_out_cycles) / cycle_count : 0.0;
        avg_input_ii      = (input_gap_samples != 0) ? (1.0 * start_to_start_cycle_sum) / input_gap_samples : 0.0;
        avg_output_ii     = (output_gap_samples != 0) ? (1.0 * output_gap_cycle_sum) / output_gap_samples : 0.0;

        $display("============================================================");
        $display(" Loopback throughput report");
        $display("============================================================");
        $display("Clock period                  : %0d ns", CLK_PERIOD_NS);
        $display("Clock frequency               : %0d MHz", CLOCK_FREQ_MHZ);
        $display("Transactions requested        : %0d", NUM_TRANSACTIONS);
        $display("FIFO enabled                  : %0d", USE_FIFO);
        $display("Total cycles                  : %0d", cycle_count);
        $display("First input cycle             : %0d", first_input_cycle);
        $display("First output cycle            : %0d", first_output_cycle);
        $display("Last output cycle             : %0d", last_output_cycle);
        $display("Latency (cycles)              : %0f", latency_cycles);
        $display("Input words accepted          : %0d", input_words_sent);
        $display("Encoder output words accepted : %0d", encoder_words_sent);
        $display("Decoder output words accepted : %0d", output_words_recv);
        $display("Total input bits              : %0d", total_input_bits);
        $display("Total encoded bits            : %0d", total_encoded_bits);
        $display("Total decoded bits            : %0d", total_output_bits);
        $display("Throughput (bits/cycle)       : %0.6f", throughput_bpc);
        $display("Throughput (Mbps)             : %0.6f", throughput_mbps);
        $display("Steady-state throughput (b/c) : %0.6f", steady_state_bpc);
        $display("Input utilization             : %0.6f", input_utilization);
        $display("Encoder->Decoder utilization  : %0.6f", mid_utilization);
        $display("Output utilization            : %0.6f", output_utilization);
        $display("Average input II (cycles)     : %0.6f", avg_input_ii);
        $display("Average output II (cycles)    : %0.6f", avg_output_ii);
        $display("============================================================");
        $finish;
    end

    // ------------------------------
    // Safety timeout
    // ------------------------------
    initial begin
        #(CLK_PERIOD_NS * 200000);
        $error("Timeout waiting for loopback test to complete.");
        $finish;
    end

endmodule
