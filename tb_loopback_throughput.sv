`timescale 1ns/1ps

module axis_fifo #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 8,
    localparam int ADDR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH)
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [WIDTH-1:0] s_data,
    input  logic             s_valid,
    output logic             s_ready,
    output logic [WIDTH-1:0] m_data,
    output logic             m_valid,
    input  logic             m_ready
);
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] rd_ptr;
    logic [ADDR_W:0]   count;

    assign s_ready = (count < DEPTH);
    assign m_valid = (count > 0);
    assign m_data  = mem[rd_ptr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count  <= '0;
        end else begin
            case ({s_valid && s_ready, m_valid && m_ready})
                2'b10: begin
                    mem[wr_ptr] <= s_data;
                    wr_ptr <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
                    count  <= count + 1'b1;
                end
                2'b01: begin
                    rd_ptr <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
                    count  <= count - 1'b1;
                end
                2'b11: begin
                    mem[wr_ptr] <= s_data;
                    wr_ptr <= (wr_ptr == DEPTH-1) ? '0 : wr_ptr + 1'b1;
                    rd_ptr <= (rd_ptr == DEPTH-1) ? '0 : rd_ptr + 1'b1;
                end
                default: begin end
            endcase
        end
    end
endmodule

module tb_loopback_throughput;
    timeunit 1ns;
    timeprecision 1ps;

    localparam int CLK_PERIOD_NS        = 10;
    localparam int CLOCK_FREQ_MHZ       = 100;
    localparam int NUM_INPUT_WORDS      = 1000;
    localparam int INPUT_BITS_PER_WORD  = 32;
    localparam int ENCODED_BITS_PER_IN  = 64;
    localparam int DECODER_IN_BITS      = 32;
    localparam int DECODER_OUT_BITS     = 16;
    localparam bit USE_FIFO             = 1'b1;
    localparam int FIFO_DEPTH           = 8;
    localparam int MAX_SIM_CYCLES       = 500000;

    logic clk;
    logic rst_n;

    // Encoder slave AXIS (bit-serial into HLS encoder).
    logic       enc_s_tdata;
    logic       enc_s_tvalid;
    logic       enc_s_tready;
    logic       enc_s_tkeep;
    logic       enc_s_tstrb;
    logic       enc_s_tlast;

    // Encoder master AXIS (bit-serial encoded stream).
    logic       enc_m_tdata;
    logic       enc_m_tvalid;
    logic       enc_m_tready;
    logic       enc_m_tkeep;
    logic       enc_m_tstrb;
    logic       enc_m_tlast;

    // Packed 32-bit stream between encoder packer and decoder.
    logic [31:0] packed_tdata;
    logic        packed_tvalid;
    logic        packed_tready;
    logic        packed_tlast;

    logic [31:0] fifo_tdata;
    logic        fifo_tvalid;
    logic        fifo_tready;

    // Decoder AXIS.
    logic [31:0] dec_s_tdata;
    logic        dec_s_tvalid;
    logic        dec_s_tready;
    logic [31:0] dec_m_tdata;
    logic        dec_m_tvalid;
    logic        dec_m_tready;

    // Driver state.
    logic [31:0] stimulus_word;
    logic [31:0] next_stimulus_word;
    int          stimulus_bit_idx;
    int          words_accepted;
    int          bits_sent_to_encoder;
    bit          sent_first_input;

    // Encoder packer state.
    logic [31:0] pack_shift_reg;
    int          pack_bit_count;
    int          packed_words_sent;
    logic        packed_last_pending;

    // Monitors and counters.
    longint unsigned cycle_count;
    longint unsigned first_input_cycle;
    longint unsigned first_output_cycle;
    longint unsigned last_output_cycle;
    longint unsigned total_input_bits;
    longint unsigned total_encoded_bits;
    longint unsigned total_decoder_input_words;
    longint unsigned total_decoded_bits;
    longint unsigned decoder_output_words;
    longint unsigned decoder_busy_cycles;
    longint unsigned packer_stall_cycles;
    longint unsigned encoder_output_active_cycles;

    real latency_cycles;
    real throughput_bpc;
    real throughput_mbps;
    real steady_state_bpc;
    real pipeline_utilization;
    real output_ii_cycles;

    // -----------------------------
    // Clock/reset generation
    // -----------------------------
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    initial begin
        rst_n = 1'b0;
        enc_s_tdata  = 1'b0;
        enc_s_tvalid = 1'b0;
        enc_s_tkeep  = 1'b1;
        enc_s_tstrb  = 1'b1;
        enc_s_tlast  = 1'b0;
        stimulus_word = '0;
        stimulus_bit_idx = 0;
        words_accepted = 0;
        bits_sent_to_encoder = 0;
        sent_first_input = 1'b0;
        pack_shift_reg = '0;
        pack_bit_count = 0;
        packed_words_sent = 0;
        packed_last_pending = 1'b0;
        dec_m_tready = 1'b1;
        cycle_count = 0;
        first_input_cycle = 0;
        first_output_cycle = 0;
        last_output_cycle = 0;
        total_input_bits = 0;
        total_encoded_bits = 0;
        total_decoder_input_words = 0;
        total_decoded_bits = 0;
        decoder_output_words = 0;
        decoder_busy_cycles = 0;
        packer_stall_cycles = 0;
        encoder_output_active_cycles = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
    end

    // -----------------------------
    // AXI driver: generate random 32-bit words and serialize to encoder
    // -----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_s_tvalid <= 1'b0;
            enc_s_tdata  <= 1'b0;
            enc_s_tlast  <= 1'b0;
            stimulus_word <= '0;
            stimulus_bit_idx <= 0;
            words_accepted <= 0;
            bits_sent_to_encoder <= 0;
            sent_first_input <= 1'b0;
        end else begin
            if (enc_s_tvalid && enc_s_tready) begin
                if (!sent_first_input) begin
                    sent_first_input <= 1'b1;
                    first_input_cycle <= cycle_count;
                end

                total_input_bits <= total_input_bits + 1;
                bits_sent_to_encoder <= bits_sent_to_encoder + 1;

                if (stimulus_bit_idx == INPUT_BITS_PER_WORD-1) begin
                    words_accepted <= words_accepted + 1;
                    stimulus_bit_idx <= 0;
                    enc_s_tvalid <= 1'b0;
                    enc_s_tlast  <= (words_accepted + 1 == NUM_INPUT_WORDS);
                end else begin
                    stimulus_bit_idx <= stimulus_bit_idx + 1;
                    enc_s_tdata <= stimulus_word[stimulus_bit_idx + 1];
                    enc_s_tlast <= 1'b0;
                end
            end

            if (!enc_s_tvalid && (words_accepted < NUM_INPUT_WORDS)) begin
                next_stimulus_word = $urandom();
                stimulus_word <= next_stimulus_word;
                enc_s_tdata   <= next_stimulus_word[0];
                enc_s_tvalid  <= 1'b1;
                enc_s_tlast   <= 1'b0;
                stimulus_bit_idx <= 0;
            end
        end
    end

    // -----------------------------
    // AXI monitor/packer: combine 32 encoded bits into decoder input words
    // -----------------------------
    assign enc_m_tready = !packed_tvalid || packed_tready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pack_shift_reg      <= '0;
            pack_bit_count      <= 0;
            packed_tdata        <= '0;
            packed_tvalid       <= 1'b0;
            packed_tlast        <= 1'b0;
            packed_words_sent   <= 0;
            packed_last_pending <= 1'b0;
        end else begin
            if (enc_m_tvalid) begin
                encoder_output_active_cycles <= encoder_output_active_cycles + 1;
            end
            if (enc_m_tvalid && !enc_m_tready) begin
                packer_stall_cycles <= packer_stall_cycles + 1;
            end

            if (enc_m_tvalid && enc_m_tready) begin
                total_encoded_bits <= total_encoded_bits + 1;
                pack_shift_reg <= {enc_m_tdata, pack_shift_reg[31:1]};

                if (pack_bit_count == DECODER_IN_BITS-1) begin
                    packed_tdata        <= {enc_m_tdata, pack_shift_reg[31:1]};
                    packed_tvalid       <= 1'b1;
                    packed_words_sent   <= packed_words_sent + 1;
                    packed_tlast        <= packed_last_pending || enc_m_tlast;
                    packed_last_pending <= 1'b0;
                    pack_bit_count      <= 0;
                end else begin
                    pack_bit_count <= pack_bit_count + 1;
                    if (enc_m_tlast) begin
                        packed_last_pending <= 1'b1;
                    end
                end
            end

            if (packed_tvalid && packed_tready) begin
                packed_tvalid <= 1'b0;
                packed_tlast  <= 1'b0;
            end
        end
    end

    // Optional FIFO buffering between encoder and decoder.
    generate
        if (USE_FIFO) begin : g_fifo
            axis_fifo #(
                .WIDTH(32),
                .DEPTH(FIFO_DEPTH)
            ) u_axis_fifo (
                .clk    (clk),
                .rst_n  (rst_n),
                .s_data (packed_tdata),
                .s_valid(packed_tvalid),
                .s_ready(packed_tready),
                .m_data (fifo_tdata),
                .m_valid(fifo_tvalid),
                .m_ready(fifo_tready)
            );

            assign dec_s_tdata  = fifo_tdata;
            assign dec_s_tvalid = fifo_tvalid;
            assign fifo_tready  = dec_s_tready;
        end else begin : g_no_fifo
            assign dec_s_tdata  = packed_tdata;
            assign dec_s_tvalid = packed_tvalid;
            assign packed_tready = dec_s_tready;
            assign fifo_tdata  = '0;
            assign fifo_tvalid = 1'b0;
            assign fifo_tready = 1'b0;
        end
    endgenerate

    // -----------------------------
    // AXI monitor: decoder ingress/egress accounting
    // -----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_decoder_input_words <= 0;
            total_decoded_bits <= 0;
            decoder_output_words <= 0;
            first_output_cycle <= 0;
            last_output_cycle <= 0;
            decoder_busy_cycles <= 0;
        end else begin
            if (dec_s_tvalid) begin
                decoder_busy_cycles <= decoder_busy_cycles + 1;
            end

            if (dec_s_tvalid && dec_s_tready) begin
                total_decoder_input_words <= total_decoder_input_words + 1;
            end

            if (dec_m_tvalid && dec_m_tready) begin
                total_decoded_bits <= total_decoded_bits + DECODER_OUT_BITS;
                decoder_output_words <= decoder_output_words + 1;
                last_output_cycle <= cycle_count;

                if (decoder_output_words == 0) begin
                    first_output_cycle <= cycle_count;
                end
            end
        end
    end

    // -----------------------------
    // Throughput counter / timeout / reporting
    // -----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
        end
    end

    initial begin
        $dumpfile("tb_loopback_throughput.vcd");
        $dumpvars(0, tb_loopback_throughput);

        wait(rst_n === 1'b1);
        wait(decoder_output_words == NUM_INPUT_WORDS*2);
        repeat (10) @(posedge clk);

        latency_cycles = (first_output_cycle >= first_input_cycle) ?
                         real'(first_output_cycle - first_input_cycle) : 0.0;
        throughput_bpc = (cycle_count != 0) ? (1.0 * total_decoded_bits) / cycle_count : 0.0;
        throughput_mbps = throughput_bpc * CLOCK_FREQ_MHZ;
        steady_state_bpc = (last_output_cycle > first_output_cycle) ?
                           (1.0 * total_decoded_bits) / (last_output_cycle - first_output_cycle + 1) : 0.0;
        pipeline_utilization = (cycle_count != 0) ? (1.0 * decoder_busy_cycles) / cycle_count : 0.0;
        output_ii_cycles = (decoder_output_words > 1) ?
                           (1.0 * (last_output_cycle - first_output_cycle)) / (decoder_output_words - 1) : 0.0;

        $display("============================================================");
        $display("Loopback throughput measurement complete");
        $display("  Total cycles               : %0d", cycle_count);
        $display("  Input words sent           : %0d", words_accepted);
        $display("  Total input bits sent      : %0d", total_input_bits);
        $display("  Encoded bits observed      : %0d", total_encoded_bits);
        $display("  Decoder input words        : %0d", total_decoder_input_words);
        $display("  Decoder output words       : %0d", decoder_output_words);
        $display("  Total decoded bits         : %0d", total_decoded_bits);
        $display("  First input cycle          : %0d", first_input_cycle);
        $display("  First output cycle         : %0d", first_output_cycle);
        $display("  Last output cycle          : %0d", last_output_cycle);
        $display("  Latency                    : %0.2f cycles", latency_cycles);
        $display("  Throughput                 : %0.6f bits/cycle", throughput_bpc);
        $display("  Throughput                 : %0.3f Mbps @ %0d MHz", throughput_mbps, CLOCK_FREQ_MHZ);
        $display("  Steady-state throughput    : %0.6f bits/cycle", steady_state_bpc);
        $display("  Pipeline utilization       : %0.2f%%", pipeline_utilization * 100.0);
        $display("  Decoder output II          : %0.3f cycles/output", output_ii_cycles);
        $display("  Packer stall cycles        : %0d", packer_stall_cycles);
        $display("============================================================");
        $finish;
    end

    initial begin
        wait(rst_n === 1'b1);
        repeat (MAX_SIM_CYCLES) @(posedge clk);
        $fatal(1, "Simulation timeout after %0d cycles", MAX_SIM_CYCLES);
    end

    // -----------------------------
    // DUT instances
    // -----------------------------
    convEncoder u_conv_encoder (
        .ap_clk       (clk),
        .ap_rst_n     (rst_n),
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

    viterbi_decoder u_viterbi_decoder (
        .ap_clk            (clk),
        .ap_rst_n          (rst_n),
        .encoded_in_TDATA  (dec_s_tdata),
        .encoded_in_TVALID (dec_s_tvalid),
        .encoded_in_TREADY (dec_s_tready),
        .decoded_out_TDATA (dec_m_tdata),
        .decoded_out_TVALID(dec_m_tvalid),
        .decoded_out_TREADY(dec_m_tready)
    );
endmodule
