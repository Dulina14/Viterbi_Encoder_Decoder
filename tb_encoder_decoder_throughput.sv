`timescale 1ns/1ps

module tb_encoder_decoder_throughput;

  // ============================================================
  // Parameters
  // ============================================================
  localparam int INPUT_WORD_WIDTH  = 32;
  localparam int DEC_IN_WORD_WIDTH = 32;
  localparam int DEC_OUT_VALID_BITS = 16;
  localparam int INPUT_BITS_PER_TXN = INPUT_WORD_WIDTH;
  localparam int NUM_TRANSACTIONS   = 1000;
  localparam int FIFO_DEPTH         = 64;
  localparam int CLK_PERIOD_NS      = 10;
  localparam int TIMEOUT_CYCLES     = 2000000;
  localparam int MSB_INDEX          = INPUT_WORD_WIDTH - 1;
  localparam real CLOCK_FREQ_MHZ    = 1000.0 / CLK_PERIOD_NS;

  localparam longint TOTAL_INPUT_BITS_TARGET = NUM_TRANSACTIONS * INPUT_BITS_PER_TXN;

  // ============================================================
  // Clock / Reset
  // ============================================================
  logic ap_clk;
  logic ap_rst_n;

  initial begin
    ap_clk = 1'b0;
    forever #(CLK_PERIOD_NS/2) ap_clk = ~ap_clk;
  end

  initial begin
    ap_rst_n = 1'b0;
    repeat (20) @(posedge ap_clk);
    ap_rst_n = 1'b1;
  end

  // ============================================================
  // Encoder AXI-Stream Slave Interface (driven by AXI driver)
  // ============================================================
  logic [0:0] enc_s_tdata;
  logic       enc_s_tvalid;
  wire        enc_s_tready;
  logic [0:0] enc_s_tkeep;
  logic [0:0] enc_s_tstrb;
  logic [0:0] enc_s_tlast;

  // ============================================================
  // Encoder AXI-Stream Master Interface (to bridge/FIFO)
  // ============================================================
  wire  [0:0] enc_m_tdata;
  wire        enc_m_tvalid;
  logic       enc_m_tready;
  wire  [0:0] enc_m_tkeep;
  wire  [0:0] enc_m_tstrb;
  wire  [0:0] enc_m_tlast;

  // ============================================================
  // Decoder AXI-Stream-like Interface
  // ============================================================
  logic [31:0] dec_in_tdata;
  logic        dec_in_tvalid;
  wire         dec_in_tready;

  wire [31:0]  dec_out_tdata;
  wire         dec_out_tvalid;
  logic        dec_out_tready;

  // ============================================================
  // DUT Instantiation
  // ============================================================
  convEncoder u_conv_encoder (
    .ap_clk      (ap_clk),
    .ap_rst_n    (ap_rst_n),
    .s_axis_TDATA(enc_s_tdata),
    .s_axis_TVALID(enc_s_tvalid),
    .s_axis_TREADY(enc_s_tready),
    .s_axis_TKEEP(enc_s_tkeep),
    .s_axis_TSTRB(enc_s_tstrb),
    .s_axis_TLAST(enc_s_tlast),
    .m_axis_TDATA(enc_m_tdata),
    .m_axis_TVALID(enc_m_tvalid),
    .m_axis_TREADY(enc_m_tready),
    .m_axis_TKEEP(enc_m_tkeep),
    .m_axis_TSTRB(enc_m_tstrb),
    .m_axis_TLAST(enc_m_tlast)
  );

  viterbi_decoder u_viterbi_decoder (
    .ap_clk           (ap_clk),
    .ap_rst_n         (ap_rst_n),
    .encoded_in_TDATA (dec_in_tdata),
    .encoded_in_TVALID(dec_in_tvalid),
    .encoded_in_TREADY(dec_in_tready),
    .decoded_out_TDATA(dec_out_tdata),
    .decoded_out_TVALID(dec_out_tvalid),
    .decoded_out_TREADY(dec_out_tready)
  );

  // ============================================================
  // AXI driver: feed continuous random 32-bit words into encoder
  // (encoder accepts 1-bit stream; each input word is serialized)
  // ============================================================
  logic [31:0] current_input_word;
  int          current_input_bit_pos;
  int          input_word_count;
  logic        input_done;

  wire enc_in_fire = enc_s_tvalid && enc_s_tready;

  always_ff @(posedge ap_clk or negedge ap_rst_n) begin
    if (!ap_rst_n) begin
      current_input_word    <= $urandom;
      current_input_bit_pos <= 0;
      input_word_count      <= 0;
      input_done            <= 1'b0;
      enc_s_tvalid          <= 1'b0;
      enc_s_tdata           <= '0;
      enc_s_tkeep           <= 1'b1;
      enc_s_tstrb           <= 1'b1;
      enc_s_tlast           <= 1'b0;
    end else begin
      // default sideband signals
      enc_s_tkeep <= 1'b1;
      enc_s_tstrb <= 1'b1;

      if (!input_done) begin
        enc_s_tvalid <= 1'b1;
        // Serialize each 32-bit word MSB-first into the encoder's 1-bit input stream.
        enc_s_tdata  <= current_input_word[MSB_INDEX - current_input_bit_pos];
        enc_s_tlast  <= (current_input_bit_pos == (INPUT_BITS_PER_TXN - 1));

        if (enc_in_fire) begin
          if (current_input_bit_pos == (INPUT_BITS_PER_TXN - 1)) begin
            current_input_bit_pos <= 0;
            input_word_count <= input_word_count + 1;
            if ((input_word_count + 1) >= NUM_TRANSACTIONS) begin
              input_done   <= 1'b1;
              enc_s_tvalid <= 1'b0;
              enc_s_tlast  <= 1'b0;
            end else begin
              current_input_word <= $urandom;
            end
          end else begin
            current_input_bit_pos <= current_input_bit_pos + 1;
          end
        end
      end else begin
        enc_s_tvalid <= 1'b0;
        enc_s_tlast  <= 1'b0;
      end
    end
  end

  // ============================================================
  // Encoder->Decoder alignment bridge + FIFO
  // Packs 32 encoded bits into one decoder input word (16 symbols)
  // ============================================================
  logic [31:0] pack_shift_reg;
  int          pack_bit_count;

  logic [31:0] fifo_mem [0:FIFO_DEPTH-1];
  int          fifo_wr_ptr;
  int          fifo_rd_ptr;
  int          fifo_count;

  logic [31:0] completed_word;
  logic        completed_word_tlast;

  wire fifo_full  = (fifo_count == FIFO_DEPTH);
  wire fifo_empty = (fifo_count == 0);

  wire enc_out_fire = enc_m_tvalid && enc_m_tready;
  wire dec_in_fire  = dec_in_tvalid && dec_in_tready;
  wire push_word    = enc_out_fire && (pack_bit_count == (DEC_IN_WORD_WIDTH - 1));
  // Pack encoder output bits by shifting left and inserting newest bit at LSB.
  wire [31:0] push_word_data = {pack_shift_reg[DEC_IN_WORD_WIDTH-2:0], enc_m_tdata};
  wire        push_word_tlast = enc_m_tlast;

  // TREADY always high except when a completed packed word cannot enter FIFO
  always_comb begin
    if ((pack_bit_count == (DEC_IN_WORD_WIDTH - 1)) && fifo_full) begin
      enc_m_tready = 1'b0;
    end else begin
      enc_m_tready = 1'b1;
    end
  end

  // Decoder input stream sourced from FIFO (ideal sink readiness at decoder output)
  always_comb begin
    dec_in_tvalid = !fifo_empty;
    dec_in_tdata  = fifo_mem[fifo_rd_ptr];
  end

  // Keep decoder output always ready under ideal conditions
  always_comb begin
    dec_out_tready = 1'b1;
  end

  // Pack encoded bits and push/pop FIFO words
  always_ff @(posedge ap_clk or negedge ap_rst_n) begin
    if (!ap_rst_n) begin
      pack_shift_reg       <= '0;
      pack_bit_count       <= 0;
      completed_word       <= '0;
      completed_word_tlast <= 1'b0;
      fifo_wr_ptr          <= 0;
      fifo_rd_ptr          <= 0;
      fifo_count           <= 0;
    end else begin
      if (enc_out_fire) begin
        if (pack_bit_count == (DEC_IN_WORD_WIDTH - 1)) begin
          completed_word       <= push_word_data;
          completed_word_tlast <= push_word_tlast;
          pack_bit_count       <= 0;
        end else begin
          pack_bit_count <= pack_bit_count + 1;
        end
        pack_shift_reg <= {pack_shift_reg[DEC_IN_WORD_WIDTH-2:0], enc_m_tdata};
      end

      // FIFO push/pop with correct handling of simultaneous operations
      unique case ({push_word, dec_in_fire})
        2'b10: begin // push only
          fifo_mem[fifo_wr_ptr] <= push_word_data;
          fifo_wr_ptr <= (fifo_wr_ptr == FIFO_DEPTH - 1) ? 0 : (fifo_wr_ptr + 1);
          fifo_count  <= fifo_count + 1;
        end
        2'b01: begin // pop only
          fifo_rd_ptr <= (fifo_rd_ptr == FIFO_DEPTH - 1) ? 0 : (fifo_rd_ptr + 1);
          fifo_count  <= fifo_count - 1;
        end
        2'b11: begin // simultaneous push/pop
          fifo_mem[fifo_wr_ptr] <= push_word_data;
          fifo_wr_ptr <= (fifo_wr_ptr == FIFO_DEPTH - 1) ? 0 : (fifo_wr_ptr + 1);
          fifo_rd_ptr <= (fifo_rd_ptr == FIFO_DEPTH - 1) ? 0 : (fifo_rd_ptr + 1);
        end
        default: begin
        end
      endcase
    end
  end

  // ============================================================
  // Throughput / Latency monitor
  // ============================================================
  longint cycle_count;
  longint total_input_bits_sent;
  longint total_encoded_bits_seen;
  longint total_decoder_words_sent;
  longint total_output_bits_received;
  longint total_output_words_received;

  longint first_input_cycle;
  longint first_output_cycle;
  longint last_output_cycle;
  bit     first_input_seen;
  bit     first_output_seen;

  // Bonus metrics
  longint decoder_input_active_cycles;
  longint encoder_output_backpressure_cycles;

  wire dec_out_fire = dec_out_tvalid && dec_out_tready;

  always_ff @(posedge ap_clk or negedge ap_rst_n) begin
    if (!ap_rst_n) begin
      cycle_count                     <= 0;
      total_input_bits_sent           <= 0;
      total_encoded_bits_seen         <= 0;
      total_decoder_words_sent        <= 0;
      total_output_bits_received      <= 0;
      total_output_words_received     <= 0;
      first_input_cycle               <= 0;
      first_output_cycle              <= 0;
      last_output_cycle               <= 0;
      first_input_seen                <= 1'b0;
      first_output_seen               <= 1'b0;
      decoder_input_active_cycles     <= 0;
      encoder_output_backpressure_cycles <= 0;
    end else begin
      cycle_count <= cycle_count + 1;

      if (enc_in_fire) begin
        total_input_bits_sent <= total_input_bits_sent + 1;
        if (!first_input_seen) begin
          first_input_seen  <= 1'b1;
          first_input_cycle <= cycle_count;
        end
      end

      if (enc_out_fire) begin
        total_encoded_bits_seen <= total_encoded_bits_seen + 1;
      end

      if (dec_in_fire) begin
        total_decoder_words_sent <= total_decoder_words_sent + 1;
      end

      if (dec_out_fire) begin
        total_output_bits_received  <= total_output_bits_received + DEC_OUT_VALID_BITS;
        total_output_words_received <= total_output_words_received + 1;
        last_output_cycle           <= cycle_count;
        if (!first_output_seen) begin
          first_output_seen  <= 1'b1;
          first_output_cycle <= cycle_count;
        end
      end

      if (dec_in_fire) begin
        decoder_input_active_cycles <= decoder_input_active_cycles + 1;
      end

      if (enc_m_tvalid && !enc_m_tready) begin
        encoder_output_backpressure_cycles <= encoder_output_backpressure_cycles + 1;
      end
    end
  end

  // ============================================================
  // End-of-test control and report
  // ============================================================
  real throughput_bpc;
  real throughput_mbps;
  real steady_bpc;
  real steady_mbps;
  real latency_cycles;
  real decoder_input_utilization;
  real output_avg_ii;

  initial begin : test_control
    throughput_bpc = 0.0;
    throughput_mbps = 0.0;
    steady_bpc = 0.0;
    steady_mbps = 0.0;
    latency_cycles = 0.0;
    decoder_input_utilization = 0.0;
    output_avg_ii = 0.0;

    // wait until reset deasserts
    @(posedge ap_rst_n);

    // Run until all expected decoded bits are received
    wait (total_output_bits_received >= TOTAL_INPUT_BITS_TARGET);

    // allow final bookkeeping cycle
    @(posedge ap_clk);

    if (cycle_count > 0) begin
      throughput_bpc = real'(total_output_bits_received) / cycle_count;
      throughput_mbps = throughput_bpc * CLOCK_FREQ_MHZ;
    end

    if (first_output_seen && (last_output_cycle >= first_output_cycle)) begin
      longint steady_cycles;
      steady_cycles = (last_output_cycle - first_output_cycle + 1);
      if (steady_cycles > 0) begin
        steady_bpc = real'(total_output_bits_received) / steady_cycles;
        steady_mbps = steady_bpc * CLOCK_FREQ_MHZ;
      end
    end

    if (first_input_seen && first_output_seen && (first_output_cycle >= first_input_cycle)) begin
      latency_cycles = first_output_cycle - first_input_cycle;
    end

    if (cycle_count > 0) begin
      decoder_input_utilization = real'(decoder_input_active_cycles) / cycle_count;
    end

    if (total_output_words_received > 1) begin
      output_avg_ii = real'(last_output_cycle - first_output_cycle) /
                      real'(total_output_words_received - 1);
    end else begin
      output_avg_ii = 0.0;
    end

    $display("\n================ THROUGHPUT REPORT ================");
    $display("Clock period (ns):                %0d", CLK_PERIOD_NS);
    $display("Assumed clock frequency (MHz):    %0.3f", CLOCK_FREQ_MHZ);
    $display("Total cycles:                     %0d", cycle_count);
    $display("Input transactions (32-bit):      %0d", input_word_count);
    $display("Total input bits sent:            %0d", total_input_bits_sent);
    $display("Total encoder bits observed:      %0d", total_encoded_bits_seen);
    $display("Total decoder words sent:         %0d", total_decoder_words_sent);
    $display("Total output bits received:       %0d", total_output_bits_received);
    $display("Latency (first in -> first out):  %0.0f cycles", latency_cycles);
    $display("Overall throughput:               %0.6f bits/cycle", throughput_bpc);
    $display("Overall throughput:               %0.3f Mbps", throughput_mbps);
    $display("Steady-state throughput:          %0.6f bits/cycle", steady_bpc);
    $display("Steady-state throughput:          %0.3f Mbps", steady_mbps);
    $display("Decoder input utilization:        %0.4f", decoder_input_utilization);
    $display("Average decoder output II:        %0.3f cycles", output_avg_ii);
    $display("Encoder backpressure cycles:      %0d", encoder_output_backpressure_cycles);
    $display("===================================================\n");

    $finish;
  end

  // timeout guard
  initial begin : timeout_guard
    #(CLK_PERIOD_NS * TIMEOUT_CYCLES);
    $display("ERROR: Timeout before receiving expected decoded bits.");
    $finish;
  end

  // ============================================================
  // Waveform dump (valid/ready/data/counters)
  // ============================================================
  initial begin
    $dumpfile("tb_encoder_decoder_throughput.vcd");
    $dumpvars(0, tb_encoder_decoder_throughput);
  end

endmodule
