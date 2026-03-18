#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

typedef hls::axis<ap_uint<32>, 0, 0, 0> axis_32;

// -------------------------------------------------------
// Encode a single bit using K=7, rate-1/2 encoder
// G1 = 1111001 (oct 171): taps at positions {1,2,3,4,7}
//   -> bit ^ state[0] ^ state[1] ^ state[2] ^ state[5]
// G2 = 1011011 (oct 133): taps at positions {1,3,4,6,7}
//   -> bit ^ state[1] ^ state[2] ^ state[4] ^ state[5]
// state[0] = most recent previous input (shiftReg position 2 in MATLAB)
// state[5] = oldest (shiftReg position 7 in MATLAB)
// -------------------------------------------------------
ap_uint<2> encode(ap_uint<1> bit, ap_uint<6> &state)
{
#pragma HLS INLINE
    ap_uint<1> g1 = bit ^ state[0] ^ state[1] ^ state[2] ^ state[5];
    ap_uint<1> g2 = bit ^ state[1] ^ state[2] ^ state[4] ^ state[5];

    // Shift state: new state[0]=bit, state[1]=old state[0], ..., state[5]=old state[4]
    // Equivalent to: shiftReg = [bit; shiftReg(1:end-1)] in MATLAB
    state = ((state << 1) & ap_uint<6>(0x3F)) | ap_uint<6>(bit);

    return (g2, g1);  // pack as {g2, g1} -> out_bits[1]=g2, out_bits[0]=g1
}

// -------------------------------------------------------
// Top-level AXI Stream convolutional encoder
// Input:  one 32-bit word (32 info bits)
// Output: two 32-bit words (64 encoded bits, rate 1/2)
// -------------------------------------------------------
void convEncoder_Seq(
    hls::stream<axis_32> &s_axis,
    hls::stream<axis_32> &m_axis)
{
#pragma HLS INTERFACE axis port=s_axis
#pragma HLS INTERFACE axis port=m_axis
#pragma HLS PIPELINE II=1

    static ap_uint<6> state = 0;  // 6-bit shift register state (K=7)

    if (!s_axis.empty())
    {
        axis_32 in_word = s_axis.read();
        ap_uint<32> data = in_word.data;

        ap_uint<64> encoded = 0;

        for (int i = 0; i < 32; i++)
        {
#pragma HLS UNROLL
            ap_uint<1> bit = data[i];

            // Compute two output bits (g1 at even index, g2 at odd index)
            // Matches MATLAB interleaving: [g1_bit0, g2_bit0, g1_bit1, g2_bit1, ...]
            ap_uint<2> out_bits = encode(bit, state);

            encoded.range(2*i+1, 2*i) = out_bits;
        }

        // Send lower 32 bits as first AXI word
        axis_32 out_word0;
        out_word0.data = encoded.range(31, 0);
        out_word0.keep = 0xF;
        out_word0.strb = 0xF;
        out_word0.last = 0;  // not last — second word follows
        m_axis.write(out_word0);

        // Send upper 32 bits as second AXI word
        axis_32 out_word1;
        out_word1.data = encoded.range(63, 32);
        out_word1.keep = 0xF;
        out_word1.strb = 0xF;
        out_word1.last = in_word.last;  // propagate TLAST from input
        m_axis.write(out_word1);
    }
}