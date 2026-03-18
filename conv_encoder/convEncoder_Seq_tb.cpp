#include <iostream>
#include <iomanip>
#include <cstring>
#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"

typedef hls::axis<ap_uint<32>, 0, 0, 0> axis_32;

// -------------------------------------------------------
// Forward declaration of DUT
// -------------------------------------------------------
void convEncoder_Seq(hls::stream<axis_32> &s_axis, hls::stream<axis_32> &m_axis);

// -------------------------------------------------------
// Software reference model (mirrors MATLAB logic exactly)
// K=7, G1=171oct, G2=133oct
// -------------------------------------------------------
void convEncoder_ref(
    const uint32_t *input_words,
    uint32_t       *output_words,  // output_words[2*n] for n input words
    int             num_words,
    uint8_t        &shiftReg)      // 6-bit shift register (pass by ref for state persistence)
{
    // G1=1111001 -> taps {1,2,3,4,7} -> bit, sr[0], sr[1], sr[2], sr[5]
    // G2=1011011 -> taps {1,3,4,6,7} -> bit, sr[1], sr[2], sr[4], sr[5]
    // sr[0] = most recent, sr[5] = oldest

    for (int w = 0; w < num_words; w++)
    {
        uint64_t encoded = 0;
        for (int i = 0; i < 32; i++)
        {
            uint8_t bit = (input_words[w] >> i) & 1;

            uint8_t g1 = bit
                       ^ ((shiftReg >> 0) & 1)
                       ^ ((shiftReg >> 1) & 1)
                       ^ ((shiftReg >> 2) & 1)
                       ^ ((shiftReg >> 5) & 1);

            uint8_t g2 = bit
                       ^ ((shiftReg >> 1) & 1)
                       ^ ((shiftReg >> 2) & 1)
                       ^ ((shiftReg >> 4) & 1)
                       ^ ((shiftReg >> 5) & 1);

            // Shift register update: new bit enters at position 0
            shiftReg = ((shiftReg << 1) & 0x3F) | bit;

            encoded |= ((uint64_t)g1 << (2 * i));
            encoded |= ((uint64_t)g2 << (2 * i + 1));
        }
        output_words[2 * w]     = (uint32_t)(encoded & 0xFFFFFFFF);
        output_words[2 * w + 1] = (uint32_t)(encoded >> 32);
    }
}

// -------------------------------------------------------
// Helper: print 32-bit word as binary string
// -------------------------------------------------------
void printBinary(uint32_t val, int bits = 32)
{
    for (int i = bits - 1; i >= 0; i--)
        std::cout << ((val >> i) & 1);
}

// -------------------------------------------------------
// Run one test case
// Returns true if PASS, false if FAIL
// -------------------------------------------------------
bool runTest(
    const char     *test_name,
    uint32_t       *input_words,
    int             num_words,
    bool            verbose = false)
{
    hls::stream<axis_32> s_axis("s_axis");
    hls::stream<axis_32> m_axis("m_axis");

    // ---- Push input words into DUT stream ----
    for (int i = 0; i < num_words; i++)
    {
        axis_32 pkt;
        pkt.data = input_words[i];
        pkt.keep = 0xF;
        pkt.strb = 0xF;
        pkt.last = (i == num_words - 1) ? 1 : 0;
        s_axis.write(pkt);
    }

    // ---- Run DUT ----
    // Call enough times to drain input and produce output
    // Each call processes one word if available
    for (int call = 0; call < num_words * 3; call++)
        convEncoder_Seq(s_axis, m_axis);

    // ---- Collect DUT output ----
    int expected_out_words = num_words * 2;
    if ((int)m_axis.size() != expected_out_words)
    {
        std::cout << "[FAIL] " << test_name
                  << " — expected " << expected_out_words
                  << " output words, got " << m_axis.size() << "\n";
        return false;
    }

    uint32_t dut_out[64] = {0};
    for (int i = 0; i < expected_out_words; i++)
    {
        axis_32 pkt = m_axis.read();
        dut_out[i]  = (uint32_t)pkt.data;
    }

    // ---- Run software reference ----
    static uint8_t  ref_state = 0;
    static uint32_t ref_out[64] = {0};
    convEncoder_ref(input_words, ref_out, num_words, ref_state);

    // ---- Compare ----
    bool pass = true;
    for (int i = 0; i < expected_out_words; i++)
    {
        if (dut_out[i] != ref_out[i])
        {
            pass = false;
            std::cout << "[FAIL] " << test_name
                      << " — output word [" << i << "]"
                      << "  DUT=0x" << std::hex << std::setw(8) << std::setfill('0') << dut_out[i]
                      << "  REF=0x" << std::hex << std::setw(8) << std::setfill('0') << ref_out[i]
                      << std::dec << "\n";
        }
    }

    if (pass)
    {
        std::cout << "[PASS] " << test_name << "\n";
        if (verbose)
        {
            for (int w = 0; w < num_words; w++)
            {
                std::cout << "  Word[" << w << "]"
                          << "  IN =0x" << std::hex << std::setw(8) << std::setfill('0') << input_words[w]
                          << "  OUT[lo]=0x" << std::setw(8) << dut_out[2*w]
                          << "  OUT[hi]=0x" << std::setw(8) << dut_out[2*w+1]
                          << std::dec << "\n";
            }
        }
    }

    return pass;
}

// -------------------------------------------------------
// Main testbench
// -------------------------------------------------------
int main()
{
    int total = 0, passed = 0;
    bool ok;

    std::cout << "========================================\n";
    std::cout << "  Conv Encoder Testbench (K=7 [171 133])\n";
    std::cout << "========================================\n\n";

    // --------------------------------------------------
    // TC1: All zeros — encoder should output all zeros
    //      (state stays 0, XOR of 0s = 0)
    // --------------------------------------------------
    {
        uint32_t input[] = {0x00000000};
        ok = runTest("TC1: All-zero input", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC2: All ones
    // --------------------------------------------------
    {
        uint32_t input[] = {0xFFFFFFFF};
        ok = runTest("TC2: All-ones input", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC3: Single bit set at LSB (bit 0 = 1, rest = 0)
    //      Classic impulse response test
    // --------------------------------------------------
    {
        uint32_t input[] = {0x00000001};
        ok = runTest("TC3: Impulse (bit 0 only)", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC4: Single bit at MSB (bit 31)
    // --------------------------------------------------
    {
        uint32_t input[] = {0x80000000};
        ok = runTest("TC4: Single bit at MSB (bit 31)", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC5: Alternating bits 0xAAAAAAAA
    // --------------------------------------------------
    {
        uint32_t input[] = {0xAAAAAAAA};
        ok = runTest("TC5: Alternating bits 0xAAAAAAAA", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC6: Alternating bits 0x55555555
    // --------------------------------------------------
    {
        uint32_t input[] = {0x55555555};
        ok = runTest("TC6: Alternating bits 0x55555555", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC7: Known pattern 0xDEADBEEF
    // --------------------------------------------------
    {
        uint32_t input[] = {0xDEADBEEF};
        ok = runTest("TC7: Known pattern 0xDEADBEEF", input, 1, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC8: Multi-word — state must carry over between words
    //      Sends 4 consecutive 32-bit words and checks
    //      that the shift register state is correctly
    //      maintained across word boundaries
    // --------------------------------------------------
    {
        uint32_t input[] = {0x12345678, 0xABCDEF01, 0xDEADBEEF, 0x00FF00FF};
        ok = runTest("TC8: Multi-word state persistence (4 words)", input, 4, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // TC9: Walking ones — each word has exactly one bit set
    // --------------------------------------------------
    {
        std::cout << "\n--- TC9: Walking ones (32 single-word tests) ---\n";
        bool all_pass = true;
        for (int b = 0; b < 32; b++)
        {
            char name[64];
            snprintf(name, sizeof(name), "TC9.%d: Walking one at bit %d", b, b);
            uint32_t input[] = {(uint32_t)(1u << b)};
            ok = runTest(name, input, 1, false);
            total++; if (ok) passed++; else all_pass = false;
        }
        if (all_pass) std::cout << "  All walking-one subtests passed.\n";
    }

    // --------------------------------------------------
    // TC10: Random-ish patterns
    // --------------------------------------------------
    {
        std::cout << "\n--- TC10: Pseudo-random patterns ---\n";
        uint32_t patterns[] = {
            0xC0FFEE42, 0x13370000, 0xBAADF00D,
            0x8BADF00D, 0xFACEFEED, 0x0B00B135
        };
        uint32_t input[] = {
            patterns[0], patterns[1], patterns[2],
            patterns[3], patterns[4], patterns[5]
        };
        ok = runTest("TC10: 6-word pseudo-random", input, 6, true);
        total++; if (ok) passed++;
    }

    // --------------------------------------------------
    // Summary
    // --------------------------------------------------
    std::cout << "\n========================================\n";
    std::cout << "  Results: " << passed << " / " << total << " tests passed\n";
    std::cout << "========================================\n";

    return (passed == total) ? 0 : 1;
}