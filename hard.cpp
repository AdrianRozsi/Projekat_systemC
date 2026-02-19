#define _CRT_SECURE_NO_DEPRECATE 
#include "hard.hpp"
#include <systemc.h>
#include <vector>
#include <complex>
#include <cmath>
#include <iostream>
#include <cstring>
#include "utils.hpp"
#include <fstream>

using Complex = std::complex<double>;

int offset_index = 0;

Hard::Hard(sc_core::sc_module_name name, Bram &bram_ref)
    : sc_core::sc_module(name),
      bram(bram_ref),
      bram_samples_socket("bram_samples_socket"),
      bram_coeffs_socket("bram_coeffs_socket"),
      bram_result_socket("bram_result_socket"),
      start(false),
      done(false),
      offset(sc_core::SC_ZERO_TIME)
{
    soft_socket.register_b_transport(this, &Hard::b_transport);
    SC_REPORT_INFO("Hard", "Constructed.");
}

Hard::~Hard()
{
    SC_REPORT_INFO("Hard", "Destroyed.");
}

// FFT algoritmus
std::vector<Complex> Hard::fft(const std::vector<std::complex<double>>& in)
{
     vector<Complex> x(in);

	// DFT
	unsigned int N = x.size(), k = N, n;
	double thetaT = 3.14159265358979323846264338328L / N;
	Complex phiT = Complex(cos(thetaT), -sin(thetaT)), T;
	while (k > 1)
	{
		n = k;
		k >>= 1;
		phiT = phiT * phiT;
		T = 1.0L;
		for (unsigned int l = 0; l < k; l++)
		{
			for (unsigned int a = l; a < N; a += n)
			{
				unsigned int b = a + k;
				Complex t = x[a] - x[b];
				x[a] += x[b];
				x[b] = t * T;
			}
			T *= phiT;
		}
	}
	// Decimate
	unsigned int m = (unsigned int)log2(N);
	for (unsigned int a = 0; a < N; a++)
	{
		unsigned int b = a;
		// Reverse bits
		b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
		b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
		b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
		b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
		b = ((b >> 16) | (b << 16)) >> (32 - m);
		if (b > a)
		{
			Complex t = x[a];
			x[a] = x[b];
			x[b] = t;
		}
	}

	return x;
}

// IFFT algoritmus
std::vector<Complex> Hard::ifft(const std::vector<std::complex<double>>& in)
{
    vector<Complex> x1(in);
    size_t size = in.size();
    
    // 1. Konjugálás
    for (int i = 0; i < size; i++) {
        x1[i] = std::conj(x1[i]);
    }

    // 2. FFT
    x1 = fft(x1);

    // 3. Konjugálás újra
    for (int i = 0; i < size; i++) {
        x1[i] = std::conj(x1[i]);
    }

    //Normalizálás
    for (int i = 0; i < size; i++) {
        x1[i] /= size; 
    }
    
    return x1;
}

// BRAM adott régiójának törlése
void Hard::clear_bram_region(unsigned char type)
{
    for (int i = 0; i < COEFFS_SIZE * 2; ++i)
    {
        write_bram(i, 0.0, type);
    }
}

// FIR funkció hardver oldalon, itt történik az IFFT számítás
void Hard::firFunction()
{
    
    std::vector<Complex> fft_data(COEFFS_SIZE);
    for (int i = 0; i < COEFFS_SIZE; ++i) {
        const num_t re = read_bram( i * 2, 1);
        const num_t im = read_bram( i * 2 + 1, 1);
        fft_data[i] = Complex(re, im);
    }

    // IFFT számítás
    std::vector<std::complex<double>> ifft_result = ifft(fft_data);

    // --- FÁJLBA ÍRÁS MÓDOSÍTÁSA ---
    // 1. "app" mód: hozzáfűzés a fájl végéhez, nem felülírás
  
    std::ofstream out_file("ifft_result.txt", std::ios::out | std::ios::app); 
    
    if (!out_file) {
        std::cerr << "Hiba a fájl megnyitásakor!" << std::endl;
        return;
    }

   
   
    out_file << "=== BLOKK " << (block_counter++) << " IFFT EREDMÉNY ===" << std::endl;

    for (size_t i = 0; i < 100 && i < ifft_result.size(); ++i) {
        out_file << "Idx " << i << ": "
                 << "Real: " << ifft_result[i].real() << ", "
                 << "Imag: " << ifft_result[i].imag() << "\n";
    }
    out_file << "==========================================\n";
    out_file.close();
    // ------------------------------

    // Eredmény írása BRAM-ba
    for (size_t i = 0; i < ifft_result.size(); i++) {
        write_bram(i * 2, ifft_result[i].real(), 2);
    }

    done = true;
}

void Hard::b_transport(pl_t &pl, sc_core::sc_time &offset)
{
    auto cmd = pl.get_command();
    auto addr = pl.get_address();
    unsigned int len = pl.get_data_length();
    auto* buf = pl.get_data_ptr();
    pl.set_response_status(tlm::TLM_OK_RESPONSE);

    if (len != BUFF_SIZE)
    {
        pl.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
        return;
    }

    uint64_t local_addr = addr - VP_ADDR_HARD_BASE;

    if (cmd == tlm::TLM_WRITE_COMMAND)
    {
        if (local_addr == ADDR_CMD)
        {
            done = false;          //minden új trigger előtt töröljük a done jelet
            //SC_REPORT_INFO("Hard", "Host triggered firFunction().");
            firFunction();
        }
        else if (local_addr == ADDR_OFFSET)
        {
            
        }
        else
        {
            pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        }
    }
    else if (cmd == tlm::TLM_READ_COMMAND)
    {
        if (local_addr == ADDR_STATUS)
        {
            //std::cout << "[HARD] Status read, done = " << done << std::endl;
            num_t done_val = done ? 1.0 : 0.0;
            to_uchar(buf, done_val);
        }
        else
        {
            pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        }
    }
    else
    {
        pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    }

    offset += sc_core::sc_time(10, sc_core::SC_NS);
}

num_t Hard::read_bram(int addr, unsigned char type)
{
    pl_t pl;
    unsigned char buf[BUFF_SIZE];
    pl.set_data_length(BUFF_SIZE);
    pl.set_data_ptr(buf);
    pl.set_command(tlm::TLM_READ_COMMAND);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    sc_core::sc_time loc_offset = sc_core::SC_ZERO_TIME;
    uint64_t base = 0;
    switch (type) {
        case 0: base = VP_ADDR_SAMPLES_BASE; break;
        case 1: base = VP_ADDR_COEFFS_BASE; break;
        case 2: base = VP_ADDR_RESULT_BASE; break;
        default:
            SC_REPORT_ERROR("Hard::read_bram", "Invalid BRAM type");
            break;
    }
    pl.set_address(static_cast<uint64_t>(addr) * BUFF_SIZE + base);
    interconnect_socket->b_transport(pl, loc_offset);
    //wait(sc_core::sc_time(10.8, sc_core::SC_NS));
    num_t result = to_fixed(buf);
/*std::cout << "[HARD DEBUG] Read from addr=0x" << std::hex
          << base + addr * BUFF_SIZE << std::dec
          << " => val = " << result << std::endl;*/
return result;
}

void Hard::write_bram(int addr, num_t val, unsigned char type)
{
    pl_t pl;
    unsigned char buf[BUFF_SIZE];
    to_uchar(buf, val);
    pl.set_data_length(BUFF_SIZE);
    pl.set_data_ptr(buf);
    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    sc_core::sc_time loc_offset = sc_core::SC_ZERO_TIME;
    uint64_t base = 0;
    switch (type) {
        case 0: base = VP_ADDR_SAMPLES_BASE; break;
        case 1: base = VP_ADDR_COEFFS_BASE; break;
        case 2: base = VP_ADDR_RESULT_BASE; break;
        default:
            SC_REPORT_ERROR("Hard::write_bram", "Invalid BRAM type");
            break;
    }
    pl.set_address(static_cast<uint64_t>(addr) * BUFF_SIZE + base);
    interconnect_socket->b_transport(pl, loc_offset);
    //wait(sc_core::sc_time(10.8, sc_core::SC_NS));
}
