#include "soft.hpp"
#include <fstream>
#include "AudioFile.h"
#include <complex>
#include <vector>
#include <cstring>
#include <cmath>
#include "typedefs.hpp"
#include <chrono>
#include "utils.hpp"

using namespace std;
using namespace std::chrono;

int thr;
char* g_coeffs;
using Complex = std::complex<double>;
using namespace sc_core;

/*
double calc_power(const std::vector<Complex>& arr) {
    double sum = 0;
    for (const auto& val : arr) {
       
        sum += std::abs(val.real()); 
    }
    return sum;
}
*/


void Soft::write_to_file(const std::string& filename, const std::vector<std::complex<double>>& data, bool append) {
    std::ofstream file;
    file.open(filename, append ? std::ios::app : std::ios::out); 
    if (file.is_open()) {
        for (const auto& elem : data) {
            file << elem.real() << " " << elem.imag() << "\n";
        }
        file.close();
    } else {
        std::cerr << "Nem sikerült megnyitni a fájlt: " << filename << std::endl;
    }
}

/*
 * Gauss generálás -  abs() a negatív frekvenciákhoz!
 */
std::vector<double> Soft::generate_gaussian(const std::vector<double>& freq, uint8_t preset)
{
    std::vector<double> gauss(freq.size(), 0.0);

    for (int i = 0; i < gauss_freq.size(); i++)
    {
        for (int j = 0; j < freq.size(); j++)
        {
            // FONTOS: std::abs() kell, mert a getFreq negatív értékeket is visszaad!
            double f_abs = std::abs(freq[j]); 
            gauss[j] += gauss_amp[preset][i] * exp(-(pow(f_abs - gauss_freq[i], 2) / (2 * pow(gauss_freq[i] / 3.0, 2))));
        }
    }
    return gauss;
}

std::vector<double> Soft::getFreq(int n, double sampleRate)
{
    std::vector<double> freq(n);
    double df = sampleRate / n;
    
    for (int i = 0; i < n; ++i) {
        if (i <= n / 2) {
            freq[i] = i * df;
        } else {
            freq[i] = (i - n) * df;
        }
    }
    return freq;
}

SC_HAS_PROCESS(Soft);

Soft::Soft(sc_core::sc_module_name name, char *coefs, char *samples, uint8_t preset_val)
  : sc_module(name), offset(sc_core::SC_ZERO_TIME), preset(preset_val), samplesPath(samples)
{
    SC_THREAD(gen);
    SC_REPORT_INFO("Soft", "Constructed");
}

Soft::~Soft()
{
    SC_REPORT_INFO("Soft", "Destroyed.");
}

// ---------------------------------------------------------
// A FŐ FELDOLGOZÓ FÜGGVÉNY 
// ---------------------------------------------------------
void Soft::gen() {
    // 1. WAV BETÖLTÉSE
    bool loadedOK = audioFile.load(samplesPath);
    if (!loadedOK) {
        std::cout << "[HIBA] Nem sikerult betolteni a wav fajlt!" << std::endl;
        sc_core::sc_stop();
        return;
    }

    oldSize = audioFile.getNumSamplesPerChannel();
    sampleRate = audioFile.getSampleRate();
    std::cout << "[DEBUG] WAV betoltve: " << oldSize << " minta, " << sampleRate << " Hz" << std::endl;

    audioFileComplex.clear();
    for (int i = 0; i < audioFile.getNumSamplesPerChannel(); i++) {
        audioFileComplex.push_back(Complex(audioFile.samples[0][i], 0.0));
    }

    std::vector<double> freq_axis = getFreq(SAMPLES_SIZE, sampleRate);
    const int block_size = SAMPLES_SIZE;
    const int hop_size = block_size / 2;
    
    // Hann ablak
    std::vector<double> hann_window(block_size);
    for (int i = 0; i < block_size; i++) {
        hann_window[i] = 0.5 * (1 - cos(2 * M_PI * i / (block_size - 1)));
    }

    // Overlap-Add buffer
    int total_blocks = (oldSize - hop_size) / hop_size; 
    if (total_blocks < 1) total_blocks = 1;
    int final_audio_size = (total_blocks - 1) * hop_size + block_size;
    std::vector<float> final_audio(final_audio_size, 0.0f);

    std::cout << "[INFO] Feldolgozas... Blokkok: " << total_blocks << std::endl;

    for (int b = 0; b < total_blocks; b++) {
        int start_idx = b * hop_size;
        
        // --- 1. BLOKK KIVÁGÁS + ABLAKOLÁS ---
        std::vector<Complex> block_in(block_size, Complex(0.0, 0.0));
        for (int i = 0; i < block_size; i++) {
            if (start_idx + i < oldSize) {
                block_in[i] = audioFileComplex[start_idx + i] * hann_window[i];
            }
        }
        
        // --- 2. FFT ---
        std::vector<Complex> fft_data = FFT_Soft(block_in);

        // --- 3. GAUSS EQ (Szűrés) ---
        std::vector<double> gauss = generate_gaussian(freq_axis, preset);
        for (int i = 0; i < SAMPLES_SIZE; i++) {
            // EQ: Hozzáadjuk a basszust az eredetihez
            fft_data[i] += fft_data[i] * gauss[i]; 
        }
        
 
        
        // --- 4. BRAM ÍRÁS ---
        for (int i = 0; i < SAMPLES_SIZE; i++) {
            write_bram(i * 2, fft_data[i].real(), 1);
            write_bram(i * 2 + 1, fft_data[i].imag(), 1);
        }

        // --- 5. HARDVER INDÍTÁS ---
        write_hard(ADDR_CMD, 1);

        // --- 6. VÁRAKOZÁS ---
        bool done = false;
        int timeout = 0;
        while(!done && timeout < 2000) {
            wait(100, sc_core::SC_NS);
            done = read_hard();
            timeout++;
        }

        // --- 7. EREDMÉNY OLVASÁS ---
        std::vector<Complex> ifft_result(SAMPLES_SIZE);
        for (int i = 0; i < SAMPLES_SIZE; i++) {
            double re = read_bram(i * 2, 2);
            double im = read_bram(i * 2 + 1, 2);
            ifft_result[i] = Complex(re, im);
        }

        // --- 8. OVERLAP-ADD ---
        for (int i = 0; i < block_size; i++) {
            int output_index = b * hop_size + i;
            if (output_index < final_audio_size) {
                final_audio[output_index] += ifft_result[i].real(); 
            }
        }
        
        if (b % 500 == 0) std::cout << "." << std::flush;
    }
    std::cout << std::endl;

    // --- 9. VÉGSŐ NORMALIZÁLÁS (PEAK) ---
    // Ez a legfontosabb rész a hangerőhöz!
    std::cout << "[DEBUG] Vegso normalizalas..." << std::endl;
    
    float max_amp = 0.0f;
    for(float s : final_audio) {
        if (std::abs(s) > max_amp) max_amp = std::abs(s);
    }

    if (max_amp > 0.0f) {
        // Normalizálás 0.95-re (-0.5 dB)
        // Ez garantálja, hogy hangos lesz, de nem torzít
        float gain = 1.03f / max_amp;
        std::cout << "[INFO] Max amp: " << max_amp << ", Gain: " << gain << std::endl;
        
        for (size_t i = 0; i < final_audio.size(); i++) {
            final_audio[i] *= gain;
        }
    }

    // --- 10. MENTÉS ---
    AudioFile<float> outputAudio;
    
    
 
        outputAudio.setNumChannels(2);
        outputAudio.setNumSamplesPerChannel(final_audio_size);
        outputAudio.samples[0] = final_audio;
        outputAudio.samples[1] = final_audio; 
    

    outputAudio.setSampleRate(sampleRate);
    //outputAudio.setBitDepth(32); 
    
    if (outputAudio.save("output_fixed.wav")) {
        std::cout << "[SIKER] output_fixed.wav elkeszult!" << std::endl;
    }
    
    sc_core::sc_stop();
}

int Soft::resize(int samples) {
  
    int nth_degree[21] = {
        1024, 2048, 4096, 8192, 16384, 32768, 65536,
        131072, 262144, 524288, 1048576, 2097152, 4194304,
        8388608, 16777216, 33554432, 67108864, 134217728,
        268435456, 536870912, 1073741824
    };
    for (int i = 0; i < 21; i++) {
        if (samples < nth_degree[i]){
            return nth_degree[i];
        }
    }
    return std::min(samples, 1073741824);
}

// FFT Soft implementáció 
std::vector<Complex> Soft::FFT_Soft(const std::vector<std::complex<double>>& in) {
    std::vector<Complex> x(in);
    unsigned int N = x.size(), k = N, n;
    double thetaT = 3.14159265358979323846264338328L / N;
    Complex phiT = Complex(cos(thetaT), -sin(thetaT)), T;
    while (k > 1) {
        n = k;
        k >>= 1;
        phiT = phiT * phiT;
        T = 1.0L;
        for (unsigned int l = 0; l < k; l++) {
            for (unsigned int a = l; a < N; a += n) {
                unsigned int b = a + k;
                Complex t = x[a] - x[b];
                x[a] += x[b];
                x[b] = t * T;
            }
            T *= phiT;
        }
    }
    unsigned int m = (unsigned int)log2(N);
    for (unsigned int a = 0; a < N; a++) {
        unsigned int b = a;
        b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
        b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
        b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
        b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
        b = ((b >> 16) | (b << 16)) >> (32 - m);
        if (b > a) {
            Complex t = x[a];
            x[a] = x[b];
            x[b] = t;
        }
    }
    return x;
}

// BRAM olvasás/írás 
num_t Soft::read_bram(sc_dt::uint64 addr, unsigned char type) {
  pl_t pl; 
  unsigned char buf[BUFF_SIZE]; 
  uint64_t taddr = 0;
  switch(type) {
    case 0: taddr = VP_ADDR_SAMPLES_BASE + addr * BUFF_SIZE; break;
    case 1: taddr = VP_ADDR_COEFFS_BASE + addr * BUFF_SIZE; break;
    case 2: taddr = VP_ADDR_RESULT_BASE + addr * BUFF_SIZE; break;
    default: SC_REPORT_ERROR("Soft::read_bram", "Invalid BRAM type"); break;
  }
  pl.set_address(taddr);
  pl.set_data_length(BUFF_SIZE); 
  pl.set_data_ptr(buf);
  pl.set_command( tlm::TLM_READ_COMMAND );
  pl.set_response_status ( tlm::TLM_INCOMPLETE_RESPONSE );
  interconnect_socket->b_transport(pl,offset);
  return to_fixed(buf); 
}

void Soft::write_bram(sc_dt::uint64 addr, num_t val, unsigned char type) {
  pl_t pl;
  unsigned char buf[BUFF_SIZE]; 
  to_uchar(buf,val);
  uint64_t taddr = 0;
  switch(type) {
    case 0: taddr = VP_ADDR_SAMPLES_BASE + addr * BUFF_SIZE; break;
    case 1: taddr = VP_ADDR_COEFFS_BASE + addr * BUFF_SIZE; break;
    case 2: taddr = VP_ADDR_RESULT_BASE + addr * BUFF_SIZE; break;
    default: SC_REPORT_ERROR("Soft::write_bram", "Invalid BRAM type"); break;
  }
  pl.set_address(taddr);
  pl.set_data_length(BUFF_SIZE);
  pl.set_data_ptr(buf);
  pl.set_command( tlm::TLM_WRITE_COMMAND );
  pl.set_response_status ( tlm::TLM_INCOMPLETE_RESPONSE );
  interconnect_socket->b_transport(pl,offset);
}

bool Soft::read_hard() {
    pl_t pl;
    unsigned char buf[BUFF_SIZE];
    pl.set_address(VP_ADDR_HARD_BASE + ADDR_STATUS);
    pl.set_data_length(BUFF_SIZE);
    pl.set_data_ptr(buf);
    pl.set_command(tlm::TLM_READ_COMMAND);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    sc_core::sc_time offset = sc_core::SC_ZERO_TIME;
    interconnect_socket->b_transport(pl, offset);
    num_t status_val = to_fixed(buf);
    return (status_val != 0.0);
}

void Soft::write_hard(sc_dt::uint64 addr, num_t val) {
  pl_t pl;
  unsigned char buf[BUFF_SIZE];
  to_uchar (buf, val); 
  pl.set_address(VP_ADDR_HARD_BASE + addr);
  pl.set_data_length(BUFF_SIZE);
  pl.set_data_ptr(buf);
  pl.set_command( tlm::TLM_WRITE_COMMAND );
  pl.set_response_status ( tlm::TLM_INCOMPLETE_RESPONSE );
  interconnect_socket->b_transport(pl,offset); 
}

std::vector<Complex> Soft::normalizePeak(const std::vector<Complex>& fftArr, uint8_t preset) {
    
    std::vector<Complex> out(fftArr.size());
    int normalizeValue = *max_element(gauss_amp[preset].begin(), gauss_amp[preset].end()) + 1;
    for (int i = 0; i < fftArr.size(); i++){
       out[i].real(fftArr[i].real() / normalizeValue);
    }
    return out;
}


// Paraméterek: 
// 1. modifiedFftArr (amit szűrtünk)
// 2. fftArrReference (az eredeti, referencia jel)
std::vector<Complex> Soft::normalizePower(const std::vector<Complex>& modifiedFftArr, const std::vector<Complex>& fftArrReference)
{
    double power1 = 0;
    double power2 = 0;
    double multiplier;

    // Eredeti jel energiája (referencia)
    for (const auto& val : fftArrReference) {
        power1 += std::abs(val.real()); 
    }

    // Módosított jel energiája
    for (const auto& val : modifiedFftArr) {
        power2 += std::abs(val.real());
    }

    if (power2 == 0) {
        return modifiedFftArr;
    }

    multiplier = pow(power1 / power2, 2);

    std::vector<Complex> out = modifiedFftArr;
    for (size_t i = 0; i < out.size(); i++) {
        out[i] *= multiplier;
    }

    return out;
}