#ifndef SOFT_HPP_
#define SOFT_HPP_
#define _CRT_SECURE_NO_DEPRECATE
#include <cstdint>
#include <iostream>
#include <fstream>
#include <systemc>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <stdio.h>
#include <stdlib.h>
#include "typedefs.hpp"
#include "utils.hpp"
#include <complex>
#include "AudioFile.h"
#include <vector>
#include <string>  

#include <cmath>
#include <algorithm>





using Complex = std::complex<double>;

const uint8_t BASS = 0;
const uint8_t MID = 1;
const uint8_t TREBLE = 2;


class Soft : public sc_core::sc_module
{
public:
  Soft(sc_core::sc_module_name name, char *coefs, char *samples, uint8_t preset_val);
  ~Soft();
  tlm_utils::simple_initiator_socket<Soft> interconnect_socket;
  tlm_utils::simple_target_socket<Soft> soft_target_socket;
  char* coeffs;
  std::string samplesPath;
  std::vector<double> fullOutput;  // ← ide gyűjtjük a kis blokkokból a teljes eredményt
  
  
std::vector<std::vector<int>> gauss_amp = {
        {4,3,2,0,0,0,0,0,0,0},
        {0,0,0,4,4,4,4,0,0,0},
        {4,3,2,0,0,0,0,2,3,4}
    };
    std::vector<uint16_t> gauss_freq = {31,62,125,250,500,1000,2000,4000,8000,16000};
vector<double> gauss;
vector<complex<double>> ifftOutput;
//AudioFile<double> outputAudio;
protected:
   void gen();
  uint8_t preset;
   //void gen(); //main software function
   pl_t pl; //payload
   sc_core::sc_time offset; //time
   //std::vector<num_t> coeffs(COEFFS_SIZE, 0);
   std::ifstream coeffsFile; 
   std::ifstream samplesFile; //input files
   
   std::ofstream output; 

   AudioFile<double> audioFile;              // audiFile (vector)
    vector<complex<double>> audioFileComplex; // audiofile converted to complex
                                              // so it can be used in in fft function
    vector<complex<double>> transform;
    size_t oldSize;
    size_t newSize;
    float sampleRate;
      double sampleRateIn = 0.0;
    int origSamples = 0;
   int nearestPow2(int samples);
  int resize(int samples);


  num_t result[10*SAMPLES_SIZE];
  num_t samplesH[10*SAMPLES_SIZE];
  num_t coeffsH[COEFFS_SIZE];
  num_t fft_res[COEFFS_SIZE];
  num_t inputSamples[BUFFER_LEN];
  num_t gain = 1.0;
    bool start, done;
   //FILE *fd_in;
   //unsigned short input[SAMPLES_SIZE];
   string line;
   int count = 0;
   //unsigned short temp;
 
  //vector<Complex> ifft(vector<Complex> in);    ok
  //vector<Complex> ifft(const vector<Complex>& in);
  //vector<Complex> ifft(const vector<Complex>& in);

   

  num_t read_bram(sc_dt::uint64 addr, unsigned char type); 
  void write_bram(sc_dt::uint64 addr, num_t val, unsigned char type); 

  bool read_hard();
  void write_hard(sc_dt::uint64 addr, num_t val);
 // soft.hpp fájlban, a declaration hozzáadása a bool paraméterhez
void write_to_file(const std::string& filename, const std::vector<std::complex<double>>& data, bool append = true);

   
    std::vector<double> finalOutput;
    double sampleRateGlobal;

    // Új függvényprototípus:
   
    
    std::vector<std::complex<double>> FFT_Soft(const std::vector<std::complex<double>>& in);
    std::vector<double> getFreq(int n, double sampleRate);
    std::vector<double> generate_gaussian(const std::vector<double>& freq, uint8_t preset);
   std::vector<Complex> normalizePeak(const std::vector<Complex>& fftArr, uint8_t preset);
  std::vector<Complex> normalizePower(const std::vector<Complex>& modifiedFftArr, const std::vector<Complex>& fftArr);
  


};

#endif // SOFT_HPP_
