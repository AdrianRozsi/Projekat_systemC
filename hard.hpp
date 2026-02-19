#ifndef HARD_HPP_
#define HARD_HPP_

#include <systemc>
#include <math.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include "typedefs.hpp"
#include "utils.hpp"

#define _CRT_SECURE_NO_DEPRECATE
#include <cstdint>
#include <vector>
#include <complex>
#include <cmath>
#include "bram.hpp"
#include <string>  // <-- erre is szükség van soft.hpp miatt

using Complex = std::complex<double>;

class Hard : public sc_core::sc_module
{
public:
  Hard(sc_core::sc_module_name name, Bram& bram_ref);
  ~Hard();

  Bram& bram;

  // Sockets
  tlm_utils::simple_initiator_socket<Hard> bram_samples_socket;
  tlm_utils::simple_initiator_socket<Hard> bram_coeffs_socket;
  tlm_utils::simple_initiator_socket<Hard> bram_result_socket;
  tlm_utils::simple_target_socket<Hard> soft_socket;
   //tlm_utils::simple_initiator_socket<Hard> soft_initiator_socket;
  tlm_utils::simple_initiator_socket<Hard> interconnect_socket;  // vagy bram_ctrl_socket
  // SC thread
  void wait_for_block();
  bool clear_ack; // Soft ACK jelzése BRAM törléshez

  // TLM b_transport callback
  void b_transport(pl_t &pl, sc_core::sc_time &offset);

protected:
  pl_t pl;
  sc_core::sc_time offset;

  bool start, done;

  num_t result[10*SAMPLES_SIZE];
  num_t samplesH[10*SAMPLES_SIZE];
  num_t coeffsH[COEFFS_SIZE];

  num_t inputSamples[BUFFER_LEN];
  num_t gain = 1.0;

  void firFunction();

 // std::vector<std::complex<num_t>> fft(const std::vector<std::complex<num_t>>& in);
std::vector<Complex> ifft(const std::vector<std::complex<double>>& in);
std::vector<Complex> fft(const std::vector<std::complex<double>>& in);
void clear_bram_region(unsigned char type);



  num_t read_bram(int addr, unsigned char type);
  void write_bram(int addr, num_t val, unsigned char type);

  private:
  int block_counter = 0;

};
SC_HAS_PROCESS(Hard);

#endif // HARD_HPP_
