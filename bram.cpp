#include "bram.hpp"
using namespace tlm;
#include <complex>
#include <vector>
#include <cstring>
#include <cmath>
#include "utils.hpp"

// pl. egy 64KB-os területhez:


Bram::Bram(sc_core::sc_module_name name) : sc_module(name)
{
  bram_port_a.register_b_transport(this, &Bram::b_transport);
  bram_port_b.register_b_transport(this, &Bram::b_transport);
 mem.resize(MEM_RESERVED);

  SC_REPORT_INFO("BRAM", "Constructed.");
}

Bram::~Bram()
{
  SC_REPORT_INFO("BRAM", "Destroyed.");
}




void Bram::b_transport(pl_t &pl, sc_core::sc_time &offset)
{
  tlm::tlm_command cmd = pl.get_command();
  sc_dt::uint64 addr = pl.get_address(); // byte offset
  unsigned int len = pl.get_data_length();
  unsigned char *buf = pl.get_data_ptr();


//std::cout << "[BRAM] " << (cmd == TLM_WRITE_COMMAND ? "WRITE" : "READ")
       //   << " addr = 0x" << std::hex << addr
         //<< ", len = " << std::dec << len << std::endl;


  // Ellenőrzés: nem lépjük túl a memóriát byte szinten
  if (addr + len > mem.size() * sizeof(num_t)) {
    pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    return;
  }

  if (cmd == tlm::TLM_WRITE_COMMAND)
  {
    // Írás memóriába - memcpy a megfelelő helyre
    std::memcpy(reinterpret_cast<unsigned char*>(&mem[0]) + addr, buf, len);
    pl.set_response_status(tlm::TLM_OK_RESPONSE);
  }
  else if (cmd == tlm::TLM_READ_COMMAND)
  {
    // Olvasás a memóriából - memcpy a megfelelő helyről
    std::memcpy(buf, reinterpret_cast<unsigned char*>(&mem[0]) + addr, len);
    pl.set_response_status(tlm::TLM_OK_RESPONSE);
  }
  else
  {
    pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
  }

  //offset += sc_core::sc_time(10, sc_core::SC_NS);
}