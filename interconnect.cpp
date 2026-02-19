
#include "interconnect.hpp"



Interconnect::Interconnect(sc_core::sc_module_name name)
  : sc_module(name)
  , offset(sc_core::SC_ZERO_TIME)
{
  soft_socket.register_b_transport(this, &Interconnect::b_transport);
  SC_REPORT_INFO("Interconnect", "Constructed.");
}

Interconnect::~Interconnect()
{
  SC_REPORT_INFO("Interconnect", "Destroyed.");
}

void Interconnect::b_transport(pl_t &pl, sc_core::sc_time &offset)
{
  sc_dt::uint64 addr = pl.get_address();

  // DEBUG
 // std::cout << "[INTERCONNECT] IN addr = 0x" << std::hex << addr << std::dec << std::endl;

  if (addr >= VP_ADDR_BRAM_L && addr <= VP_ADDR_BRAM_H)
  {
      // A BRAM vezérlő majd eldönti, hogy samples/coeffs/results melyik
      bram_socket->b_transport(pl, offset);
  }
  else if (addr >= VP_ADDR_HARD_L && addr <= VP_ADDR_HARD_H)
  {
      //std::cout << "[INTERCONNECT] forwarding to HARD, addr = 0x" << std::hex << addr << std::dec << std::endl;
      hard_socket->b_transport(pl, offset);
  }
  else
  {
      SC_REPORT_ERROR("Interconnect", "Wrong address.");
      pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
      return;
  }

  //offset += sc_core::sc_time(10, sc_core::SC_NS);
}

