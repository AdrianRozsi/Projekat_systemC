#include "vp.hpp"

Vp::Vp(sc_core::sc_module_name name, char *coefs, char *samples, uint8_t preset)
  : sc_module(name)
  , soft("Soft", coefs, samples, preset)
  , bram_smpl("Bram_S")
  , bram_coef("Bram_C")
  , bram_result("Bram_R")
  , bram_ctrl("BramCtrl")
  , hard("Hard", bram_result)
  , interconnect("Interconnect")
{
soft.interconnect_socket.bind(interconnect.soft_socket);
interconnect.bram_socket.bind(bram_ctrl.soft_socket);

bram_ctrl.bram_samples_socket.bind(bram_smpl.bram_port_a);
bram_ctrl.bram_coeffs_socket.bind(bram_coef.bram_port_a);
bram_ctrl.bram_result_socket.bind(bram_result.bram_port_a);

// ezt engedd vissza:
bram_ctrl.soft_initiator_socket.bind(soft.soft_target_socket);

bram_ctrl.hard_socket.bind(hard.interconnect_socket);     // új ← működni fog

interconnect.hard_socket.bind(hard.soft_socket);

hard.bram_samples_socket.bind(bram_smpl.bram_port_b);
hard.bram_coeffs_socket.bind(bram_coef.bram_port_b);
hard.bram_result_socket.bind(bram_result.bram_port_b);


  SC_REPORT_INFO("Virtual Platform", "Constructed.");
}

Vp::~Vp()
{
  SC_REPORT_INFO("Virtual Platform", "Destroyed.");
}