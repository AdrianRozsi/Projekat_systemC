#include "bram_ctrl.hpp"
#include <cstring>
#include <cmath>
#include "utils.hpp"
using namespace sc_core;
using namespace tlm;
using namespace sc_dt;

#include "bram_ctrl.hpp"

BramCtrl::BramCtrl(sc_core::sc_module_name name) : sc_module(name)
{
  soft_socket.register_b_transport(this, &BramCtrl::b_transport);
  hard_socket.register_b_transport(this, &BramCtrl::b_transport);  // ugyanaz a handler oké
  SC_REPORT_INFO("BRAM Controller", "Constructed.");
}

BramCtrl::~BramCtrl()
{
  SC_REPORT_INFO("BRAM Controller", "Destroyed.");
}


void BramCtrl::b_transport(pl_t &pl, sc_core::sc_time &offset)
{
    tlm::tlm_command cmd = pl.get_command();
    sc_dt::uint64 addr = pl.get_address();
    sc_dt::uint64 taddr = 0;
    unsigned int len = pl.get_data_length();
    unsigned char *buf = pl.get_data_ptr();
    sc_core::sc_time local_offset = sc_core::SC_ZERO_TIME;

    // Válasszuk ki, melyik BRAM szegmens
    tlm_utils::simple_initiator_socket<BramCtrl>* target_socket = nullptr;

    if (addr >= VP_ADDR_SAMPLES_L && addr <= VP_ADDR_SAMPLES_H) {
        taddr = addr - VP_ADDR_SAMPLES_BASE;
        target_socket = &bram_samples_socket;
    }
    else if (addr >= VP_ADDR_COEFFS_L && addr <= VP_ADDR_COEFFS_H) {
        taddr = addr - VP_ADDR_COEFFS_BASE;
        target_socket = &bram_coeffs_socket;
    }
    else if (addr >= VP_ADDR_RESULT_L && addr <= VP_ADDR_RESULT_H) {
        taddr = addr - VP_ADDR_RESULT_BASE;
        target_socket = &bram_result_socket;
    }
    else {
        SC_REPORT_ERROR("BramCtrl", "Wrong address.");
        pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        return;
    }

    // Debug log
   // std::cout << "[BRAM_CTRL] Global addr = 0x" << std::hex << addr
            //  << ", local addr = 0x" << taddr
            // << ", len = " << std::dec << len << std::endl;

    // Új payload forwardoláshoz
    pl_t pl_local;
    pl_local.set_command(cmd);
    pl_local.set_address(taddr);
    pl_local.set_data_ptr(buf);
    pl_local.set_data_length(len);
    pl_local.set_streaming_width(len);
    pl_local.set_byte_enable_ptr(nullptr);
    pl_local.set_dmi_allowed(false);
    pl_local.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    // Továbbítás a megfelelő BRAM szegmenshez
    (*target_socket)->b_transport(pl_local, local_offset);


    // Válasz visszatöltése
    pl.set_response_status(pl_local.get_response_status());
   // offset += local_offset;
}
