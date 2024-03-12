
/*
 * Copyright 2024 Kamel Fakih
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *     - Kamel Fakih
 */

#ifndef INTERCONNECT_H
#define INTERCONNECT_H
#include <iomanip>
#include <systemc>
#include <tlm.h>
#include <tlm_utils/peq_with_cb_and_phase.h>
#include "../tlm_memory_manager/memory_manager.h"
#include "../tlm_protocol_checker/tlm2_base_protocol_checker.h"

using namespace sc_core;
using namespace sc_dt;

template<unsigned int I, unsigned int T>
class interconnect : public sc_module, tlm::tlm_bw_transport_if<>, tlm::tlm_fw_transport_if<>
{        
    private:

    sc_time randomDelay()
    {
        unsigned int nanoseconds = rand()%1000;
        return sc_time(nanoseconds, SC_NS);
    }
    
    std::map<tlm::tlm_generic_payload*, int> bwRoutingTable;
    std::map<tlm::tlm_generic_payload*, int> fwRoutingTable;

    // |----- BEGIN REQ ====>|                     | FW
    // |                     |----- BEGIN REQ ---->|
    // |                     |<==== END REQ -------| BW
    // |<---- END REQ -------|                     |
    // |                     |<==== BEGIN RESP ----| BW
    // |<---- BEGIN RESP ----|                     |
    // |----- END RESP =====>|                     |
    // |                     |----- END RESP ----->| FW

    public:

    tlm::tlm_target_socket<> tSocket[T];
    tlm::tlm_initiator_socket<> iSocket[I];

    interconnect(sc_module_name name)
        : sc_module(name)
    {
        for(int i=0; i<I; i++)
        {            
            iSocket[i].bind(*this);       
        }

        for(int i=0; i<T; i++)
        {            
            tSocket[i].bind(*this);       
        }
    }
    SC_HAS_PROCESS(interconnect);    

    int routeFW(int inPort,
                tlm::tlm_generic_payload &trans,
                bool store)
    {
        int outPort = 0;

        // Memory map implementation:
        if(trans.get_address() < 512)
        {
            outPort = 0;
        }
        else if(trans.get_address() >= 512 && trans.get_address() < 1024)
        {
            // Correct Address:
            trans.set_address(trans.get_address() - 512);
            outPort = 1;
        }
        else
        {
            trans.set_response_status( tlm::TLM_ADDRESS_ERROR_RESPONSE );
        }

        if(store)
        {
            bwRoutingTable[&trans] = inPort;  // From where it comes
            fwRoutingTable[&trans] = outPort; // Where it should go
        }

        return outPort;
    }

    virtual void b_transport( tlm::tlm_generic_payload& trans,
                              sc_time& delay )
    {        

        int outPort = routeFW(id, trans, false);
        iSocket[outPort]->b_transport(trans, delay);
    }


    virtual tlm::tlm_sync_enum nb_transport_fw( tlm::tlm_generic_payload& trans,
                                                tlm::tlm_phase& phase,
                                                sc_time& delay )
    {

        sc_assert(id < T);
        int outPort = 0;

        if(phase == tlm::BEGIN_REQ)
        {
            // In the case of nb_transport_fw the address attribute is valid
            // immediately upon entering the function but only when the phase
            // is BEGIN_REQ. Following the return from any forward path TLM-2.0
            // interface method call, the address attribute will have the value
            // set by the interconnect component lying furthest downstream, and
            // so should be regarded as being undefined for the purposes of
            // transaction routing.
            trans.acquire();

            // Modify address accoring to memory map:
            outPort = routeFW(id, trans, true);
        }
        else if(phase == tlm::END_RESP)
        {
            // Adress was already modified in BEGIN_REQ phase:
            outPort = fwRoutingTable[&trans];
            trans.release();
        }
        else
        {
            SC_REPORT_FATAL(name(),"Illegal phase received by initiator");
        }

        cout << "\033[1;37m("
             << name()
             << ")@"  << setfill(' ') << setw(12) << sc_time_stamp()
             << ": Addr = " << setfill('0') << setw(8)
             << dec << trans.get_address()
             << "  inPort = " << dec << setfill(' ') << setw(2) << id
             << " outPort = " << dec << setfill(' ') << setw(2) << outPort
             << " ptr = " << &trans
             << "\033[0m" << endl;


        return iSocket[outPort]->nb_transport_fw(trans, phase, delay);
    }


    virtual tlm::tlm_sync_enum nb_transport_bw( tlm::tlm_generic_payload& trans,
                                                tlm::tlm_phase& phase,
                                                sc_time& delay )
    {        

        sc_assert(id == fwRoutingTable[&trans]);

        int inPort = bwRoutingTable[&trans];

        return tSocket[inPort]->nb_transport_bw(trans, phase, delay);
    }

    // TLM-2 forward DMI method
    virtual bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                                    tlm::tlm_dmi& dmi_data)
    {
        // Dummy method
        return false;
    }

    // TLM-2 debug transport method
    virtual unsigned int transport_dbg(tlm::tlm_generic_payload& trans)
    {
        // Dummy method
        return 0;
    }

    // TLM-2 backward DMI method
    virtual void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                           sc_dt::uint64 end_range)
    {
        // Dummy method
    }
};

#endif