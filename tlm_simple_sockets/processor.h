
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

#ifndef PROCESSOR_H
#define PROCESSOR_H
#include <iostream>
#include <iomanip>
#include <systemc>
#include <tlm.h>
#include <random>
#include <tlm_utils/peq_with_cb_and_phase.h>
#include <tlm_utils/multi_passthrough_initiator_socket.h>
#include <tlm_utils/multi_passthrough_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include "../tlm_memory_manager/memory_manager.h"
#include "../tlm_protocol_checker/tlm2_base_protocol_checker.h"
#include "util.h"

#define LENGTH 20

using namespace sc_core;
using namespace sc_dt;

SC_MODULE(processor)
{
    public:
    
    tlm_utils::simple_initiator_socket<processor> iSocket;

    SC_CTOR(processor)
        : iSocket("processor intiator socket"),
        requestInProgress(0),
        peq(this, &processor::peqCallback)
    {
        iSocket.register_nb_transport_bw(this, &processor::nb_transport_bw);
        SC_THREAD(processRandom);    
    }

    private:

    MemoryManager mm;    
    tlm::tlm_generic_payload* requestInProgress;
    sc_event endRequest;
    tlm_utils::peq_with_cb_and_phase<processor> peq;

    void processRandom()
    {
        tlm::tlm_generic_payload* trans;
        tlm::tlm_phase phase;
        sc_time delay;        

        std::default_random_engine randGenerator;    
        std::uniform_int_distribution<uint64_t> distrAddr(0, 1023);
        std::uniform_int_distribution<uint32_t> distData(65, 90);    

        // generate a sequence of random writes
        for(int i=0; i<10; i++)
        {        
            unsigned char *data;
            data = new unsigned char[4];

            // generate random address 
            int addr = distrAddr(randGenerator);
            
            // generat random data
            for(int i=0; i<4; i++)
            {
                data[i] = distData(randGenerator);
            }

            // write a random value to a random address
            tlm::tlm_command cmd = tlm::TLM_WRITE_COMMAND;

            // get a new transaction from memory manager
            trans = mm.allocate();
            trans->acquire();        
            trans->set_command(cmd);
            trans->set_address(addr);
            trans->set_data_ptr(data);
            trans->set_data_length(4);
            trans->set_streaming_width(4);
            trans->set_byte_enable_ptr(0);
            trans->set_dmi_allowed(false);
            trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

            // BEGIN_REQ/END_REQ exclusion rule
            if(requestInProgress)
            {
                wait(endRequest);
            }

            requestInProgress = trans;
            phase = tlm::BEGIN_REQ;
            delay = randomDelay();

            std::cout << "\033[1;31m"
                    << "(I) @"  << std::setfill(' ') << std::setw(12) << sc_time_stamp()
                    << ": " << std::setw(12) << (cmd ? "Write to " : "Read from ")
                    << "Addr = " << std::setw(4) << addr << std::setw(12)
                    << " Data = " << data[0] << data[1] << data[2] << data[3] << "\033[0m" << endl;

            // Non-blocking transport call on the forward path
            tlm::tlm_sync_enum status;


            // Call [1.0]:
            status = iSocket->nb_transport_fw( *trans, phase, delay );

            // Check value returned from nb_transport_fw
            if (status == tlm::TLM_UPDATED) // [2.0] or [4.0]
            {
                // The timing annotation must be honored
                peq.notify(*trans, phase, delay);
            }
            else if (status == tlm::TLM_COMPLETED) // [3.0]
            {
                // The completion of the transaction
                // necessarily ends the BEGIN_REQ phase
                requestInProgress = 0;                        

                // Allow the memory manager to free the transaction object
                trans->release();
            }
            // In the case of TLM_ACCEPTED [1.1] we
            // will recv. a BW call in the future [1.2, 1.4]

            wait(randomDelay());        
        }    
    }

    // [1.2, 1.4]
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                                tlm::tlm_phase& phase,
                                                sc_time& delay)
    {
        // Queue the transaction into the peq until
        // the annotated time has elapsed
        peq.notify(trans, phase, delay);

        // HINT: a Return Path shortcut can be implemented here [2.1]

        return tlm::TLM_ACCEPTED; // [1.3, 1.5]
    }

    void peqCallback(tlm::tlm_generic_payload& trans,
                     const tlm::tlm_phase& phase)
    {
        if (phase == tlm::END_REQ // <-- [1.2, 2.0]
            // or [4.0] --V
                || (&trans == requestInProgress && phase == tlm::BEGIN_RESP))
        {
            // The end of the BEGIN_REQ phase
            requestInProgress = 0;
            endRequest.notify(); // wake up suspended main process
        }
        else if (phase == tlm::BEGIN_REQ || phase == tlm::END_RESP)
        {
            SC_REPORT_FATAL(name(), "Illegal transaction phase received");
        }

        if (phase == tlm::BEGIN_RESP) // [1.4]
        {        
            // Send final phase transition to target
            tlm::tlm_phase fw_phase = tlm::END_RESP;
            sc_time delay = sc_time(randomDelay());
            // [1.6]
            iSocket->nb_transport_fw( trans, fw_phase, delay ); // Ignore return
            
            if(trans.get_command() == tlm::TLM_WRITE_COMMAND)
            {
                checkValue(trans);
            }        

            // Allow the memory manager to free the transaction object
            trans.release();
        }
    }

    void checkValue(tlm::tlm_generic_payload& trans)
    {
        unsigned char *data_expected = trans.get_data_ptr();
        unsigned char data[4];    
        sc_time delay = SC_ZERO_TIME;            
        trans.set_command(tlm::TLM_READ_COMMAND);
        trans.set_data_ptr(data);
        iSocket->b_transport(trans, delay);  

        std::cout << "\033[1;31m"
                    << "(I) @"  << std::setfill(' ') << std::setw(12) << sc_time_stamp()
                    << ": " << std::setw(12) << (trans.get_command() ? "Write to " : "Read from ")
                    << "Addr = " << std::setw(4) << trans.get_address() << std::setw(12)
                    << " Data = " << data[0] << data[1] << data[2] << data[3] << "\033[0m" << endl;

        for(int i=0; i<4; i++)
        {    
            if(data[i] != data_expected[i])
            {
                SC_REPORT_FATAL("processor", "Write operation failed");
            }
        }

        free(data_expected);        
    }    

};

#endif