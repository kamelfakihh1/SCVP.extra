
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

class interconnect : public sc_module, tlm::tlm_bw_transport_if<>, tlm::tlm_fw_transport_if<>
{
    private: 

    tlm::tlm_generic_payload* transactionInProgress;
    sc_event targetDone;
    bool responseInProgress;
    tlm::tlm_generic_payload* nextResponsePending;
    tlm::tlm_generic_payload* endRequestPending;
    tlm_utils::peq_with_cb_and_phase<interconnect> tpeq;

    MemoryManager mm;   // socket memory manager
    tlm::tlm_generic_payload* requestInProgress;
    sc_event endRequest;
    tlm_utils::peq_with_cb_and_phase<interconnect> ipeq;

    sc_time randomDelay()
    {
        unsigned int nanoseconds = rand()%1000;
        return sc_time(nanoseconds, SC_NS);
    }

    public:

    tlm::tlm_target_socket<> tSocket;
    tlm::tlm_initiator_socket<> iSocket;

    interconnect(sc_module_name name)
        : sc_module(name),
        tSocket("interconnect transport socket"),
        transactionInProgress(0),
        responseInProgress(false),
        nextResponsePending(0),
        endRequestPending(0),
        tpeq(this, &interconnect::tpeqCallback),
        ipeq(this, &interconnect::ipeqCallback)
    {
        tSocket.bind(*this); 
        iSocket.bind(*this);       
    }
    SC_HAS_PROCESS(interconnect);    

    virtual tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans, 
                                                tlm::tlm_phase& phase,
                                                sc_time& delay)
    {
        // Queue the transaction into the peq until
        // the annotated time has elapsed
        ipeq.notify(trans, phase, delay);

        // HINT: a Return Path shortcut can be implemented here [2.1]

        return tlm::TLM_ACCEPTED; // [1.3, 1.5]
    }

    void interconnect::ipeqCallback(tlm::tlm_generic_payload& trans,
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

            // Allow the memory manager to free the transaction object
            trans.release();
        }
    }

    virtual void b_transport(tlm::tlm_generic_payload& trans,
                             sc_time& delay)
    {
        executeTransaction(trans);
    }

    // [1.0, 1.6]
    virtual tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload& trans,
                                               tlm::tlm_phase& phase,
                                               sc_time& delay)
    {
        // Queue the transaction into the tpeq until
        // the annotated time has elapsed
        tpeq.notify( trans, phase, delay);

        // HINT: Implementation of:
        //       - Return Path Shortcuts [2.0]
        //       - Early Completion [3.0]
        //       - Skip END_REQ [4.0]
        // should be here

        return tlm::TLM_ACCEPTED; // [1.1, 1.7, (1.8)]
    }

    void tpeqCallback(tlm::tlm_generic_payload& trans,
                     const tlm::tlm_phase& phase)
    {
        sc_time delay;

        if(phase == tlm::BEGIN_REQ) // [1.0]
        {
            // Increment the transaction reference count
            trans.acquire();

            if (!transactionInProgress)
            {
                sendEndRequest(trans); // [1.2]
                // HINT: instead of [1.2] we can call also [4.1] (ie. [1.4])
            }
            else
            {
                // Put back-pressure on initiator by deferring END_REQ
                endRequestPending = &trans;
            }
        }
        else if (phase == tlm::END_RESP) // [1.6]
        {
            // On receiving END_RESP, the target can release the transaction
            // and allow other pending transactions to proceed

            if (!responseInProgress)
            {
                SC_REPORT_FATAL("TLM-2",
                   "Illegal transaction phase END_RESP received by target");
            }

            // Flag must only be cleared when END_RESP is sent
            transactionInProgress = 0;

            // Target itself is now clear to issue the next BEGIN_RESP
            responseInProgress = false;
            if (nextResponsePending)
            {
                sendResponse(*nextResponsePending);
                nextResponsePending = 0;
            }

            // ... and to unblock the initiator by issuing END_REQ
            if (endRequestPending)
            {
                sendEndRequest(*endRequestPending);
                endRequestPending = 0;
            }

        }
        else // tlm::END_REQ or tlm::BEGIN_RESP
        {
            SC_REPORT_FATAL(name(), "Illegal transaction phase received");
        }
    }

    void sendEndRequest(tlm::tlm_generic_payload& trans)
    {
        tlm::tlm_phase bw_phase;
        sc_time delay;

        // Queue the acceptance and the response with the appropriate latency
        bw_phase = tlm::END_REQ;
        delay = randomDelay(); // Accept delay

        tlm::tlm_sync_enum status;
        status = tSocket->nb_transport_bw( trans, bw_phase, delay ); // [1.2]
        // Ignore return value (has to be TLM_ACCEPTED anyway)
        // initiator cannot terminate transaction at this point

        // Queue internal event to mark beginning of response
        delay = delay + randomDelay(); // Latency
        targetDone.notify( delay );

        assert(transactionInProgress == 0);
        transactionInProgress = &trans;
    }

    // Method process that runs on targetDone
    void executeTransactionProcess()
    {
        // Execute the read or write commands
        executeTransaction(*transactionInProgress);

        // Target must honor BEGIN_RESP/END_RESP exclusion rule
        // i.e. must not send BEGIN_RESP until receiving previous
        // END_RESP or BEGIN_REQ
        if (responseInProgress)
        {
            // Target allows only two transactions in-flight
            if (nextResponsePending)
            {
                SC_REPORT_FATAL(name(),
                   "Attempt to have two pending responses in target");
            }
            nextResponsePending = transactionInProgress;
        }
        else
        {
            sendResponse(*transactionInProgress);
        }
    }

    // Common to b_transport and nb_transport
    void executeTransaction(tlm::tlm_generic_payload& trans)
    {
        trans.acquire();
        
    }

    void sendResponse(tlm::tlm_generic_payload& trans)
    {
        tlm::tlm_sync_enum status;
        tlm::tlm_phase bw_phase;
        sc_time delay;

        responseInProgress = true;
        bw_phase = tlm::BEGIN_RESP;
        delay = SC_ZERO_TIME;
        status = tSocket->nb_transport_bw( trans, bw_phase, delay ); // [1.4]

        if (status == tlm::TLM_UPDATED) // [2.1]
        {
            // The timing annotation must be honored
            tpeq.notify( trans, bw_phase, delay);
        }
        else if (status == tlm::TLM_COMPLETED) // [3.1]
        {
            // The initiator has terminated the transaction
            transactionInProgress = 0;
            responseInProgress = false;
        }
        // In the case of TLM_ACCEPTED [1.5] we will recv. a FW call [1.6]

        trans.release();
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