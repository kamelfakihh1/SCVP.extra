#include <iostream>
#include <iomanip>
#include <systemc.h>

#include "memory.h"
#include "processor.h"
#include "interconnect.h"


int sc_main (int, char **)
{
    processor cpu0("cpu0");    
    processor cpu1("cpu1");

    memory<512> memory0("memory0");
    memory<512> memory1("memory1");

    interconnect<2,2> bus("bus0");

    cpu0.iSocket.bind(bus.tSocket[0]);
    cpu1.iSocket.bind(bus.tSocket[1]);
    bus.iSocket[0].bind(memory0.tSocket);
    bus.iSocket[1].bind(memory1.tSocket);

    cpu0.iSocket.bind(memory0.tSocket);    

    // std::cout << std::endl << "Name "
    //           << std::setfill(' ') << std::setw(10)
    //           << "Time" << " "
    //           << std::setfill(' ') << std::setw(5)
    //           << "CMD" << "   "
    //           << std::setfill(' ') << std::setw(8)
    //           << "Address"
    //           << "   " << std::hex
    //           << std::setfill(' ') << std::setw(8)
    //           << "Data"
    //           << " " << std::endl
    //           << "-------------------------------------------"
    //           << std::endl;

    sc_start();

    std::cout << std::endl;
    return 0;
}
