add_executable(tlm_simple_sockets
main.cpp
memory.h
processor.h
interconnect.h
../tlm_memory_manager/memory_manager.cpp
../tlm_memory_manager/memory_manager.h
../tlm_protocol_checker/tlm2_base_protocol_checker.h
)

target_include_directories(tlm_simple_sockets
    PRIVATE ${SYSTEMC_INCLUDE}
)

target_link_libraries(tlm_simple_sockets
    PRIVATE ${SYSTEMC_LIBRARY}
)
