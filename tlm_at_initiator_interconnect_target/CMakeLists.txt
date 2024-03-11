add_executable(target_at_intiator_interconnect_target
main.cpp
memory.h
processor.h
interconnect.h
../tlm_memory_manager/memory_manager.cpp
../tlm_memory_manager/memory_manager.h
../tlm_protocol_checker/tlm2_base_protocol_checker.h
)

target_include_directories(target_at_intiator_interconnect_target
    PRIVATE ${SYSTEMC_INCLUDE}
)

target_link_libraries(target_at_intiator_interconnect_target
    PRIVATE ${SYSTEMC_LIBRARY}
)
