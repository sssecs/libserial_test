
add_executable(bin_main main.cpp)
add_library(stm32_comms stm32_comms.cpp)

target_link_libraries(stm32_comms PUBLIC serial)
target_link_libraries(bin_main stm32_comms)