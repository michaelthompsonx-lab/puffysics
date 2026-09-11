# Headless examples (no graphics). Built only with
# PUFFYSICS_BUILD_EXAMPLES=ON so core CI stays lean.
add_executable(minimal_cpu minimal_cpu.c)
target_link_libraries(minimal_cpu PRIVATE Puffysics::puffysics)
if(UNIX)
  target_link_libraries(minimal_cpu PRIVATE m)
endif()

add_executable(batch_cpu batch_cpu.c)
target_link_libraries(batch_cpu PRIVATE Puffysics::puffysics)
