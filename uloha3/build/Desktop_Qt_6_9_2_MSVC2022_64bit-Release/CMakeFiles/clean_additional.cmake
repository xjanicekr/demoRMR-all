# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "demoRMR\\CMakeFiles\\demoRMR_autogen.dir\\AutogenUsed.txt"
  "demoRMR\\CMakeFiles\\demoRMR_autogen.dir\\ParseCache.txt"
  "demoRMR\\demoRMR_autogen"
  )
endif()
