cmake_minimum_required(VERSION 2.8.12)
project(pyShm)
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_subdirectory(pybind11)
set(PYBIND11_CPP_STANDARD -std=c++11)

add_library(pyShm MODULE pyShm.cc)
target_link_libraries(pyShm PRIVATE pybind11::module ${Boost_LIBRARIES} -lpthread -lrt)
set_target_properties(pyShm PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")

