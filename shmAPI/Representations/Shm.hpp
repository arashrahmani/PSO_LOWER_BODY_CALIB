#ifndef SHM_HPP
#define SHM_HPP
#include <boost/interprocess/managed_shared_memory.hpp>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace boost::interprocess;


py::list get(char * segment_name,char * object_name,int size){
    managed_shared_memory shm(open_only,segment_name);
    std::pair<double *,std::size_t> p = shm.find<double>(object_name);
    py::list valus;
    for(auto i=p.first;i<p.first+size;i++){
        valus.append(*i);
    }
    return valus;
}

void set(char * segment_name,char * object_name,std::vector<double> valus){
    managed_shared_memory shm(open_only,segment_name);
    std::pair<double *,std::size_t> p = shm.find<double>(object_name);
    for(auto i=0;i<valus.size();i++){
        p.first[i] =valus[i];
    }
}


// void remove(char * name){
//     shared_memory_object::remove(segments.at(name));  
// }

#define PYSHM(m) \
      m.def("get",&get,py::return_value_policy::copy); \
      m.def("set",&set); \
      
#endif