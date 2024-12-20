#include <pybind11/pybind11.h>
#include "optimization/rehab_optimizer.hpp"  

namespace py = pybind11;

PYBIND11_MODULE(bindings, m) {
    py::class_<RehabOptimizer>(m, "RehabOptimizer")
        .def(py::init<>())
        .def("optimize", &RehabOptimizer::optimize);
}
