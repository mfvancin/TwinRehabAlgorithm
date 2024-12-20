#include <pybind11/pybind11.h>
#include "analysis/kinematics_solver.hpp"  

namespace py = pybind11;

PYBIND11_MODULE(bindings, m) {
    py::class_<KinematicsSolver>(m, "KinematicsSolver")
        .def(py::init<>())
        .def("solve", &KinematicsSolver::solve);
}
