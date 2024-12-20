#include <pybind11/pybind11.h>
#include "core/model_interface.hpp"  

namespace py = pybind11;

PYBIND11_MODULE(bindings, m) {
    py::class_<ModelInterface>(m, "ModelInterface")
        .def(py::init<std::string>())
        .def("load_model", &ModelInterface::load_model)
        .def("predict", &ModelInterface::predict);
}
