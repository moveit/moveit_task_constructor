#include <pybind11/pybind11.h>

namespace moveit {
namespace python {

void export_properties(pybind11::module& m);
void export_solvers(pybind11::module& m);
void export_core(pybind11::module& m);
void export_stages(pybind11::module& m);

}  // namespace python
}  // namespace moveit

PYBIND11_MODULE(pymoveit_mtc, m) {
	m.doc() = "MoveIt Task Constructor";

	auto msub = m.def_submodule("core");
	msub.doc() = "MTC core components";
	moveit::python::export_properties(msub);
	moveit::python::export_solvers(msub);
	moveit::python::export_core(msub);

	msub = m.def_submodule("stages");
	msub.doc() = "MTC stages";
	moveit::python::export_stages(msub);
}
