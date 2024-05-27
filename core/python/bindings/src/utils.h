#include <pybind11/pybind11.h>

namespace moveit {
namespace python {
namespace {

// utility function to normalize index: negative indeces reference from the end
size_t normalize_index(size_t size, long index) {
	if (index < 0)
		index += size;
	if (index >= long(size) || index < 0)
		throw pybind11::index_error("Index out of range");
	return index;
}

// implement operator[](index)
template <typename T>
typename T::value_type get_item(const T& container, long index) {
	auto it = container.begin();
	std::advance(it, normalize_index(container.size(), index));
	return *it;
}

}  // namespace
}  // namespace python
}  // namespace moveit
