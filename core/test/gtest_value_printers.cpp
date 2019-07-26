#include "gtest_value_printers.h"

using namespace moveit::task_constructor;

::std::ostream& operator<<(::std::ostream& os, const InterfaceFlag& flag) {
	switch (flag) {
		case READS_START:
			return os << "READS_START";
		case READS_END:
			return os << "READS_END";
		case WRITES_NEXT_START:
			return os << "WRITES_NEXT_START";
		case WRITES_PREV_END:
			return os << "WRITES_PREV_END";
		default:
			return os << "unknown";
	}
}

::std::ostream& operator<<(::std::ostream& os, const InterfaceFlags& flags) {
	os << "InterfaceFlags(";
	bool have_previous = false;
	for (InterfaceFlag f : { READS_START, READS_END, WRITES_NEXT_START, WRITES_PREV_END }) {
		if (flags & f) {
			os << (have_previous ? ", " : "{") << f;
			have_previous = true;
		}
	}
	return os << (have_previous ? "})" : ")");
}
