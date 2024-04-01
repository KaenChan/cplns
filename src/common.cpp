#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	for (const auto& state : path)
	{
		os << state.location << "\t"; // << "(" << state.is_single() << "),";
	}
	return os;
}


bool isSamePath(const Path& p1, const Path& p2)
{
	if (p1.size() != p2.size())
		return false;
	for (unsigned i = 0; i < p1.size(); i++)
	{
		if (p1[i].location != p2[i].location)
			return false;
	}
	return true;
}

void pin_thread(std::size_t thread_id, std::thread& thread) {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(thread_id, &cpu_set);
    int rc = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpu_set);
    (void)rc;
}