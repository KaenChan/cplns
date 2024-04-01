#pragma once

#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <map>
#include <assert.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <fstream>
#include <atomic>
#include <iostream>     // std::cout, std::fixed


namespace SysInfo {
    struct RuntimeInfo {int cpu = -1; double vmUsage = 0; double residentSetSize = 0;};
    enum SubprocessMode {RECURSE, FLAT};

	// std::map<long, Proc::CpuInfo> Proc::_cpu_info_per_tid;
	// Mutex Proc::_cpu_info_lock;

	pid_t getPid() {
		return getpid();
	}

	pid_t getParentPid() {
		return getppid();
	}

	long getTid() {
		return syscall(SYS_gettid);
	}


	// https://stackoverflow.com/a/671389
	RuntimeInfo getRuntimeInfo(pid_t pid, SubprocessMode mode) {

		using std::ios_base;
		using std::ifstream;
		using std::string;

		RuntimeInfo info;

		auto statFile = "/proc/" + std::to_string(pid) + "/stat";
		ifstream stat_stream(statFile.c_str(), ios_base::in);
		if (!stat_stream.good()) return info;

		// dummy vars for leading entries in stat that we don't care about
		string str_pid, comm, state, ppid, pgrp, session, tty_nr;
		string tpgid, flags, minflt, cminflt, majflt, cmajflt;
		string utime, stime, cutime, cstime, priority, nice;
		string O, itrealvalue, starttime;

		// the two fields we want
		unsigned long vsize = 0;
		long rss = 0;

		stat_stream >> str_pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
					>> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
					>> utime >> stime >> cutime >> cstime >> priority >> nice
					>> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

		stat_stream.close();

		long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
		info.vmUsage = vsize / 1024.0;
		info.residentSetSize = rss * page_size_kb;
		info.cpu = sched_getcpu();

		// Recursively read memory usage of all children
		if (mode == RECURSE) {
			auto childFile = "/proc/" + std::to_string(pid) + "/task/" + std::to_string(pid) + "/children";
			ifstream children_stream(childFile.c_str(), ios_base::in);
			if (!children_stream.good()) return info;

			pid_t childPid;
			while (children_stream >> childPid) {
				RuntimeInfo childInfo = getRuntimeInfo(childPid, mode);
				info.vmUsage += childInfo.vmUsage;
				info.residentSetSize += childInfo.residentSetSize;
			}
		}

		return info;
	}

    float getMemUsage() 
	{
		auto info = getRuntimeInfo(getPid(), SubprocessMode::FLAT);
		const float r = 1 / 1024. / 1024.;
		info.vmUsage *= r;
		info.residentSetSize *= r;
		return info.residentSetSize;
	}

	float getMemUsageRate()
	{
		struct sysinfo s_info;
		if(sysinfo(&s_info) == 0)
		{
			auto pid_info = getRuntimeInfo(getPid(), SubprocessMode::FLAT);
			float rate = pid_info.residentSetSize*1024.f / s_info.totalram;
			// log(0, "mem_usage %.2f / %.2f = %.4f\n", pid_info.residentSetSize/1024./1024., s_info.totalram/1024./1024./1024., rate);
			return rate;
		}
		return 0;
	}
}