#pragma once
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif
namespace base::detail
{
// Call from the worker itself; Linux nice values apply to the calling TID.
inline bool BackgroundPriority(const char *name)
{
#ifdef _WIN32
    (void)name;
    return ::SetThreadPriority(::GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#elif defined(__linux__)
    pthread_setname_np(pthread_self(), name);
    return setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10) == 0;
#else
    (void)name;
    return true;
#endif
}
} // namespace base::detail
