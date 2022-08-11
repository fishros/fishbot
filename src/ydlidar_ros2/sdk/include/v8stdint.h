#ifndef V8STDINT_H_
#define V8STDINT_H_

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#include <sys/stat.h>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif

#if defined(_WIN32) && !defined(__MINGW32__)
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else

#include <stdint.h>

#endif

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


#ifdef _AVR_
typedef uint8_t        _size_t;
#define THREAD_PROC
#elif defined (WIN64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef uint32_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (_M_X64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _size_t;
#define THREAD_PROC
#elif defined (__ICCARM__)
typedef uint32_t       _size_t;
#define THREAD_PROC
#endif

typedef _size_t (THREAD_PROC *thread_proc_t)(void *);

typedef int32_t result_t;

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2

#define INVALID_TIMESTAMP (0)

enum {
  DEVICE_DRIVER_TYPE_SERIALPORT = 0x0,
  DEVICE_DRIVER_TYPE_TCP = 0x1,
};


#define IS_OK(x)    ( (x) == RESULT_OK )
#define IS_TIMEOUT(x)  ( (x) == RESULT_TIMEOUT )
#define IS_FAIL(x)  ( (x) == RESULT_FAIL )


// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif


static volatile sig_atomic_t g_signal_status = 0;

#ifdef HAS_SIGACTION
static struct sigaction old_action;
#else
typedef void (* signal_handler_t)(int);
static signal_handler_t old_signal_handler = 0;
#endif

#ifdef HAS_SIGACTION
inline struct sigaction
set_sigaction(int signal_value, const struct sigaction &action)
#else
inline signal_handler_t
set_signal_handler(int signal_value, signal_handler_t signal_handler)
#endif
{
#ifdef HAS_SIGACTION
  struct sigaction old_action;
  ssize_t ret = sigaction(signal_value, &action, &old_action);

  if (ret == -1)
#else
  signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);

  // NOLINTNEXTLINE(readability/braces)
  if (old_signal_handler == SIG_ERR)
#endif
  {
    const size_t error_length = 1024;
    // NOLINTNEXTLINE(runtime/arrays)
    char error_string[error_length];
#ifndef _WIN32
#if (defined(_GNU_SOURCE) && !defined(ANDROID) &&(_POSIX_C_SOURCE >= 200112L))
    char *msg = strerror_r(errno, error_string, error_length);

    if (msg != error_string) {
      strncpy(error_string, msg, error_length);
      msg[error_length - 1] = '\0';
    }

#else
    int error_status = strerror_r(errno, error_string, error_length);

    if (error_status != 0) {
      throw std::runtime_error("Failed to get error string for errno: " +
                               std::to_string(errno));
    }

#endif
#else
    strerror_s(error_string, error_length, errno);
#endif
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
      error_string);
    // *INDENT-ON*
  }

#ifdef HAS_SIGACTION
  return old_action;
#else
  return old_signal_handler;
#endif
}

inline void trigger_interrupt_guard_condition(int signal_value) {
  g_signal_status = signal_value;
  signal(signal_value, SIG_DFL);
}

inline void
#ifdef HAS_SIGACTION
signal_handler(int signal_value, siginfo_t *siginfo, void *context)
#else
signal_handler(int signal_value)
#endif
{
  // TODO(wjwwood): remove? move to console logging at some point?
  printf("signal_handler(%d)\n", signal_value);

#ifdef HAS_SIGACTION

  if (old_action.sa_flags & SA_SIGINFO) {
    if (old_action.sa_sigaction != NULL) {
      old_action.sa_sigaction(signal_value, siginfo, context);
    }
  } else {
    if (
      old_action.sa_handler != NULL &&  // Is set
      old_action.sa_handler != SIG_DFL &&  // Is not default
      old_action.sa_handler != SIG_IGN) { // Is not ignored
      old_action.sa_handler(signal_value);
    }
  }

#else

  if (old_signal_handler) {
    old_signal_handler(signal_value);
  }

#endif

  trigger_interrupt_guard_condition(signal_value);
}

namespace ydlidar {

inline void init(int argc, char *argv[]) {
  UNUSED(argc);
  UNUSED(argv);
#ifdef HAS_SIGACTION
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  sigemptyset(&action.sa_mask);
  action.sa_sigaction = ::signal_handler;
  action.sa_flags = SA_SIGINFO;
  ::old_action = set_sigaction(SIGINT, action);
  set_sigaction(SIGTERM, action);

#else
  ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
  // Register an on_shutdown hook to restore the old signal handler.
#endif
}
inline bool ok() {
  return g_signal_status == 0;
}
inline void shutdownNow() {
  trigger_interrupt_guard_condition(SIGINT);
}

//inline bool fileExists(const std::string filename) {
//    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
//}

inline bool fileExists(const std::string filename) {
#ifdef _WIN32
  struct _stat info = {0};
  int ret = _stat(filename.c_str(), &info);
#else
  struct stat info = {0};
  int ret = stat(filename.c_str(), &info);
#endif
  return (ret == 0);
  /*return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!*/
}


}// namespace ydlidar


#endif  // V8STDINT_H_
