#ifndef OUTPUTSIGNAL
#define OUTPUTSIGNAL

#include "general_header.hpp"

#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <string.h>   // String function definitions
#include <sys/ioctl.h>
#include <stdint.h>   // Standard types
#include <stdlib.h>
#include <getopt.h>

extern "C" {
    void on_toggle_output_signal (GtkToggleButton *togglebutton, gpointer data);
}

#endif
