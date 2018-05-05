#ifndef UNDERGRAD
#define UNDERGRAD

/* header files */
#include "general_header.hpp"
#include "astar.hpp"
#include "vision.hpp"
#include "amplifier.hpp"
#include "s826_subroutine.h"

/* GUI handles */
extern "C" {
    void on_tB_actuation_toggled (GtkToggleButton *togglebutton, gpointer data);    // start/stop automatic control thread
}

#endif
