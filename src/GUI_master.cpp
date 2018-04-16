#include "GUI_master.hpp"

static int fThread = 0;             // flag for GUI update running
static int fCam    = 0;             // flag for camera on/off
static GtkLabel *GUILabel;          // pointer to time display label
static Mat presentFrame;            // present frame to display

// update GUI time display
static gboolean update_GUI_time (gpointer userdata) {
    time_t rawtime;
    struct tm *info;
    char buffer[80];

    time( &rawtime );

    info = localtime( &rawtime );

    double timeInMilliSec;
	struct timeval curTime;
	gettimeofday ( &curTime, NULL );
    timeInMilliSec = (double) curTime.tv_usec * 1e-3 ; // Current time

    //printf("ms is %d.\n", (int)l_time_ms);
    sprintf(buffer, "%smillisec: %d", asctime(info), (int)timeInMilliSec);

    // asctime: Converts given calendar time std::tm to a textual representation of the following fixed 25-character form: Www Mmm dd hh:mm:ss yyyy\n

    gtk_label_set_text (GUILabel, buffer);
    return G_SOURCE_REMOVE;
}

/* THREAD: GUI refresh thread */
static void* GUI_update_THREAD ( void *threadid ) {
    printf("at the start of GUI_update_THREAD.\n");
    while (fThread) {
        g_main_context_invoke (NULL, update_GUI_time  , NULL);
        my_sleep (30);			// correspond to 30 Hz camera refresh rate
    }
    printf("at the end of GUI_update_THREAD.\n");
}

/* activate GUI refresh thread */
void GUI_master_activate ( GtkLabel *timeLabel ) {
    GUILabel = timeLabel;
    pthread_t GUI_update_thread;
    fThread = 1;
    pthread_create ( &GUI_update_thread, NULL, GUI_update_THREAD, NULL);  //start control loop thread

}

void GUI_master_deactivate(void) {
    fThread = 0;
    my_sleep (1000);
    //usleep(1e5);
}
