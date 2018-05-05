#include "vision.hpp"

/* file-wide variable */
static int fThread = 0;             // flag of whether or not thread is running
static Mat raw_frame = Mat(1024,1280,CV_8UC3);
static int raw_frame_lock = 0;

static int centerP_dataSafeLock = 0;                    // 1: centerP is being changed, wait until it is done
static float centerPointCoorArray[3]  = {320, 240, 0};  // Current pose of robot
static float centerPointCoorArray_2[3]  = {320, 140, 0}; // Current pose of cargo
static float angle1;                                    // robot angle ?

static Point mouse, mouseC, mouseR;
static float goalPointCoorArray[2]   = {320, 240};


//static FWcamera cam; // create a new instance of FWcamera
static Mat presentFrame = Mat(480,640,CV_8UC3);            // present captured frame
static Mat frameForDisplay = Mat(480,640,CV_8UC3);         // different with presentFrame by annotation/drawing
static int copyLock = 0;

int GetImage(void *buffer, unsigned int buffersize, void *context){
    if(buffersize > 0){ // [Tianqi] <= 0 indicates a bad frame is coming. e.g. the first few frames after initializing the camera
        Mat temp = Mat(1024,1280, CV_8UC3, buffer);
        while (raw_frame_lock);
        raw_frame_lock = 1;
        temp.copyTo(raw_frame);
        raw_frame_lock = 0;
    }
}

int isFrameValid (void) {
    while (raw_frame_lock);
    raw_frame_lock = 1;
    Size s = raw_frame.size();
    int rows = s.height;
    int cols = s.width;
    raw_frame_lock = 0;

    if (rows > 0 && cols > 0){
        return 1;
    } else{
        //printf("frame not valid.\n");
        return 0;
    }
}

int initCamera(int CAM_ID){
    int result;
    result = NETUSBCAM_Init();		// look for ICubes
	if(result==0){
		printf("No device\n");
		return 0;
    }
	result = NETUSBCAM_Open(CAM_ID);	// open camera
	if(result!=0){
		printf("Error: Open; Result = %d\n", result);
		return 0;
    }
    // set the camera clock lower, if a lot of bad frames arriving
	result = NETUSBCAM_SetCamParameter(CAM_ID,REG_PLL,20);
	if(result!=0){
		printf("Error: REG_PLL; Result = %d\n", result);
		return 0;
    }
	// if active, badframes are sent to the callback with buffersize = 0
	result = NETUSBCAM_SetCamParameter(CAM_ID,REG_CALLBACK_BR_FRAMES,1);
	if(result!=0){
		printf("Error: REG_CALLBACK_BR_FRAMES; Result = %d\n", result);
		return 0;
    }
    // set the callback to get the frame buffer
  	result = NETUSBCAM_SetCallback(CAM_ID,CALLBACK_RGB,&GetImage,NULL);
  	if(result!=0){
  		printf("Error: SetCallback; Result = %d\n", result);
  		return 0;
    }
  	 // start streaming of camera
  	result = NETUSBCAM_Start(CAM_ID);
  	if(result!=0){
  		printf("Error: Start; Result = %d\n", result);
  		return 0;
    }
    return 1;
}

int closeCamera(int CAM_ID){
    int result;
    // stop streaming of camera
    result = NETUSBCAM_Stop(CAM_ID);
    if(result!=0){
      printf("Error: Stop; Result = %d\n", result);
      return 0;
    }
    // close camera
    result = NETUSBCAM_Close(CAM_ID);
    if(result!=0){
      printf("Error: Close; Result = %d\n", result);
      return 0;
    }
    return 1;
}

static void* video_stream_THREAD ( void *threadid ) {
    printf("at the start of video_stream_THREAD.\n");
    // Vision_Master myVision;
    //int i = 0;
    if(!initCamera(0)){
        printf("error in init. camera\n");
    }

    while ( fThread ) {
        //printf("start of loop\n");
        if ( !isFrameValid() ) continue; // continue if receiving an empty frame

        //imshow("camera",raw_frame);
        //char c = (char)waitKey(15); // duration of the frames (ms)
        //if (c == 27) break; // exit when pressing ESC key

        while (raw_frame_lock);
        raw_frame_lock = 1;
        //raw_frame.copyTo(frameForDisplay);
        //printf("before copy\n");
        //if (raw_frame.data == NULL)
        //    printf("raw frame is null.\n");
        resize(raw_frame, frameForDisplay, Size(640,480) );
        //printf("after copy\n");
        raw_frame_lock = 0;

        my_sleep(30);           // correspond to 30 Hz camera rate
        //printf("later in loop %d\n", i++);
    }

    closeCamera(0);
    //return 0;

    //if (!fSim) {
        // cam.stopGrabbingVideo();
    	// my_sleep(100);
    	// cam.deinitialize();
        // cam.deinitialize();
    //}

    printf("at the end of video_stream_THREAD.\n");
}

/* activate the video capture and processing thread */
void camera_activate (void) {
    fThread = 1;
    pthread_t videoStreamThread;
    pthread_create( &videoStreamThread, NULL, video_stream_THREAD, NULL);  //start vision thread
}

void camera_deactivate (void) {
    fThread = 0;
}

void get_present_image ( Mat * container ) {
    while (raw_frame_lock);
    raw_frame_lock = 1;
    frameForDisplay.copyTo(*container);
    raw_frame_lock = 0;
}

void stop_video_stream(void) {
    fThread = 0;
}

float *get_robot_pose(void){
	//	int *centerPointCoorArray  = (int*) malloc(2*sizeof(int));
	//	printf("TEST1\n");
	while (centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
	    // centerPointCoorArray[0] =  centerP_adjusted.x; // Set to global variables  UNCOMMENT
	    // centerPointCoorArray[1] =  480 - centerP_adjusted.y;   // do not forget 480 offset UNCOMMENT
		// centerPointCoorArray[1] =  centerP_adjusted.y;
		centerPointCoorArray[0] = 520; // Test x coordinate
		centerPointCoorArray[1] = 360; // Test y coordinate

		centerPointCoorArray[2] = angle1; // Set to global angle variable
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);
	    return centerPointCoorArray;
}

float *get_cargo_pose(void){
	while(centerP_dataSafeLock);                   // wait until change is done
		centerP_dataSafeLock = 1;
		// centerPointCoorArray_2[0] =  centerP_adjusted_2.x; // Set to global variables
		// centerPointCoorArray_2[1] =  480 - centerP_adjusted_2.y;   // do not forget 480 offset
		// centerPointCoorArray_2[1] = centerP_adjusted_2.y;   // do not forget 480 offset

		centerPointCoorArray_2[0] = 120; // Test x coordinate
		centerPointCoorArray_2[1] = 100; // Test y coordinate

		centerPointCoorArray_2[2] = 0;
		// centerPointCoorArray_2[2] = angle2; // Eventually use for CV
		centerP_dataSafeLock = 0;
		// printf("x= %f, y= %f", centerPointCoorArray[0], centerPointCoorArray[1]);
		return centerPointCoorArray_2;
}

float * getGoalPointCoor(void){
    if(mouse.x>0){
        goalPointCoorArray[0] = (float)mouse.x;
        goalPointCoorArray[1] = (float)(480 - mouse.y);   // Note the positive y dir. Note resolution difference between cameras
    }
    return goalPointCoorArray;
}

gboolean key_event (GtkWidget *widget, GdkEventKey *event) {
    //printf ("key pressed %s\n", gdk_keyval_name( event->keyval ) );
    int keycode = -1;

    switch (event->keyval) {
        case 65363: keycode = 0; break;
        case 65362: keycode = 1; break;
        case 65361: keycode = 2; break;
        case 65364: keycode = 3; break;
        case    32: keycode = -1; break;
    }

    //set_directionCode (keycode);
    return TRUE;
}

gboolean key_event_release (GtkWidget *widget, GdkEventKey *event) {
    return TRUE;
}

void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) {
    switch(whichMouse) {
    	case 1: //left mouse
    		mouse.x 		= mouseClick[0];
    		mouse.y 		= mouseClick[1];
    		break;
    	case 2: //right mouse
    		mouseR.x 		= mouseClick[0];
    		mouseR.y		= mouseClick[1];
    		break;
    	case 3: //center mouse
    		mouseC.x 		= mouseClick[0];
    		mouseC.y 		= mouseClick[1];
    		break;
    }

}

gboolean on_eventbox1_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data) {
    int click[2];
	click[0] = (int)event->x; //x position from top left in pixels
	click[1] = (int)event->y; //y position from top left in pixels
	int button_click = event->button; //which mouse button was clicked
	//g_print("Top video window %d click at location [%d %d].\n", button_click, click[0], click[1]);
	setMouse (0, button_click, click );      //void setMouse(int whichScreen, int whichMouse, int mouseClick[2] ) //click in pixels
}
