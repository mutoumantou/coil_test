#include "vision.hpp"

static int fThread = 0;             // flag of whether or not thread is running
static Mat raw_frame = Mat(1024,1280,CV_8UC3);
static int raw_frame_lock = 0;

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
