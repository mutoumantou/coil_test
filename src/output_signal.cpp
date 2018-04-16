#include "output_signal.hpp"

static int dir = 1;             // direction of current; 1: positive; -1 negative
static int per = 0;             // percentage of current [0, 100]
static int fThread = 0;

/*  */

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init (const char* serialport, int baud) {
    struct termios toptions;    // declare a variable toptions of structure termios
    int fd;

    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NONBLOCK );

    if (fd == -1)  {
        perror("serialport_init: Unable to open port ");
        return -1;
    }

    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        // tcgetattr - get the parameters associated with the terminal
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
        case 4800:   brate=B4800;   break;
        case 9600:   brate=B9600;   break;
    #ifdef B14400
        case 14400:  brate=B14400;  break;
    #endif
        case 19200:  brate=B19200;  break;
    #ifdef B28800
        case 28800:  brate=B28800;  break;
    #endif
        case 38400:  brate=B38400;  break;
        case 57600:  brate=B57600;  break;
        case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;

    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

int serialport_flush(int fd) {
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}

int serialport_writebyte( int fd, uint8_t b) {
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}

//
int serialport_write(int fd, const char* str) {
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

int serialport_read_until ( int fd, char* buf, char until,
                            int buf_max, int timeout) {
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do {
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
#ifdef SERIALPORTDEBUG
        printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0];
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[i] = 0;  // null terminate the string
    return 0;
}

void error(char* msg)
{
    fprintf(stderr, "%s\n",msg);
    exit(EXIT_FAILURE);
}

/* THREAD: output signal thread */
static void* output_signal_THREAD ( void *threadid ) {
    printf("at the start of output_signal_THREAD.\n");

    const int buf_max = 256;

    char serialport[buf_max];
    int baudrate = 9600;  // default
    char quiet=0;
    char eolchar = '\n';
    int timeout = 50;
    char buf[buf_max];
    int rc,n;

    char optarg[] = "/dev/cu.usbmodem1411";             //TODO: change this according to the real COM name
    int fd = serialport_init(optarg, baudrate);
    printf("initialization result is %d.\n", fd);

    /* exit Safe-Start */
    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x83);
    printf("after writing.\n");
    if(rc==-1) error("error writing");
    my_sleep(500);

    //serialport_flush(fd);
    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x85);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    //serialport_flush(fd);
    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x00);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x1E);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    // printf("before writing.\n");
    // rc = serialport_writebyte(fd, (uint8_t)18);
    // printf("after writing.\n");
    // if(rc==-1) error("error writing");
    //
    /*
    memset(buf,0,buf_max);  //
    printf("before reading.\n");
    rc = serialport_read_until(fd, buf, eolchar, buf_max, timeout);
    printf("read string:");
    printf("%s\n", buf);
    for (int i = 0; i < buf_max; i ++) {
        //printf("  %#02x  ", buf[i]);
        printf("  %d  ", buf[i]);
    }
    printf("reading result code %d.\n", rc);
    */
    // memset(buf,0,buf_max);  //
    // printf("before reading.\n");
    // serialport_read_until(fd, buf, eolchar, buf_max, timeout);
    // printf("read string:");
    // printf("%s\n", buf);

    while (fThread) {
        my_sleep (30);			// correspond to 30 Hz camera refresh rate
    }

    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x85);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    //serialport_flush(fd);
    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x00);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    printf("before writing.\n");
    rc = serialport_writebyte(fd, (uint8_t)0x00);
    printf("after writing.\n");
    if(rc==-1) error("error writing");

    printf("at the end of output_signal_THREAD.\n");
}

/* toggle the output signal */
void on_toggle_output_signal (GtkToggleButton *togglebutton, gpointer data) {
    int d = gtk_toggle_button_get_active (togglebutton);
    if (d) {
        pthread_t output_signal_thread;
        fThread = 1;
        pthread_create ( &output_signal_thread, NULL, output_signal_THREAD, NULL);  //start control loop thread
    } else
        fThread = 0;
}
