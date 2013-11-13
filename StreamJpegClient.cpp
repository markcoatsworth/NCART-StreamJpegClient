#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <signal.h>

//Socket includes
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

//PThread include
#include <pthread.h>

//OpenCV2.2 includes
#include <opencv2/opencv.hpp>
//#include <opencv/highgui.h>

//Jpeg Lib includes
#include <jpeglib.h>

//Liblo includes
//#include <lo/lo.h>

//Local includes
#include "process_jpeg.h"
#include "globals.h"
#include "osc_handlers.h"


#define OUTPUT_BUF_SIZE 4096 /* choose an efficiently fwrite’able size */

using namespace std;


// *****************************************************************************
// READ ME!!!!!

//global vars - make sure if you edit anything here you edit the globals.h file!
//and if need be, the globals.cpp file
//IplImage* img = NULL;
//IplImage *depthImage = NULL;
cv::Mat RawRGBFrame; 
int IsDataReady = 0;
int sock;
char* ServerIP;
int ServerPort;
//int stream_clientWidth = 640;
//int stream_clientHeight = 480;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
struct sigaction SignalActionManager;

char charFileLength[10];
int fileLength;

void SignalHandler(int sigNum);
void* streamClient(void* arg);
void quit(char* msg, int retval);

//char testdata[6] = "ABCDE";

int main(int argc, char **argv) 
{
    pthread_t thread_c;
    char key = '0';
    int i = 0;

	// Set up signal handler
	SignalActionManager.sa_handler = SignalHandler;
	sigaction(SIGINT, &SignalActionManager, NULL);
	sigaction(SIGPIPE, &SignalActionManager, NULL);

    // build a blob object from some data
    //lo_blob btest = lo_blob_new(sizeof(testdata), testdata);

	// Verify command line arguments
    if (argc != 3) 
	{
        quit("Usage: StreamJpegClient <ServerIP> <ServerPort>", 0);
    }

    // Get the server/port parameters
    ServerIP = argv[1];
    ServerPort = atoi(argv[2]);
    
    // Run the streaming client as a separate thread
    if (pthread_create(&thread_c, NULL, streamClient, NULL)) 
	{
        quit("pthread_create failed.", 1);
    }
	
	//printf("Creating namedWindow\n");
    cv::namedWindow("stream_client");//, CV_WINDOW_AUTOSIZE);
	//printf("NamedWindow created...\n");

    //char charFileLength[10];

    // an address to send messages to. sometimes it is better to let the server
    //lo_address t = lo_address_new(NULL, "7770");

	// Begin the main client loop which checks for available data, then displays it as a video stream
    while(1) 
	{
        //Display the received image, make it thread safe by enclosing it using pthread_mutex_lock
        pthread_mutex_lock(&mutex);

        if (IsDataReady) 
		{
            //cvShowImage("stream_client", RawRGBFrame);
			//cv::Mat RGBFrameMat = cv::Mat(RawRGBFrame, true);
			printf("Caught image, trying to display...\n");
			imshow("stream_client", RawRGBFrame);
            	
			RawRGBFrame.release();

            IsDataReady = 0;
        }
        pthread_mutex_unlock(&mutex);
    }

    printf("Exit");

    // user has pressed 'q', terminate the streaming client
    if (pthread_cancel(thread_c)) 
	{
        quit("pthread_cancel failed.", 1);
    }

    // Free memory
    cv::destroyWindow("stream_client");
    quit(NULL, 0);
}

/**
 * Signal Handler function is designed to handle two signals:
 * SIGINT: close all sockets and shut down gracefully
 * SIGPIPE: ...?
 */

void SignalHandler(int sigNum)
{
	if(sigNum == SIGINT)
	{
		printf("Caught SIGINT, exiting gracefully...\n");
		quit(NULL, 0);
	}
	else if(sigNum == SIGPIPE)
	{
		printf("Caught SIGPIPE yall\n");
	}
}


/**
 * This is the streaming client, run as separate thread
 */
void* streamClient(void* arg) 
{
    struct sockaddr_in server;

    //make this thread cancellable using pthread_cancel()
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    //create socket
    if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) 
	{
        quit("socket() failed.", 1);
    }

    // setup server parameters
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(ServerIP);
    server.sin_port = htons(ServerPort);

    printf("Rec'v stream\n");

    //connect to server
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) 
	{
        quit("connect() failed.", 1);
    } 
	else 
	{
        printf("connect successful\n");
    }

    char *sockdata;
    char lengthBuff[4];
    int fileLength;
    int i, j, k, bytes;

    /* start receiving images */
    while(1) 
	{

        //get how many bytes in the next frame
        bytes = recv(sock, charFileLength, 10, 0);
        fileLength =  atoi(charFileLength);
        sockdata = ( char *) malloc(fileLength);

        // get raw data from the frame
        for (i = 0; i < fileLength; i += bytes) 
		{
            if ((bytes = recv(sock, sockdata + i, (fileLength)-i, 0)) == -1) 
			{

                printf("jpeg recv failed!! %d\n", i);
                quit("jpeg recv failed", 1);
            }
        }

        // Write the raw jpeg data to the stringstream
		stringstrm.write(sockdata, fileLength);
		

        // Uncompress jpeg data to an IplImage, then to a Mat
		IplImage *IplJpegImageStream = NULL;
        IplJpegImageStream = readJpeg(stringstrm);
		RawRGBFrame = cv::Mat(IplJpegImageStream);

        //free the socket data
        free(sockdata);


        pthread_mutex_lock(&mutex);

        //clear the stringstream
        stringstrm.str("");
        IsDataReady = 1;

        //the frame came in well, unlock the thread and proceed to get the next frame.
        pthread_mutex_unlock(&mutex);

        // have we terminated yet?
        pthread_testcancel();
        bytes = 0;

        // no, take a rest for a while
        usleep(1000);
    }
}

/**
 * This function provides a way to exit nicely from the system
 */
void quit(char* msg, int retval) 
{
    if (retval == 0) 
	{
        fprintf(stdout, (msg == NULL ? "" : msg));
        fprintf(stdout, "\n");
    } 
	else 
	{
        fprintf(stderr, (msg == NULL ? "" : msg));
        fprintf(stderr, "\n");
    }

    if (sock) close(sock);
    //if (img) cvReleaseImage(&img);

    pthread_mutex_destroy(&mutex);

    exit(retval);
}

