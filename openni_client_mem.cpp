#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>

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
#include <lo/lo.h>

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
IplImage* img = NULL;
IplImage *depthImage = NULL;
cv::Mat rawRGBFrame; 
int is_data_ready = 0;
int sock;
char* server_ip;
int server_port;
int width = 640, height = 480;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

char charFileLength[10];
int fileLength;


void* streamClient(void* arg);
void quit(char* msg, int retval);

char testdata[6] = "ABCDE";

int main(int argc, char **argv) 
{
    pthread_t thread_c;
    char key = '0';
    int i = 0;

    // build a blob object from some data
    //lo_blob btest = lo_blob_new(sizeof(testdata), testdata);

    if (argc != 5) 
	{
        quit("Usage: stream_client <server_ip> <server_port> <width> <height>", 0);
    }

    // Get the server/port parameters
    server_ip = argv[1];
    server_port = atoi(argv[2]);
    //width = atoi(argv[3]);
    //height = atoi(argv[4]);

    /* create image */
    //rawRGBFrame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	//cv::Mat rawRGBFrame(640, 480, CV_8UC3);

    // Run the streaming client as a separate thread
    if (pthread_create(&thread_c, NULL, streamClient, NULL)) 
	{
        quit("pthread_create failed.", 1);
    }

    cv::namedWindow("stream_client", CV_WINDOW_AUTOSIZE);

    char charFileLength[10];

    // an address to send messages to. sometimes it is better to let the server
    lo_address t = lo_address_new(NULL, "7770");

    while(1) 
	{

        //Display the received image, make it thread safe by enclosing it using pthread_mutex_lock
        pthread_mutex_lock(&mutex);
        if (is_data_ready) 
		{
            //cvShowImage("stream_client", rawRGBFrame);
			//cv::Mat RGBFrameMat = cv::Mat(rawRGBFrame, true);
			
			imshow("stream_client", rawRGBFrame);
            //cvReleaseImage(&rawRGBFrame);
			/*			
			vector<int> compression_params;
	    	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	    	compression_params.push_back(80);
	        imwrite("imagetest2.jpg", rawRGBFrame, compression_params);
			*/			
			rawRGBFrame.release();

            is_data_ready = 0;
        }
        pthread_mutex_unlock(&mutex);

		// Wait for keystroke and handle via LiLo server. Not sure if we're going to keep this.        
		key = cvWaitKey(2);
        if (key == 27) 
		{
            //lo_send(t, "/quit", NULL);
            break;
        } 
		else if ((key == 67) || (key == 99)) 
		{
            lo_send(t, "/capture", "s", "c");   //save the jpeg image
        }
    }

    printf("Exit");

    // user has pressed 'q', terminate the streaming client
    if (pthread_cancel(thread_c)) 
	{
        quit("pthread_cancel failed.", 1);
    }

    //free memory
    cvDestroyWindow("stream_client");
    quit(NULL, 0);
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
    server.sin_addr.s_addr = inet_addr(server_ip);
    server.sin_port = htons(server_port);

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

        //write the raw jpeg data to the stringstream
		//printf("Writing data to file stream, fileLength=%d\n", fileLength);
        stringstrm.write(sockdata, fileLength);
		

        //uncompress the raw jpeg data to an IplImage
		// weird conversion problems happening here!
		IplImage *IplJpegImageStream = NULL;
        IplJpegImageStream = readJpeg(stringstrm);
		rawRGBFrame = cv::Mat(IplJpegImageStream);
        //cvCvtColor(rawRGBFrame, rawRGBFrame, CV_RGB2BGR);
		
		//cv::Mat RGBFrameMat = cv::Mat(rawRGBFrame, true);
		//cv::imshow("stream_client", RGBFrameMat);
		// Mark test: output this image to a file
		
		//cv::Mat OutputImage = cv::Mat(rawRGBFrame, true);		
		/*		
		vector<int> compression_params;
    	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    	compression_params.push_back(80);
        imwrite("imagetest.jpg", rawRGBFrame, compression_params);
		*/

        //free the socket data
        free(sockdata);


        pthread_mutex_lock(&mutex);

        //clear the stringstream
        stringstrm.str("");
        is_data_ready = 1;

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
    if (img) cvReleaseImage(&img);

    pthread_mutex_destroy(&mutex);

    exit(retval);
}

