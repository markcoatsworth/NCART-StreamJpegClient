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
#include <opencv/highgui.h>

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


// Global variables
// Any changes made here should be reflected in globals.h

char* ServerIP;
cv::Mat DepthFrame;
cv::Mat RawRGBDFrame; 
int IsDataReady = 0;
int sock;
int ServerPort;
IplImage *IplJpegImageStream;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
struct sigaction SignalActionManager;

std::stringstream DepthStringStream;
std::stringstream ImageStringStream;

char DepthFileLengthString[10];
char ImageFileLengthString[10];
int DepthFileLength;
int ImageFileLength;

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

	// Verify command line arguments
    if (argc != 3) 
	{
        quit("Usage: ./StreamJpegClient <ServerIP> <ServerPort>", 0);
    }

    // Get the server/port parameters
    ServerIP = argv[1];
    ServerPort = atoi(argv[2]);
    
	// Initialize the DepthFrame matrix to 640x480. Note that we do not need to initialize the RawRGBDFrame matrix.
	//cv::Mat DepthFrame(640, 480, CV_8UC3);
	//cout << "DepthFrame.rows=" << DepthFrame.rows << ", DepthFrame.cols=" << DepthFrame.cols << endl;

    // Run the streaming client as a separate thread
    if (pthread_create(&thread_c, NULL, streamClient, NULL)) 
	{
        quit("pthread_create failed.", 1);
    }
	
	// Setup the video display window
    cv::namedWindow("stream_client");//, CV_WINDOW_AUTOSIZE);

	// Debug: set up the depth data output file
	//std::ofstream DepthFile;

	

	// Begin the main client loop which checks for available data, then displays it as a video stream
    while(1) 
	{
        //Display the received image, make it thread safe by enclosing it using pthread_mutex_lock
        pthread_mutex_lock(&mutex);

        if (IsDataReady) 
		{
            // Display image, clear data
			try
			{
				//cout << "DepthFrame.rows=" << DepthFrame.rows << ", DepthFrame.cols=" << DepthFrame.cols << ", DepthFrame.channels=" << DepthFrame.channels() << ", DepthFrame.type=" << DepthFrame.type() << endl;
				//cout << "RawRGBDFrame.rows=" << RawRGBDFrame.rows << ", RawRGBDFrame.cols=" << RawRGBDFrame.cols << ", RawRGBDFrame.channels=" << RawRGBDFrame.channels() << ", RawRGBDFrame.type=" << RawRGBDFrame.type() << endl;
				//imshow("stream_client", RawRGBDFrame);
				cv::hconcat(RawRGBDFrame, DepthFrame, RawRGBDFrame);				
				imshow("stream_client", RawRGBDFrame);
			}
			catch(cv::Exception E)
			{
				printf("Invalid JPEG file, skipping.");
			}
            
			// Clear image + depth string stream
			RawRGBDFrame.release();
			DepthFrame.release();			
			ImageStringStream.str("");			
			DepthStringStream.str("");
        	cvReleaseImage(&IplJpegImageStream);

			// Now wait for the next data
			IsDataReady = 0;
        }
        pthread_mutex_unlock(&mutex);

		// Wait for esc key. Note we *must* have the cvWaitKey here for the cvNamedWindow to show up.
		key = cvWaitKey(27);
		if(key == 27)
		{
			break;
		}
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

    // Connect to server
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
    int i, j, k, bytes;
	std::ofstream DepthFile;

	// Initialize the DepthFrame matrix to 640x480. Note that we do not need to initialize the RawRGBDFrame matrix.
	//cv::Mat DepthFrame(480, 640, CV_8UC3);

    // Start receiving images + depth data
    while(1) 
	{

        // Read number of bytes of image data
        bytes = recv(sock, ImageFileLengthString, 10, 0);
        ImageFileLength =  atoi(ImageFileLengthString);
        sockdata = ( char *) malloc(ImageFileLength);

		cout << "ImageFileLength=" << ImageFileLength << endl;

        // Read image data from the socket
        for (i = 0; i < ImageFileLength; i += bytes) 
		{
            if ((bytes = recv(sock, sockdata + i, (ImageFileLength)-i, 0)) == -1) 
			{

                printf("jpeg recv failed!! %d\n", i);
                quit("jpeg recv failed", 1);
            }
        }

        // Write the raw jpeg data to the stringstream.
		ImageStringStream.write(sockdata, ImageFileLength);
			
        // Uncompress jpeg data to an IplImage, then write it to the RGBD matrix
		IplJpegImageStream = readJpeg(ImageStringStream);
		RawRGBDFrame = cv::Mat(IplJpegImageStream);

		// Clear the socket data before moving on to depth frame
		free(sockdata);

		// Now we have to bring in depth data. Read number of bytes of depth data
		bytes = recv(sock, DepthFileLengthString, 10, 0);
        DepthFileLength = atoi(DepthFileLengthString);		       
		sockdata = ( char *) malloc(DepthFileLength);

		cout << "DepthFileLength=" << DepthFileLength << ", sizeof(sockdata)=" << sizeof(*sockdata) << endl;

        // Read depth data from the socket
        for (i = 0; i < DepthFileLength; i += bytes) 
		{
            if ((bytes = recv(sock, sockdata + i, (DepthFileLength)-i, 0)) == -1) 
			{

                printf("Depth data receive failed. %d\n", i);
                quit("Depth data receive failed.", 1);
            }
			else
			{
				//cout << "Received " << bytes << " bytes of depth data from socket." << endl;
			}
        }

		//cout << "Finished receiving depth data." << endl;

 		// Write the raw depth data to the stringstream.		
		DepthStringStream.write(sockdata, DepthFileLength);
		cout << "Captured " << DepthStringStream.str().size() << " bytes of depth data." << endl;
		
		// Now add the depth data to the DepthFrame matrix	
		DepthFrame = cv::Mat(480, 640, CV_8UC3);
		char MajorDepthValue;
		char MinorDepthValue;

		// Write depth data to file (debugging!)
		/*
		DepthFile.open("depth.data", std::ofstream::out | std::ofstream::app);
		DepthFile << DepthStringStream.rdbuf();
		DepthFile << endl << "newframe" << endl;	
		DepthFile.close();
		*/
		


		for(int i = 0; i < 640*480; i++)
		{
		
			//cout << "Adding depth data, row=" << row << ", col=" << col << endl;
			MajorDepthValue = sockdata[(i*2)];
			MajorDepthValue = sockdata[(i*2)+1];
			//DepthStringStream >> MajorDepthValue >> MinorDepthValue;
			unsigned short DepthValue = ((unsigned short)MajorDepthValue * 256) + (unsigned short)MinorDepthValue;
			
			unsigned short lb = DepthValue/10 % 256;
		    unsigned short ub = DepthValue/10 / 256;
			
			
		    switch (ub) 
			{
		        case 0:
		            DepthFrame.datastart[3*i+2] = 255;
		            DepthFrame.datastart[3*i+1] = 255-lb;
		            DepthFrame.datastart[3*i+0] = 255-lb;
		        break;
		        case 1:
		            DepthFrame.datastart[3*i+2] = 255;
		            DepthFrame.datastart[3*i+1] = lb;
		            DepthFrame.datastart[3*i+0] = 0;
		        break;
		        case 2:
		            DepthFrame.datastart[3*i+2] = 255-lb;
		            DepthFrame.datastart[3*i+1] = 255;
		            DepthFrame.datastart[3*i+0] = 0;
		        break;
		        case 3:
		            DepthFrame.datastart[3*i+2] = 0;
		            DepthFrame.datastart[3*i+1] = 255;
		            DepthFrame.datastart[3*i+0] = lb;
		        break;
		        case 4:
		            DepthFrame.datastart[3*i+2] = 0;
		            DepthFrame.datastart[3*i+1] = 255-lb;
		            DepthFrame.datastart[3*i+0] = 255;
		        break;
		        case 5:
		            DepthFrame.datastart[3*i+2] = 0;
		            DepthFrame.datastart[3*i+1] = 0;
		            DepthFrame.datastart[3*i+0] = 255-lb;
		        break;
		        default:
		            DepthFrame.datastart[3*i+2] = 0;
		            DepthFrame.datastart[3*i+1] = 0;
		            DepthFrame.datastart[3*i+0] = 0;
		        break;
		    } 
		
		}

		/*		
		for(int row = 0; row < 480; row++)
		{
			for(int col = 0; col < 640; col ++)
			{
				//cout << "Adding depth data, row=" << row << ", col=" << col << endl;
				
				DepthStringStream >> MajorDepthValue >> MinorDepthValue;
				unsigned short DepthValue = ((unsigned short)MajorDepthValue * 256) + (unsigned short)MinorDepthValue;
				cout << "MajorDepthValue=" << MajorDepthValue << ", MinorDepthValue=" << MinorDepthValue << ", DepthValue=" << DepthValue << endl;
				
				DepthFrame.at<cv::Vec3b>(row, col)[0] = 60;
				DepthFrame.at<cv::Vec3b>(row, col)[1] = 0;
				DepthFrame.at<cv::Vec3b>(row, col)[2] = 0;
			
			}
		}
		*/

        pthread_mutex_lock(&mutex);

        // Clear the stringstream and free socket data
		//cout << "Clearing image string stream..." << endl;
        //ImageStringStream.str("");
        IsDataReady = 1;
        free(sockdata);
        
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

