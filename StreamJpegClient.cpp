#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <signal.h>
#include <time.h>

// Socket includes
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// PThread include
#include <pthread.h>

// OpenCV2.2 includes
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

// zLib includes
#include <zlib.h>
#include <bzlib.h>
#include <snappy.h>

// Jpeg Lib includes
#include <jpeglib.h>

// Local includes
#include "process_jpeg.h"
#include "globals.h"
#include "osc_handlers.h"

// Global constants
#define DISPLAY_RGB_STREAM 1
#define DISPLAY_DEPTH_STREAM 1
#define FILE_OUTPUT 1

using namespace std;


// Global variables
// Any changes made here should be reflected in globals.h

char* ServerIP;
char DepthCompressionLibrary[10] = "snappy"; // can be "zlib", "snappy", or "none"
char DepthFileLengthString[10];
char ImageFileLengthString[10];
char StreamDataFileName[40];

cv::Mat DepthFrame;
cv::Mat RawRGBDFrame;

int DepthFileLength;
int ImageFileLength;
int IsDataReady = 0;
int NumFramesCaptured = 0;
int sock;
int ServerPort;

IplImage *IplJpegImageStream;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
struct sigaction SignalActionManager;

std::ofstream StreamDataFile;
std::stringstream DepthStringStream;
std::stringstream ImageStringStream;

time_t StartTime;
time_t CurrentTime;
double RunningTime;
float CurrentFPS;


// Function declarations

void SignalHandler(int sigNum);
void* streamClient(void* arg);
cv::Mat BuildRGBDepthFrame(char* _depthData, int _depthFileLength);
void quit(char* msg, int retval);


// Main program

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
    
    // Run the streaming client as a separate thread
    if (pthread_create(&thread_c, NULL, streamClient, NULL)) 
	{
        quit("pthread_create failed.", 1);
    }

	// Determine name of stream data capture file
	struct tm* CurrentDateTime;
	time_t RawTime;
	time(&RawTime);
	CurrentDateTime = localtime(&RawTime);
	strftime(StreamDataFileName, 40, "stream_data_%Y-%m-%d_%H%M%S.csv", CurrentDateTime);
	
	// Setup the video display window
    cv::namedWindow("stream_client");//, CV_WINDOW_AUTOSIZE);

	// Begin the main client loop which checks for available data, then displays it as a video stream
    while(1) 
	{
        //Display the received image, make it thread safe by enclosing it using pthread_mutex_lock
        pthread_mutex_lock(&mutex);

        if (IsDataReady) 
		{
			// Record the current time, increment the capture count and determine current framerate
			NumFramesCaptured++;
			time(&CurrentTime);
			RunningTime = difftime(CurrentTime, StartTime);
			CurrentFPS = (float)NumFramesCaptured / (float)RunningTime;
			//cout << "NumFramesCaptured=" << NumFramesCaptured << ", Time = " << RunningTime << ", FPS=" << CurrentFPS << endl;

			// Output the frame rate to file
			if(FILE_OUTPUT == 1)
			{
				StreamDataFile.open(StreamDataFileName, std::ofstream::out | std::ofstream::app);
				StreamDataFile << DepthCompressionLibrary << "," << NumFramesCaptured << "," << RunningTime << endl;
				StreamDataFile.close();
			}

			// Put the FPS value onto the image frame
			stringstream FPSString (stringstream::in | stringstream::out);
			FPSString << fixed << setprecision(2) << CurrentFPS << " fps";
			cv::putText(RawRGBDFrame, FPSString.str(), cvPoint(15, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);

            // Merge image + depth data into single frame, display in viewer window
			try
			{
				if(DISPLAY_DEPTH_STREAM == 1)
				{
					cv::hconcat(RawRGBDFrame, DepthFrame, RawRGBDFrame);				
				}				
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

	// Record the start time
	time(&StartTime);

    // Start receiving images + depth data
	cout << "Starting to receive images + depth data" << endl;
    while(1) 
	{

        // Read number of bytes of image data
        bytes = recv(sock, ImageFileLengthString, 10, 0);
        ImageFileLength =  atoi(ImageFileLengthString);
        sockdata = ( char *) malloc(ImageFileLength);

		//cout << "ImageFileLength=" << ImageFileLength << endl;

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

		//cout << "DepthFileLength=" << DepthFileLength << ", sizeof(sockdata)=" << sizeof(*sockdata) << endl;

        // Read depth data from the socket
        for (i = 0; i < DepthFileLength; i += bytes) 
		{
            if ((bytes = recv(sock, sockdata + i, (DepthFileLength)-i, 0)) == -1) 
			{

                printf("Depth data receive failed. %d\n", i);
                quit("Depth data receive failed.", 1);
            }
        }
		
		// Now add the depth data to the DepthFrame matrix	
		DepthFrame = cv::Mat(480, 640, CV_8UC3);
		if(DISPLAY_DEPTH_STREAM == 1)
		{
			DepthFrame = BuildRGBDepthFrame(sockdata, DepthFileLength);		
		}

        pthread_mutex_lock(&mutex);

		// Reset IsDataReady, clear socket data 
        IsDataReady = 1;
        free(sockdata);
        
		// The frame came in well, unlock the thread and proceed to get the next frame.
        pthread_mutex_unlock(&mutex);

        // Have we terminated yet?
        pthread_testcancel();
        bytes = 0;

        // no, take a rest for a while
        usleep(1000);
    }
}

/**
 * Reads depth data from a character pointer, decompresses it if necessary, then returns a cv::Mat object with an RGB visualization
**/

cv::Mat BuildRGBDepthFrame(char* _depthData, int _depthFileLength)
{
	// Decompression variables
	
	unsigned char* DataUncompressed;

	// Decompress the depth data from sockdata
	if(strcmp(DepthCompressionLibrary, "zlib") == 0)
	{
		ulong SizeDataUncompressed = 640*480*2;
		DataUncompressed = (unsigned char*)malloc(SizeDataUncompressed);
		
		int z_result = uncompress(DataUncompressed, &SizeDataUncompressed, (unsigned char*)_depthData, _depthFileLength);

		switch(z_result)
		{
			case Z_OK:
				printf("Decompression successful! depthFileLength=%d\n", _depthFileLength);
				break;
			case Z_BUF_ERROR:
				printf("Decompression error: output buffer wasn't large enough!\n");
				break;
		}
	}
	else if(strcmp(DepthCompressionLibrary, "bzip2") == 0)
	{
		uint SizeDataUncompressed = 640*480*2;
		DataUncompressed = (unsigned char*)malloc(SizeDataUncompressed);

		int bz_result = BZ2_bzBuffToBuffDecompress((char*)DataUncompressed, &SizeDataUncompressed, _depthData, _depthFileLength, 0, 0);

		switch(bz_result)
		{
			case BZ_OK:
				printf("Decompression successful! depthFileLength=%d\n", _depthFileLength);
				break;
		
		}
	}
	else if(strcmp(DepthCompressionLibrary, "snappy") == 0)
	{
		ulong SizeDataUncompressed = 640*480*2;
		std::string DataCompressed(_depthData, _depthFileLength);
		std::string DataUncompressedStdString;

		DataUncompressed = (unsigned char*)malloc(SizeDataUncompressed);
		DataUncompressedStdString.resize(SizeDataUncompressed);
		try 
		{
			snappy::Uncompress(DataCompressed.data(), _depthFileLength, &DataUncompressedStdString);
		}
		catch(cv::Exception E)
		{
			cout << "Depth data error, could not uncompress." << endl;
		}

		for(int i = 0; i < 614400; i ++)
		{
			DataUncompressed[i] = DataUncompressedStdString.at(i);
		}		
		//cout << "String length=" << DataUncompressedStdString.length() << ", size=" << DataUncompressedStdString.size() << endl;
		//unsigned char* TempString = new unsigned char[ 640*480*2 ];
		//strcpy( DataUncompressed, DataUncompressedStdString.c_str() );
		//DataUncompressed = (unsigned char*)DataUncompressedStdString.c_str();
		cout << "strlen(DataUncompressed)=" << strlen((char*)DataUncompressed) << endl;
		
		//printf("DataUncompressed=%s\n", DataUncompressed);
	}
	else if(strcmp(DepthCompressionLibrary, "none") == 0)
	{
		// If no compression used, simply point DataUncompressed to _depthData to save time
		DataUncompressed = (unsigned char*)_depthData;
	}

	// Declare variables used to build depth RGB image
	char MajorDepthValue;
	char MinorDepthValue;
	cv::Mat DepthFrame(480, 640, CV_8UC3);

	// Build DepthFrame from the uncompressed data
	for(int i = 0; i < 640*480; i++)
	{	
		//cout << "i=" << i;
		MajorDepthValue = DataUncompressed[(i*2)+1];
		MinorDepthValue = DataUncompressed[(i*2)];
		
		unsigned short DepthValue = ((unsigned short)MajorDepthValue * 256) + (unsigned short)MinorDepthValue;			
		unsigned short lb = DepthValue/5 % 256;
	    unsigned short ub = DepthValue/5 / 256;
		//cout << ", DepthValue=" << DepthValue << endl;
		
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

	return DepthFrame;

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

