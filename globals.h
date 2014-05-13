#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

#include <sstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <GL/glut.h>

#define DIV 1

extern std::stringstream DepthStringStream;
extern std::stringstream ImageStringStream; //(stringstream::in | stringstream::out);
extern cv::Mat RawRGBFrame; 
extern int sock;
extern pthread_mutex_t mutex;
extern char* ServerIp;
extern int ServerPort;
extern int IsDataReady;
extern int fileLength;
extern char charFileLength[10];
extern CvCapture* capture;

extern int done;

#endif // GLOBALS_H_INCLUDED
