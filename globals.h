#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

#include <sstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <GL/glut.h>

#define DIV 1

extern std::stringstream stringstrm; //(stringstream::in | stringstream::out);
extern int sock;
extern pthread_mutex_t mutex;
extern char* server_ip;
extern int server_port;
extern int width;
extern int height;
extern int is_data_ready;
extern int fileLength;
extern char charFileLength[10];
extern CvCapture* capture;
extern IplImage *img;

extern int done;

#endif // GLOBALS_H_INCLUDED
