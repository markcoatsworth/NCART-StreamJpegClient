#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "libfreenect.h"
#include "libfreenect_cv.h"
#include "libfreenect_sync.h"

#include <GL/glut.h>
#include "kinect_point_cloud.h"
/******* Global Variables ************/
//
//double transFudgedVals[] = 		{-4.0051469877240551e-02,  -0.6296501168410337e-02,  -5.6312216296436068e-03 };
//
//double rgbIntrinsicVals[] =		{ 5.1997666258124059e+02,  0., 3.2035780640775630e+02, 0.,
//								  5.1906823659381246e+02,  2.5225475675063598e+02, 0., 0., 1.};
//
//double rgbDistortionVals[] =	{ 2.3282269533065048e-01, -9.2676911644431559e-01,
//								  1.3785293110664286e-03, -3.0566975301827910e-03, 1.4220722610860992e+00 };
//double dIntrinsicVals[] =		{ 5.8607137222498295e+02,  0., 3.1056890695602476e+02, 0.,
//								  5.8567560309796363e+02,  2.4912535666434175e+02, 0., 0., 1.};
//double dDistortionVals[] =		{-2.1835400781694411e-01,  1.7910224568641626e+00,
//								  3.8831737257415703e-03, -7.9254527156540370e-04, -5.6385675132989217e+00 };
//double rotationVals[] =			{ 9.9997653425563382e-01,  6.8485245163549995e-03,
//								 -1.6926322742441872e-04, -6.8493271512092896e-03,
//								  9.9996243174913368e-01, -5.3124201579767246e-03,
//								  1.3287462880795254e-04,  5.3134548373026313e-03,
//								  9.9998587467125000e-01 };
//double translationVals[] = 		{ 2.3951469877240551e-02, -3.6296501168410337e-03,  -5.6312216296436068e-03 };
/*
double transFudgedVals[] = 	{-1.0051469877240551e-02, 3.6296501168410337e-02,  -5.6312216296436068e-03 };


double rgbIntrinsicVals[] = {	5.1997666258124059e+02, 0., 3.2035780640775630e+02, 0.,
								5.1906823659381246e+02, 2.5225475675063598e+02, 0., 0., 1.};
double rgbDistortionVals[] = {	2.3282269533065048e-01, -9.2676911644431559e-01,
									1.3785293110664286e-03, -3.0566975301827910e-03, 1.4220722610860992e+00 };
double dIntrinsicVals[] = {	5.8607137222498295e+02, 0., 3.1056890695602476e+02, 0.,
							5.8567560309796363e+02, 2.4912535666434175e+02, 0., 0., 1.};
double dDistortionVals[] = {-2.1835400781694411e-01, 1.7910224568641626e+00,
							3.8831737257415703e-03, -7.9254527156540370e-04, -5.6385675132989217e+00  };
double rotationVals[] = {	9.9997653425563382e-01, 6.8485245163549995e-03,
							-1.6926322742441872e-04, -6.8493271512092896e-03,
							9.9996243174913368e-01, -5.3124201579767246e-03,
							1.3287462880795254e-04, 5.3134548373026313e-03,
							9.9998587467125000e-01 };
double translationVals[] = 	{2.3951469877240551e-02, -3.6296501168410337e-03,  -5.6312216296436068e-03 };
*/

freenect_raw_tilt_state *tiltState = 0;
double dx, dy, dz;
FILE *plyOutput = 0;
/* OpenGL stuffs */
float angleX = 0.0, angleY=0.0;
float moveX =0, moveY=0, moveZ=-5;
int firstClick = 0, originX=0, originY=0;


int width = 800;
int height = 600;
int buttonStatus;

unsigned char command=0;
int tiltAngle = 0;
double kinectAngleX, kinectAngleY, kinectAngleZ;
double kinectRotMat[9];
/******Point Cloud Matching variables ******/

int uRGB, vRGB;
int invalid=0;
double d;
int ptCount = 0;

unsigned char *rgbImg;

/******************************************/

/****** Image variables **************/
//IplImage *undistortedRGBFrame;
//IplImage *undistortedDepthFrame;
//IplImage *newColorFrame;
/*************************************/


void matMul(double *mat1, int r1, int c1, double *mat2, int r2, int c2, double *result) {

	int i, j, k;

	for (i = 0; i < r1; i++) {
		for (j = 0; j < c2; j++) {
			result[(i*c2)+j] = 0;
			for (k = 0; k < r2; k++) {
				result[(i*c2)+j] +=  mat1[(i*c1)+k] * mat2[(k*c2)+j];
			}
		}
	}
}


void getKinectAngle(void) {
	int i;
	double totalX=0, totalY=0, totalZ=0;
	double tempPoint1[3];
	double tempPoint2[3];

	for (i=0; i<10; i++) {
		// Get the raw accelerometer values and tilt data
		if (freenect_sync_get_tilt_state(&tiltState, 0)) exit(1);
		// Get the processed accelerometer values (calibrated to gravity)
		freenect_get_mks_accel(tiltState, &dx, &dy, &dz);
		totalX+=dx; totalY+=dy; totalZ+=dz;
	}

	dx=(totalX/10)/98;
	dy=(totalY/10)/98;
	dz=(totalZ/10)/98;

	//dy-=0.1;
	//printf("length: %lf\n", sqrt(dx*dx+dy*dy+dz*dz));

	kinectAngleX = 0;//atan(dx/(sqrt(dy*dy+dz*dz)));
	kinectAngleZ = 0;//atan(dy/(sqrt(dx*dx+dz*dz)))-3.14159265/2;
	kinectAngleY = 0;//atan(dz/(sqrt(dx*dx+dz*dz)));

	printf("pitch: %lf ", kinectAngleX*(180.0/3.14159265));
	printf("yaw: %lf ", kinectAngleY*(180.0/3.14159265));
	printf("roll: %lf \n", kinectAngleZ*(180.0/3.14159265));

	kinectRotMat[0] = cos(kinectAngleZ)*cos(kinectAngleY);
	kinectRotMat[1] = cos(kinectAngleZ)*sin(kinectAngleY)*sin(kinectAngleX) - sin(kinectAngleZ)*cos(kinectAngleX);
	kinectRotMat[2] = cos(kinectAngleZ)*sin(kinectAngleY)*cos(kinectAngleX) + sin(kinectAngleZ)*sin(kinectAngleX);
	kinectRotMat[3] = sin(kinectAngleZ)*cos(kinectAngleY);
	kinectRotMat[4] = sin(kinectAngleZ)*sin(kinectAngleY)*sin(kinectAngleX) + cos(kinectAngleZ)*cos(kinectAngleX);
	kinectRotMat[5] = sin(kinectAngleZ)*sin(kinectAngleY)*cos(kinectAngleX) - cos(kinectAngleZ)*sin(kinectAngleX);
	kinectRotMat[6] = -sin(kinectAngleY);
	kinectRotMat[7] = cos(kinectAngleY)*sin(kinectAngleX);
	kinectRotMat[8] = cos(kinectAngleY)*cos(kinectAngleX);

	tempPoint1[0]=dx;
	tempPoint1[1]=dy;
	tempPoint1[2]=dz;

	matMul(kinectRotMat,3,3,tempPoint1,3,1,tempPoint2);

	dx=tempPoint2[0];
	dy=tempPoint2[1];
	dz=tempPoint2[2];
	//printf("accel[%lf,%lf,%lf]\n", dx,dy,dz);

}

int cleanPointCloud(double *rgbToWorld) {

	int u, v, i, j, k, count=0, pointDel=0;
	double x, y, z, x2, y2, z2, distance;

	for (u=10; u < 630; u++) {
		for (v=10; v < 470; v++) {
			i=v*640+u;
			//printf("cleanPointCloud i; %d\n", i);
			x = rgbToWorld[i*3];
			y = rgbToWorld[i*3+1];
			z = rgbToWorld[i*3+2];

			for (j=-5; j<5; j++) {
				for(k=-5; k<5; k++) {
					x2 = rgbToWorld[((v+j)*640+u+k)*3];
					y2 = rgbToWorld[((v+j)*640+u+k)*3+1];
					z2 = rgbToWorld[((v+j)*640+u+k)*3+2];
					distance = sqrt((x -x2)*(x -x2)+(y -y2)*(y -y2)+(z -z2)*(z -z2));
					if(distance > 0.03)
						count++;
				}
			}

			if (count > 90) {
				rgbToWorld[i*3]=0;
				rgbToWorld[i*3+1]=0;
				rgbToWorld[i*3+2]=0;
				count = 0;
				pointDel++;

			}
		}
	}

	return pointDel;
}

void mapDepthRGB (IplImage *rawDepth, IplImage *rawRGB, IplImage *undistortedDepth, IplImage *undistortedRGB, double *rgbToWorld, int *worldToRGB) {
	int u, v, uRGB, vRGB, i;
	double x, y, z, x2, y2, z2;

	IplImage *newColorFrame = cvCreateImage(cvSize(640, 480), 8, 3);

	cvCvtColor(rawRGB, newColorFrame, CV_RGB2BGR);
	cvUndistort2(newColorFrame, undistortedRGB, &rgbIntrinsic, &rgbDistortion);
	cvUndistort2(rawDepth, undistortedDepth, &dIntrinsic, &dDistortion);

	for(i=0; i < KINECT_RES; i++) {
		worldToRGB[i*2]=0;
		worldToRGB[i*2+1]=0;
		rgbToWorld[i*3]=0;
		rgbToWorld[i*3+1]=0;
		rgbToWorld[i*3+2]=0;
	}

	for (v=0; v < 480; v++) {
		for (u=0; u < 640; u++) {

			i=v*640+u;

			if (u >=10 && u < 630 && v >= 10 && v < 470) {

				z = depthTable[((short*)undistortedDepth->imageData)[i]];

				if ((z < 0.5) || (z >10)) {
					x2=0.0; y2=0.0; z2=0;
					uRGB=0, vRGB=0;
				}
				else {
					x = (u - dIntrinsicVals[2])*z / dIntrinsicVals[0];
					y =	(v - dIntrinsicVals[5])*z / dIntrinsicVals[4];

					x2 = (x*rotationVals[0] + y*rotationVals[1] + z*rotationVals[2]) + transFudgedVals[0];
					y2 = (x*rotationVals[3] + y*rotationVals[4] + z*rotationVals[5]) + transFudgedVals[1];
					z2 = (x*rotationVals[6] + y*rotationVals[7] + z*rotationVals[8]) + transFudgedVals[2];

					uRGB = (int)((x2*rgbIntrinsicVals[0]/z2) + rgbIntrinsicVals[2]);
					vRGB = (int)((y2*rgbIntrinsicVals[4]/z2) + rgbIntrinsicVals[5]);

					if ((uRGB < 0) || (vRGB < 0) || (uRGB >= 640) || (vRGB >= 480)) {
						x2=0; y2=0; z2=0;
						uRGB=0, vRGB=0;
					}
				}

				worldToRGB[i*2]=uRGB;
				worldToRGB[i*2+1]=vRGB;
				rgbToWorld[(vRGB*640+uRGB)*3]=x2;
				rgbToWorld[(vRGB*640+uRGB)*3+1]=y2;
				rgbToWorld[(vRGB*640+uRGB)*3+2]=z2;
			}
		}
	}

	cvReleaseImage(&newColorFrame);
}



float rawDepthToMeters(int rawDepth)
{
	if (rawDepth < 2047)
		return (float) (0.1236 * tan(rawDepth/2842.5 + 1.1863));

	return 0;
}

float* populateDepthTable()
{
	float* depthTable;
	int i;
	depthTable = (float*) malloc(sizeof(float)*2048);

	for (i = 0; i < 2047; i++) {
		depthTable[i] = rawDepthToMeters(i);
	}
	return depthTable;
}



/*
 * Detect mouse button pressed
 */
void mouse(int button, int state, int x, int y) {
	buttonStatus = button; //

	if (state) {
		firstClick = 0;
	}
	else  {
		if(firstClick ==0) {
			originX=x; originY=y;
			firstClick = 1;
			printf("x=%d y=%d\n",x,y);
		}

	}
}

/*
 * Detect mouse motion to rotate view
 */
void processMouseActiveMotion(int x, int y) {
	if(buttonStatus == GLUT_DOWN) {
		// Only activated with left mouse click
		if(buttonStatus == GLUT_LEFT_BUTTON) {

			//Limits mouse movement detection within the window.
			if (x < 0) x=0;
			else if (x > width) x = width;
			if (y < 0) y=0;
			else if (y > height) y = height;

			angleX = (originX-x)*0.4;//180.0 * ((float) x)/width;
			angleY = (originY-y)*0.4;//180.0 * ((float) y)/height;

		}

	}

}

void mouseWheel(int button, int dir, int x, int y)
{

    if (dir > 0)
    {
        printf("Wheel up\n");
    }
    else
    {
        printf("Wheel down\n");
		// Zoom out
    }


}

/*
** Function called when the window is created or resized
*/
void ReshapeFunc(int width, int height){


	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	//gluPerspective(20, width / (float) height, 1, 100);

    GLfloat zNear = 1.0f;
    GLfloat zFar = 100.0f;
    GLfloat aspect = float(width)/float(height);
    GLfloat fH = tan( float(40 / 360.0f * 3.14159f) ) * zNear;
    GLfloat fW = fH * aspect;
    glFrustum( -fW, fW, -fH, fH, zNear, zFar );

	glViewport(0, 0, width, height);

	glMatrixMode(GL_MODELVIEW);

	glutPostRedisplay();
}

/*
** Function called to update rendering
*/
void DisplayFunc(void) {
	int r, c, i, j, k, u, v;
	double x, y, z;
	double red, green, blue;
	double tempPoint[3];
	double tempPoint2[3];
	//if (command==0) {

		/*rawRGBFrame = freenect_sync_get_rgb_cv(0);
		if (!rawRGBFrame) {
			printf("Error: Kinect not connected?\n");
			exit(1);
		}
		rawDepthFrame = freenect_sync_get_depth_cv(0);
		if (!rawDepthFrame) {
			printf("Error: Kinect not connected?\n");
			exit(1);
		}*/

		getKinectAngle();
		//Find XYZ points and match them to RGB image pixels
		mapDepthRGB(rawDepthFrame, rawRGBFrame, undistortedDepthFrame, undistortedRGBFrame, rgbToWorld, worldToRGB);

	//}
	/* Clear the buffer, clear the matrix */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	//glPushMatrix();

	glTranslatef(moveX, moveY, moveZ);
	glRotatef(angleX, 0, 1, 0);
	glRotatef(angleY, 1, 0, 0);

	//glColor3f(1, 0, 0);
	//renderSphere_convenient(0,0,0,0.03,100);

	/* Begins rendering the points */
	glBegin(GL_POINTS);


	rgbImg = (unsigned char *)undistortedRGBFrame->imageData;
	for (c=10; c < 630; c++) {
		for (r=10; r < 470; r++) {

			i=(r*640+c);



			u = worldToRGB[i*2];
			v = worldToRGB[i*2+1];

			tempPoint[0] = rgbToWorld[(v*640+u)*3];
			tempPoint[1] = rgbToWorld[(v*640+u)*3+1];
			tempPoint[2] = rgbToWorld[(v*640+u)*3+2];

			matMul(kinectRotMat,3,3,tempPoint,3,1,tempPoint2);

			x = tempPoint2[0];
			y = -tempPoint2[1];
			z = -tempPoint2[2];

			red = (double)(rgbImg[(v*640 + u)*3 + 2])/255.0;
			green = (double)(rgbImg[(v*640 + u)*3 + 1])/255.0;
			blue = (double)(rgbImg[(v*640 + u)*3])/255.0;

			glColor3f(red,green,blue);
			glVertex3f(x, y, z);
		}
	}
	//printf("totalPoints: %d\n", totalPoints);

	glEnd();

	/*glBegin(GL_LINES);

	glColor3f(1,1,0);
	glVertex3f(0, 0, 0.0);
	glVertex3f(dx, dy, dz);

	glColor3f(0,1,0);			//green x
	glVertex3f(0, 0, 0);
	glVertex3f(0.1, 0, 0);

	glColor3f(0,0,1);			//blue y
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.1, 0);

	glColor3f(1,0,0);			//red z
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -2.1);
	glEnd( );*/

	/* End */
	glFlush();
	glutSwapBuffers();

	/* Update again and again */
	glutPostRedisplay();
}

/*
** Function called when a key is hit
*/
void KeyboardFunc(unsigned char key, int x, int y) {
	//Map keys to move about the environment in the X, Y, Z direction
	switch (key) {
		case 'a' : moveX+=0.05;break;
		case 'd' : moveX-=0.05;break;
		case 'w' : moveZ+=0.5;break;
		case 's' : moveZ-=0.5;break;
		case 'e' : moveY+=0.05;break;
		case 'q' : moveY-=0.05;break;
		case '8' : if (freenect_sync_set_tilt_degs(tiltAngle+=5, 0)) exit(1);break;
		case '2' : if (freenect_sync_set_tilt_degs(tiltAngle-=5, 0)) exit(1);break;
		case '5' : if (freenect_sync_set_tilt_degs(tiltAngle=0, 0)) exit(1);break;
		case 'h' : moveX=0;moveY=0;moveZ=-5;angleX=0;angleY=0;break;
		case 'f' : command = command^1; printf("points deleted: %d\n", cleanPointCloud(rgbToWorld)); break; //freeze frame
		case 'x' :
		case 'X' :
		case 27  :exit(0);
	}





}
