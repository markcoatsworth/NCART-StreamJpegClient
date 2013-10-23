#include <stdio.h>
#include <stdlib.h>
#include "globals.h"

void unpackDepthData(unsigned char *packedDepthData, short *uDepthVals, int resDiv)
{
	int i;
	int croppedIndex = 0;
	unsigned long long temp;

	for (i = 0; i < 307200/resDiv; i+=4) {

		temp = 0;

		temp |= (unsigned long long) packedDepthData[croppedIndex+0];
		temp |= (unsigned long long) packedDepthData[croppedIndex+1] << 8;
		temp |= (unsigned long long) packedDepthData[croppedIndex+2] << 16;
		temp |= (unsigned long long) packedDepthData[croppedIndex+3] << 24;
		temp |= (unsigned long long) packedDepthData[croppedIndex+4] << 32;

        uDepthVals[i+0] = (short) ((temp & ((unsigned long long) 1023)));
        uDepthVals[i+1] = (short) ((temp & ((unsigned long long) 1023 << 10)) >> 10);
		uDepthVals[i+2] = (short) ((temp & ((unsigned long long) 1023 << 20)) >> 20);
		uDepthVals[i+3] = (short) ((temp & ((unsigned long long) 1023 << 30)) >> 30);

        if(uDepthVals[i+0]) uDepthVals[i+0] += 200;
        else uDepthVals[i+0] = 2047;
        if(uDepthVals[i+1]) uDepthVals[i+1] += 200;
        else uDepthVals[i+1] = 2047;
        if(uDepthVals[i+2]) uDepthVals[i+2] += 200;
        else uDepthVals[i+2] = 2047;
        if(uDepthVals[i+3]) uDepthVals[i+3] += 200;
        else uDepthVals[i+3] = 2047;

		croppedIndex += 5;
	}
}

IplImage *GlViewColor(short *depth)
{
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), 8, 3);
	unsigned char *depth_mid = (unsigned char *) image->imageData;
	int i;
	for (i = 0; i < 640*480; i++) {
		int lb = depth[i] % 256;
		int ub = depth[i] / 256;
		switch (ub) {
			case 0:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+2] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+0] = 0;
				break;
			case 2:
				depth_mid[3*i+2] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = 0;
				break;
			case 3:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+0] = lb;
				break;
			case 4:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+0] = 255;
				break;
			case 5:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 255-lb;
				break;
			default:
				depth_mid[3*i+2] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+0] = 0;
				break;
		}
	}
	return image;
}
