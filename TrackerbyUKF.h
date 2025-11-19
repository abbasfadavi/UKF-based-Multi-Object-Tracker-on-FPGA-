#pragma once

#include <cmath>
#include <cstring>
#include "ap_int.h"

#include <iostream>
#include <fstream>
#include "hls_stream.h"
using namespace std;

#define HEIGHT 360
#define WIDTH 640
#define NumFrames 531
#define MAX_CORNERS 50
#define MAX_TRACKS 100
#define STATE_DIM 7
#define EPS 1e-8f
#define WINDOW_SIZE 11
#define MAX_WIN 20

void tracker_step
(
		unsigned char curFrame[HEIGHT][WIDTH],
		int frameCount,

		bool active_out[MAX_TRACKS],
		float state_out[MAX_TRACKS][2]
);
void correlation
(
		unsigned char prevImg[HEIGHT][WIDTH],
		unsigned char currImg[HEIGHT][WIDTH],
		float state[MAX_TRACKS][STATE_DIM],
		bool active[MAX_TRACKS],
		float trackedPoints[MAX_TRACKS][2],
		bool isFound[MAX_TRACKS]
);
void detector
(
		unsigned char x[HEIGHT][WIDTH],
		unsigned char xx[HEIGHT][WIDTH],
		int corners[MAX_CORNERS][2],
		int &num_corners
);









