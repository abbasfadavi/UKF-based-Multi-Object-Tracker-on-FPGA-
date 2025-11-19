#include "TrackerbyUKF.h"
// ========================================
static inline void mat_zero(float A[STATE_DIM][STATE_DIM])
{
#pragma HLS INLINE
	for (int i = 0; i < STATE_DIM; i++)
		for (int j = 0; j < STATE_DIM; j++)
#pragma HLS PIPELINE II=1
			A[i][j] = 0.0f;
}

static inline void mat_eye(float A[STATE_DIM][STATE_DIM], float val)
{
#pragma HLS INLINE
	mat_zero(A);
	for (int i = 0; i < STATE_DIM; i++)
#pragma HLS UNROLL
		A[i][i] = val;
}

static inline void mat_copy
(
		float dst[STATE_DIM][STATE_DIM],
		float src[STATE_DIM][STATE_DIM]
)
{
#pragma HLS INLINE
	for (int i = 0; i < STATE_DIM; i++)
		for (int j = 0; j < STATE_DIM; j++)
		{
#pragma HLS PIPELINE II=1
			dst[i][j] = src[i][j];
		}
}
// ========================================
static inline void motionModel(float x_in[STATE_DIM], float dt, float x_out[STATE_DIM])
{
#pragma HLS INLINE

	float dt2 = 0.5f * dt * dt;

	x_out[0] = x_in[0] + dt * x_in[2] + dt2 * x_in[4];
	x_out[1] = x_in[1] + dt * x_in[3] + dt2 * x_in[5];
	x_out[2] = x_in[2] + dt * x_in[4];
	x_out[3] = x_in[3] + dt * x_in[5];
	x_out[4] = x_in[4];
	x_out[5] = x_in[5];
	x_out[6] = x_in[6]; // scale factor
}

// ========================================
static void chol_simple
(
		float A[STATE_DIM][STATE_DIM],
		float L[STATE_DIM][STATE_DIM]
)
{


	for (int i = 0; i < STATE_DIM; i++)
	{
		for (int j = 0; j < STATE_DIM; j++)
			L[i][j] =  0.0f;
	}

	for (int i = 0; i < STATE_DIM; i++)
	{

		for (int j = 0; j <= i; j++)
		{

			float s = 0.0f;
			for (int k = 0; k < j; k++)
			{
#pragma HLS PIPELINE II=1
				s += L[i][k] * L[j][k];
			}
			if (i == j)
			{
				float v = A[i][i] - s;
				L[i][j] = (v > 0.0f) ? sqrtf(v) : EPS;
			}
			else
			{
				L[i][j] = (A[i][j] - s) / (L[j][j] + EPS);
			}
		}
	}
}
// ========================================
static void track_predict
(
		float state[STATE_DIM],
		float p_flat[STATE_DIM * STATE_DIM]
)
{
#pragma HLS INLINE off

	const int n = STATE_DIM;
	const float w = 1.0f / (2.0f * n);  // 1/14
	float P[STATE_DIM][STATE_DIM];
	float L[STATE_DIM][STATE_DIM];
	//
	for (int i = 0; i < n; i++)
	{
//#pragma HLS UNROLL
#pragma HLS PIPELINE II=1
		for (int j = 0; j < n; j++)
		{
			P[i][j] = p_flat[i*n + j] + ((i == j) ? 1e-6f : 0.0f);
		}
	}
	//
	chol_simple(P, L);
	//
	float X[14][STATE_DIM];
	for (int i = 0; i < n; i++)
	{
#pragma HLS PIPELINE II=1
		for (int j = 0; j < n; j++)
		{
#pragma HLS UNROLL
			X[i][j]     = state[j] + L[j][i];
			X[i+7][j]   = state[j] - L[j][i];
		}
	}

	//
	const float A[STATE_DIM][STATE_DIM] = {
			{1.0f, 0.0f, 0.033f, 0.0f, 0.0005445f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.033f, 0.0f, 0.0005445f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f, 0.033f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.033f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.033f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.033f}
	};
	float x_pred1[14][STATE_DIM];
	for (int s = 0; s < 14; s++)
	{
		for (int i = 0; i < n; i++)
		{
			float sum = 0.0f;
			for (int j = 0; j < n; j++)
			{
#pragma HLS PIPELINE II=1
				sum += A[i][j] * X[s][j];
			}
			x_pred1[s][i] = sum;
		}
	}
	//
	float x_pred2[STATE_DIM] = {0};
	for (int i = 0; i < n; i++)
	{
		float sum = 0.0f;
		for (int s = 0; s < 14; s++)
		{
#pragma HLS PIPELINE II=1
			sum += x_pred1[s][i];
		}
		x_pred2[i] = sum * w;
	}

	//
	float Q_diag[STATE_DIM] = {1e-2f, 1e-2f, 1e-1f, 1e-1f, 1e-3f, 1e-3f, 1e-4f};
	float P_pred[STATE_DIM][STATE_DIM] = {0};

	for (int s = 0; s < 14; s++)
	{
		float diff[STATE_DIM];
		for (int i = 0; i < n; i++)
		{
			diff[i] = x_pred1[s][i] - x_pred2[i];
		}
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
#pragma HLS PIPELINE II=1
				P_pred[i][j] += w * diff[i] * diff[j];
			}
		}
	}
	for (int i = 0; i < n; i++)
	{
#pragma HLS PIPELINE II=1
		P_pred[i][i] += Q_diag[i];
	}

	//
	for (int i = 0; i < n; i++)
	{
		state[i] = std::round(x_pred2[i]);
	}
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
#pragma HLS PIPELINE II=1
			p_flat[i*n + j] = P_pred[i][j];
		}
	}
}
// ========================================
static void edit_track
(
		bool active[MAX_TRACKS],
		float state[MAX_TRACKS][STATE_DIM],
		float ws[MAX_TRACKS],
		float age[MAX_TRACKS],
		float trackedPoints[MAX_TRACKS][2],
		bool isFound[MAX_TRACKS]
)
{
#pragma HLS INLINE
	for (int t = 0; t < MAX_TRACKS; t++)
	{
#pragma HLS PIPELINE II = 2
		if (isFound[t])
		{
			active[t] = true;
			state[t][0] = trackedPoints[t][0];
			state[t][1] = trackedPoints[t][1];
			if (state[t][6] < 0.5f) state[t][6] = 0.5f;
			if (state[t][6] > 5.0f) state[t][6] = 5.0f;
			ws[t] += 1.0f;
			age[t] += 1.0f;
		}
		else
		{
			active[t] = false;
		}
	}
}
// ========================================
static void merge_tracks
(
		float state[MAX_TRACKS][STATE_DIM],
		float age[MAX_TRACKS],
		bool active[MAX_TRACKS]
)
{
#pragma HLS INLINE off
	for (int i = 0; i < MAX_TRACKS; i++)
	{
		if (!active[i]) continue;
		for (int j = i+1; j < MAX_TRACKS; j++)
		{
#pragma HLS PIPELINE II = 11
			if (!active[j]) continue;
			float dx = state[i][0] - state[j][0];
			float dy = state[i][1] - state[j][1];
			float diff_traks = sqrtf(dx*dx + dy*dy);
			if  (diff_traks < 40.0f)
			{
				if (age[i] >= age[j])
				{
					active[j] = false;
				}
				else
				{
					active[i] = false;
				}
			}
		}
	}
}
// ========================================
static void cut_tracks
(
		float history[MAX_TRACKS][2],
		float state[MAX_TRACKS][STATE_DIM],
		float age[MAX_TRACKS],
		bool active[MAX_TRACKS]
) {
#pragma HLS INLINE
	for (int i = 0; i < MAX_TRACKS; i++)
	{
		if (!active[i]) continue;
		float dx = fabsf(history[i][0] - state[i][0]);
		float dy = fabsf(history[i][1] - state[i][1]);
		if ((dx + dy)/2 < age[i])
		{
			active[i] = false;
		}
		else if(age[i]>100)active[i] = false;
	}
}
// ========================================
void add_track
(
		bool active[MAX_TRACKS],
		float age[MAX_TRACKS],
		float state[MAX_TRACKS][STATE_DIM],
		float history[MAX_TRACKS][2],
		float ws[MAX_TRACKS],
		float p[MAX_TRACKS][49],
		int points[MAX_TRACKS][2],
		int num_points
)
{
#pragma HLS INLINE

	int i = 0;
	for (int t = 0; t < MAX_TRACKS; t++)
	{
#pragma HLS PIPELINE II=3
		if (!active[t] && i < num_points)
		{
			active[t] = true;
			age[t] = 1.0f;
			ws[t] = 20.0f;

			state[t][0] = points[i][0];  // x
			state[t][1] = points[i][1];  // y
			state[t][2] = 0.0f;  // vx
			state[t][3] = 0.0f;  // vy
			state[t][4] = 0.0f;  // ax
			state[t][5] = 0.0f;  // ay
			state[t][6] = 1.0f;  // scale factor

			history[t][0] = points[i][0];
			history[t][1] = points[i][1];

			// P = eye * 1e-2
			for (int k = 0; k < 49; k++) p[i][k] = 0.0f;
			p[t][ 0] = 1e-2f;  // [0,0]
			p[t][ 8] = 1e-2f;  // [1,1]
			p[t][16] = 1e-2f;  // [2,2]
			p[t][24] = 1e-2f;  // [3,3]
			p[t][32] = 1e-2f;  // [4,4]
			p[t][40] = 1e-2f;  // [5,5]
			p[t][48] = 1e-2f;  // [6,6]
			i++;
		}
	}
}
//}
// ========================================
int num_active
(
		bool active[MAX_TRACKS]
)
{
	int cnt = 0;
#pragma HLS INLINE
	for (int t = 0; t < MAX_TRACKS; t++)
	{
		if(active[t])cnt++;
	}
	return cnt;
}
// ========================================
void frame_copy
(
		unsigned char curFrame[HEIGHT][WIDTH],
		unsigned char prevFrame[HEIGHT][WIDTH]
)
{
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=curFrame
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=prevFrame
	loop_copy1 : for (int r = 0; r < HEIGHT; r++)
	{
#pragma HLS PIPELINE II=10
		loop_copy2 : for (int c = 0; c < WIDTH; c++)
		{
#pragma HLS UNROLL factor=8
			prevFrame[r][c] = curFrame[r][c];
		}
	}
}
// ========================================
void tracker_step
(
		unsigned char curFrame[HEIGHT][WIDTH],
		int frameCount,

		bool active_out[MAX_TRACKS],
		float state_out[MAX_TRACKS][2]
)
{
	static bool active[MAX_TRACKS];
	static float state[MAX_TRACKS][STATE_DIM];
	static float p[MAX_TRACKS][STATE_DIM * STATE_DIM];
	static float history[MAX_TRACKS][2];
	static float ws[MAX_TRACKS];
	static float age[MAX_TRACKS];
	static unsigned char prevFrame[HEIGHT][WIDTH];
	static float trackedPoints[MAX_TRACKS][2];
	static bool isFound[MAX_TRACKS];
	static int corners[MAX_CORNERS][2];
	int num;
	int num1 = 0,num2 = 0,num3 = 0,num4 = 0,num5 = 0;

#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=curFrame
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=prevFrame
	//
	if (frameCount > 1)
	{
		detector(curFrame,prevFrame, corners,num);num1 = num;
		add_track(active, age, state, history, ws, p, corners, num);num2 = num_active(active);
		for (int t = 0; t < MAX_TRACKS; t++)
		{
			if (active[t])
			{
				track_predict(state[t],p[t]);
			}
		}
		correlation(prevFrame, curFrame, state, active, trackedPoints, isFound);
		edit_track(active,state, ws, age, trackedPoints, isFound);num3 = num_active(active);
		merge_tracks(state,age, active);num4 = num_active(active);
		cut_tracks(history, state, age, active);num5 = num_active(active);
	}
	frame_copy(curFrame,prevFrame);
	// Output
	for (int t = 0; t < MAX_TRACKS; t++)
	{
#pragma HLS PIPELINE II=4
		active_out[t] = active[t];
		state_out[t][0] = state[t][0];
		state_out[t][1] = state[t][1];
	}
	//	cout << "num1 = " << num1 << endl;
	//	cout << "num2 = " << num2 << endl;
	//	cout << "num3 = " << num3 << endl;
	//	cout << "num4 = " << num4 << endl;
	//	cout << "num5 = " << num5 << endl;
}
