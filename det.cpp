#include "TrackerbyUKF.h"
// ========================================
void detector
(
		unsigned char x[HEIGHT][WIDTH],
		unsigned char xx[HEIGHT][WIDTH],
		int corners[MAX_CORNERS][2],
		int &num_corners
)
{
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=x
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=xx
#pragma HLS INLINE
	num_corners = 0;
	const int n = 5;
	const int radius = 2*n + 1;  // 11
	const int total_pixels = radius * radius;  // 121
	const int threshold = total_pixels - 1;     // 120
	float var[HEIGHT][WIDTH];
	bool valid;
	//
	loop_det1 : for (int i = 0; i < HEIGHT; i++)
	{
		loop_det2 : for (int j = 0; j < WIDTH; j++)
		{
#pragma HLS PIPELINE II=1
			var[i][j] = 0.0f;
		}
	}

	loop_det3 : for (int i = n; i < HEIGHT - n; i = i + 5)
	{
		loop_det4 : for (int j = n; j < WIDTH - n; j++)
		{

			valid = abs((int)x[i][j] - (int)xx[i][j]) >= 30;


			if(valid == 1)
			{
				float sum1 = 0.0f;
				float sum2 = 0.0f;
				float val[16]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
				float a1[16];
				float aa1[16];
				float a2[8];
				float aa2[8];
				float a3[4];
				float aa3[4];
				float a4[2];
				float aa4[2];

				loop_det5 : for (int di = -n; di <= n; di++)
				{
#pragma HLS PIPELINE ii = 6
					loop_det61 : for (int dj = -n; dj <= n; dj++)
					{
#pragma HLS UNROLL factor=11
						val[dj+n] = x[i+di][j+dj];
					}
					loop_det62 : for (int dj = 0; dj <= 15; dj++)
					{
#pragma HLS UNROLL factor=16
						a1[dj]= val[dj];
						aa1[dj]= val[dj]*val[dj];
					}
					loop_det63 : for (int dj = 0; dj <= 7; dj++)
					{
#pragma HLS UNROLL factor=8
						a2[dj]= a1[2*dj]+a1[2*dj+1];
						aa2[dj]= aa1[2*dj]+aa1[2*dj+1];
					}
					loop_det64 : for (int dj = 0; dj <= 3; dj++)
					{
#pragma HLS UNROLL factor=4
						a3[dj]= a2[2*dj]+a2[2*dj+1];
						aa3[dj]= aa2[2*dj]+aa2[2*dj+1];
					}
					loop_det65 : for (int dj = 0; dj <= 1; dj++)
					{
#pragma HLS UNROLL factor=2
						a4[dj]= a3[2*dj]+a3[2*dj+1];
						aa4[dj]= aa3[2*dj]+aa3[2*dj+1];
					}
					loop_det66 : for (int dj = 0; dj <= 1; dj++)
					{
#pragma HLS UNROLL factor=2
						sum1 += a4[dj];
						sum2 += aa4[dj];
					}
				}
				float v = sum2 - (sum1 * sum1) / total_pixels;
				float vv = sqrtf(v);
				vv = std::round(vv);
				var[i][j] = vv;
			}
		}
	}
	//
	loop_det7 : for (int i = n; i < HEIGHT - n; i = i + 5)
	{
		loop_det8 : for (int j = n; j < WIDTH - n; j++)
		{
#pragma HLS PIPELINE II = 11
			valid = abs((int)x[i][j] - (int)xx[i][j]) >= 30;
			if(valid)
			{
				int cnt = 0;
				loop_det9 : for (int dj = -n; dj <= n; dj++)
				{
//#pragma HLS UNROLL factor=4
					if (var[i][j] > var[i][j+dj]) cnt++;

				}

				if (cnt == 10 && num_corners < MAX_CORNERS)
				{
					corners[num_corners][0] = j;
					corners[num_corners][1] = i;
					num_corners++;
				}
			}
		}
	}
}
