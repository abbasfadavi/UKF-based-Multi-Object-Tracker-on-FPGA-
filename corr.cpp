#include "TrackerbyUKF.h"
// ========================================
void correlation
(
		unsigned char prevImg[HEIGHT][WIDTH],
		unsigned char currImg[HEIGHT][WIDTH],
		float state[MAX_TRACKS][STATE_DIM],
		bool active[MAX_TRACKS],
		float trackedPoints[MAX_TRACKS][2],
		bool isFound[MAX_TRACKS]
)
{
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=prevImg
#pragma HLS ARRAY_PARTITION dim=2 factor=32 type=block variable=currImg


	float corr_mat[21][21];


	const int MAX_SEARCH = 2 * 20 + 1;
	int win = 10;
	int wins = 7;
	int n = (2*win + 1) * (2*win + 1);
	//
	loop_corr1 : for(int t = 0; t < MAX_TRACKS; t++)
	{
		isFound[t] = 0;
		int px = (int)state[t][0];
		int py = (int)state[t][1];

		if (active[t] && px - win >= 0 && px + win < WIDTH && py - win >= 0 && py + win < HEIGHT)
		{

			float sx = 0;
			float sxx = 0;
			float val[32]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
			float sx1[32];
			float sxx1[32];
			float sx2[16];
			float sxx2[16];
			float sx3[8];
			float sxx3[8];
			float sx4[4];
			float sxx4[4];
			float sx5[2];
			float sxx5[2];

			loop_corr2 : for(int i = -win; i <= win; i++)
			{
#pragma HLS PIPELINE ii = 8
				loop_corr31 : for(int j = -win; j <= win; j++)
				{
#pragma HLS UNROLL factor=21
					val[j+win] = float(prevImg[py+i][px+j]);
				}
				loop_corr32 : for(int j = 0; j <= 31; j++)
				{
#pragma HLS UNROLL factor=32
					sx1[j] = val[j];
					sxx1[j] = val[j] * val[j];
				}
				loop_corr33 : for(int j = 0; j <= 15; j++)
				{
#pragma HLS UNROLL factor=16
					sx2[j]= sx1[2*j]+sx1[2*j+1];
					sxx2[j]= sxx1[2*j]+sxx1[2*j+1];
				}
				loop_corr34 : for(int j = 0; j <= 7; j++)
				{
#pragma HLS UNROLL factor=8
					sx3[j]= sx2[2*j]+sx2[2*j+1];
					sxx3[j]= sxx2[2*j]+sxx2[2*j+1];
				}
				loop_corr35 : for(int j = 0; j <= 3; j++)
				{
#pragma HLS UNROLL factor=4
					sx4[j]= sx3[2*j]+sx3[2*j+1];
					sxx4[j]= sxx3[2*j]+sxx3[2*j+1];
				}
				loop_corr36 : for(int j = 0; j <= 1; j++)
				{
#pragma HLS UNROLL factor=2
					sx5[j]= sx4[2*j]+sx4[2*j+1];
					sxx5[j]= sxx4[2*j]+sxx4[2*j+1];
				}
				loop_corr37 : for(int j = 0; j <= 1; j++)
				{
#pragma HLS UNROLL factor=2
					sx += sx5[j];
					sxx += sxx5[j];
				}
			}

			float max_corr = 0;
			int best_i = 0, best_j = 0;
			//

			for(int ii = 0; ii <= 20; ii++)
				for(int jj = 0; jj <= 20; jj++)
					corr_mat[ii][jj] = 0.0f;






			loop_corr4 : for(int di = -wins; di <= wins; di++)
			{
				bool flag = 1;
				loop_corr5 : for(int dj = -wins; dj <= wins; dj++)
				{
					float sy = 0, syy = 0, sxy = 0;

					if(flag == 1 && py+di-win >= 0 && py+di+win < HEIGHT && px+dj-win >= 0 && px+dj+win < WIDTH)
					{
						float x_val[32]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
						float y_val[32]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
						float sy1[32];
						float syy1[32];
						float sxy1[32];
						float sy2[16];
						float syy2[16];
						float sxy2[16];
						float sy3[8];
						float syy3[8];
						float sxy3[8];
						float sy4[4];
						float syy4[4];
						float sxy4[4];
						float sy5[2];
						float syy5[2];
						float sxy5[2];

						loop_corr6 : for(int i = - win; i  <= win; i++)
						{
#pragma HLS PIPELINE ii = 11
							loop_corr71 : for(int j = -win; j <= win; j++)
							{
#pragma HLS UNROLL factor=21
								x_val[j+win] = prevImg[py+i][px+j];
								y_val[j+win] = currImg[py+i+di][px+j+dj];
							}
							loop_corr72 : for(int j = 0; j <= 31; j++)
							{
#pragma HLS UNROLL factor=32
								sy1[j] = y_val[j];
								syy1[j] = y_val[j] * y_val[j];
								sxy1[j] = x_val[j] * y_val[j];
							}
							loop_corr73 : for(int j = 0; j <= 15; j++)
							{
#pragma HLS UNROLL factor=16
								sy2[j]= sy1[2*j]+sy1[2*j+1];
								syy2[j]= syy1[2*j]+syy1[2*j+1];
								sxy2[j]= sxy1[2*j]+sxy1[2*j+1];
							}
							loop_corr74 : for(int j = 0; j <= 7; j++)
							{
#pragma HLS UNROLL factor=8
								sy3[j]= sy2[2*j]+sy2[2*j+1];
								syy3[j]= syy2[2*j]+syy2[2*j+1];
								sxy3[j]= sxy2[2*j]+sxy2[2*j+1];
							}
							loop_corr75 : for(int j = 0; j <= 3; j++)
							{
#pragma HLS UNROLL factor=4
								sy4[j]= sy3[2*j]+sy3[2*j+1];
								syy4[j]= syy3[2*j]+syy3[2*j+1];
								sxy4[j]= sxy3[2*j]+sxy3[2*j+1];
							}
							loop_corr76 : for(int j = 0; j <= 1; j++)
							{
#pragma HLS UNROLL factor=2
								sy5[j]= sy4[2*j]+sy4[2*j+1];
								syy5[j]= syy4[2*j]+syy4[2*j+1];
								sxy5[j]= sxy4[2*j]+sxy4[2*j+1];
							}
							loop_corr77 : for(int j = 0; j <= 1; j++)
							{
#pragma HLS UNROLL factor=2
								sy  += sy5[j];
								syy += syy5[j];
								sxy += sxy5[j];
							}
						}

					// NCC
					float k1 = n * sxy - sx * sy;
					float k2 = n * sxx - sx * sx;
					float k3 = n * syy - sy * sy;
					float denom = sqrtf(k2 * k3) + EPS;
					float corr = abs(k1) / denom;

					if (corr > max_corr)
					{
						max_corr = corr;
						best_i = di;
						best_j = dj;
					}


					if(corr < 0.5f)flag = 0;

					corr_mat[di+win][dj+win] = std::round(1000*corr);



				}

			}

			}
			//
//			if(t==0)
//			{
//				for(int ii = 0; ii <= 20; ii++)
//				{
//					for(int jj = 0; jj <= 20; jj++)
//					{
//						cout << " , " << corr_mat[ii][jj];
//
//					}
//					cout << endl;
//				}
//			}
			//

			//
			int new_x = px + best_j;
			int new_y = py + best_i;
			new_x = (new_x < 1) ? 1 : (new_x >= WIDTH) ? WIDTH-1 : new_x;
			new_y = (new_y < 1) ? 1 : (new_y >= HEIGHT) ? HEIGHT-1 : new_y;

			max_corr = std::round(1000*max_corr);
			//
			if (max_corr > 900.0f)
			{
				isFound[t] = true;
				trackedPoints[t][0] = new_x;
				trackedPoints[t][1] = new_y;
			}
			else
			{
				isFound[t] = false;
			}
		}
	}
}
