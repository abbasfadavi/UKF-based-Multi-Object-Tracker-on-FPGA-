#include "TrackerbyUKF.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// ========================================
int main()
{
	bool active_out[MAX_TRACKS];
	float state_out[MAX_TRACKS][2];
	//
	cout << "-----------------------------------------------" << endl;
	FILE *fp_result = fopen("result_matrix.bin", "rb");
	FILE *fp_frame = fopen("frame_matrix.bin", "rb");
	if (fp_result == NULL)printf("result_matrix.bin!\n");
	if (fp_frame == NULL)printf("frame_matrix.bin!\n");
	//
	for (int frameCount = 1; frameCount <= NumFrames ; frameCount++)
	{
		//cout << "frameCount = " << frameCount << endl;
		unsigned char curFrame[HEIGHT][WIDTH];
		fread(&curFrame, sizeof(unsigned char),HEIGHT*WIDTH, fp_frame);
		tracker_step(curFrame,frameCount,active_out,state_out);
		//
		float resultFrame[MAX_TRACKS][3];
		fread(&resultFrame, sizeof(float),3*100, fp_result);
				int cnt = 0;
				for (int t = 0; t < MAX_TRACKS; t++)
				{
					float a0 = active_out[t];
					float a1 = state_out[t][0];
					float a2 = state_out[t][1];
					float b0 = resultFrame[t][0];
					float b1 = resultFrame[t][1];
					float b2 = resultFrame[t][2];
					if(a0 != b0)
					{
						//cout << "frameCount = " << frameCount << endl;
						cout << " we have error 1! " << endl;
						cout << "t = " << t << endl;
					}
					else if(a0 == 1 && b0 == 1)
					{
						if (a1+1 != b1)cout << " we have error 2! " << endl;
						if (a2+1 != b2)cout << " we have error 3! " << endl;
					}
				}
	}
	fclose(fp_frame);
	fclose(fp_result);
	cout << "-----------------------------------------------" << endl;
	return 0;
}
