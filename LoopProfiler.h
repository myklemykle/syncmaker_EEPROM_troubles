#ifndef LOOPPROFILER_H
#define LOOPPROFILER_H

#define PROFILE_CPS 20
#define PROFILE_AVGOVER 100
#define PROFILE_POINTNAMELEN 10

// The idea is that we measure the time between checkpoints on every loop
// and then average them over some number of samples.
typedef struct {
	char pointName[PROFILE_POINTNAMELEN];
	unsigned long sample;
	unsigned int sampleCount;
	double average;
} LoopProfileCheckpoint;

class LoopProfiler {
	private:
		LoopProfileCheckpoint checkpoints[PROFILE_CPS];
		int pointCursor, maxPoints;
		unsigned long loopStart_us;

	public:
		void init();
		void startLoop();
		void markPoint(char *pName);
		void printRaw();
		void printAverage();
		void printDelta();
};



#endif //LOOPPROFILER_H
