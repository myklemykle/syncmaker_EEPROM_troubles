#ifndef LOOPPROFILER_H
#define LOOPPROFILER_H


// With this object, we measure the time between checkpoints on every loop
// and then average them over some number of samples.
//
#define PROFILE_AVGOVER 100 		// max number of samples to average over
//
// This is all statically allocated, so these two effect our mem footprint:
#define PROFILE_POINTNAMELEN 10 // how long can the label of a checkpoint be
#define PROFILE_CPS 20  				// max number of checkpoints

typedef struct {
	char name[PROFILE_POINTNAMELEN];
	unsigned long sampleStart;
	unsigned int sampleCount;
	unsigned long sampleLen;
	double averageLen;
} LoopProfileCheckpoint;

class LoopProfiler {
	private:
		LoopProfileCheckpoint checkpoints[PROFILE_CPS];
		int pointCursor, maxPoints;
		unsigned long loopStart_us;
		int findPointByName(char *pName);

	public:
		void init();
		void startLoop();
		void markStart(char *pName);
		void markEnd(char *pName);
		void printRaw();
		void printAverage();
		void printDelta();
};

#ifdef PROFILE

// single global instance of the object:
static LoopProfiler __profile;

// handy macros:
#define PROFILE_SETUP()   		__profile.init()
#define PROFILE_START_LOOP()  __profile.startLoop()
#define PROFILE_MARK_START(label)  __profile.markStart(label)
#define PROFILE_MARK_END(label)  __profile.markEnd(label)
#define PROFILE_PRINT_RAW()  __profile.printRaw()
#define PROFILE_PRINT_AVG()  __profile.printAverage()
#define PROFILE_PRINT_DELTA()  __profile.printDelta()

#else // !PROFILE 

// empty macros:
#define PROFILE_SETUP()   
#define PROFILE_START_LOOP()
#define PROFILE_MARK_START(label)
#define PROFILE_MARK_END(label)
#define PROFILE_PRINT_RAW()  
#define PROFILE_PRINT_AVG() 
#define PROFILE_PRINT_DELTA()

#endif //PROFILE

#endif //LOOPPROFILER_H
