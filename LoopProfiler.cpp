#include "LoopProfiler.h"
#include "config.h"



void LoopProfiler::init(){
	// zero stuff:
	for (int i = 0; i < PROFILE_CPS; i++){
		checkpoints[i].sample = 0;
		checkpoints[i].sampleCount = 0;
		checkpoints[i].average = 0;
		strncpy(checkpoints[i].pointName, "", PROFILE_POINTNAMELEN -1);
	}
	pointCursor = 0;
}

void LoopProfiler::startLoop(){
	if (pointCursor > 0)
		markPoint("END");

	maxPoints = pointCursor;
	pointCursor = 0;
	loopStart_us = micros();
	markPoint("BEGIN");
}

void LoopProfiler::markPoint(char *pName){
	// only have room for this many ...
	if (pointCursor == PROFILE_CPS) {
		Dbg_println("loopProfiler: only PROFILE_CPS checkpoints allowed");
		return;
	}

	unsigned long now_us = micros();
	LoopProfileCheckpoint *cp = &(checkpoints[pointCursor]);

	strncpy(cp->pointName, pName, PROFILE_POINTNAMELEN -1);

	// measure gap
	cp->sample = now_us - loopStart_us;

	// calculate avg
	cp->average = 
		((cp->average * cp->sampleCount) + cp->sample) / (float)(cp->sampleCount + 1);
	if (cp->sampleCount < PROFILE_AVGOVER)
		cp->sampleCount++;

	if (pointCursor < PROFILE_CPS)
		pointCursor++;
}

// say deep things about the profiling (for this loop):
void LoopProfiler::printRaw(){
	Dbg_print("raw: ");
	for (int i=0;i<maxPoints;i++){
		Dbg_printf("%s=%d ",checkpoints[i].pointName, checkpoints[i].sample);
	}
	Dbg_println("");
}

void LoopProfiler::printAverage(){
	Dbg_print("avg: ");
	for (int i=0;i<maxPoints;i++){
		Dbg_printf("%s=%.2f ",checkpoints[i].pointName, checkpoints[i].average);
	}
	Dbg_println("");
}

void LoopProfiler::printDelta(){
	Dbg_print("deltas: ");
	Dbg_print(checkpoints[0].pointName);
	for (int i=1;i<maxPoints;i++){
		float d = checkpoints[i].average - checkpoints[i-1].average;
		Dbg_printf(":%.2f:%s",d, checkpoints[i].pointName);
	}
	Dbg_println("");
}

