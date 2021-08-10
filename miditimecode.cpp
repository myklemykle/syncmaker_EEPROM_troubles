// quick & dirty MIDI Timecode implementation.

#include <elapsedMillis.h>

class MidiTimecodeGenerator { 
public: 
	elapsedMillis myClock;
	byte currentFrame;
	const int myClockTick = 40; // 25 frames/second is the PAL SMPTE/MTC standard, more int-friendly than the other options.
	byte nowPos[4] = {0,0,0,0}; // hhmmssff
	byte framePos[4] = {0,0,0,0}; // hhmmssff
	int nextBits = 0;

	void setup() {
		myClock = 0;
		currentFrame = 0;
	}

	// send the next frame in the running "time"
	void sendQuarterFrame(){
		if (currentFrame == 0){
			// all 8 pieces of a 2-frame message describe the moment of the start of the first frame
			// so copy that moment for the next 7 4/frames to use
			framePos[0] = nowPos[0];
			framePos[1] = nowPos[1];
			framePos[2] = nowPos[2];
			framePos[3] = nowPos[3];
		}

		// which piece of the timestamp?
		byte timepiece = framePos[3 - (currentFrame >> 1)]; // hh, mm, ss or ff

		byte fragment;
		if (currentFrame & 1) { // if odd send MSBs
			fragment = (timepiece >> 4) ;
		} else { // if even send LSBs
			fragment = (timepiece & 0b00001111) ;
		}
		/* Dbg_print(currentFrame >> 1); */
		/* Dbg_print("="); */
		/* Dbg_print(timepiece); */
		/* Dbg_print("; "); */
		/* Dbg_print(currentFrame); */
		/* Dbg_print("/"); */
		/* Dbg_println(fragment); */

		usbMIDI.sendTimeCodeQuarterFrame(currentFrame, fragment);
		// advance frame.
		currentFrame = (currentFrame + 1) % 8;
	}

	// send absolute time position (for restarting, etc.)
	void sendStamp() {
		byte buf[10] = {0xF0, 0x7F, 0x7F, 0x01, 0x01, 
			nowPos[0],nowPos[1],nowPos[2],nowPos[3], 
			0xF7};
		usbMIDI.sendSysEx(10, buf, true); // 'true' == buffer already starts with 0xF0 & ends with 0xF7)
		currentFrame = 0;
	}

	void setPosition(int hh, int mm, int ss, int ff){
		nowPos[0] = (hh & 0b00011111) | (0b00100000);  // 001 in leftmost bits indicates 25FPS frame rate
		nowPos[1] = mm;
		nowPos[2] = ss;
		nowPos[3] = ff;
	}

	void incFrame() {
		if (++nowPos[3] >= 25) {
			nowPos[3] = 0;
			if (++nowPos[2] >= 60) {
				nowPos[2] = 0;
				if (++nowPos[1] >= 60) {
					nowPos[1] = 0;
					if ((++nowPos[0] & 0b00011111) >= 24) {
						nowPos[0] = 0b00100000;								// 001 in leftmost bits indicates 25FPS frame rate
					}
				}
			}
		}
		/* Dbg_print(nowPos[0]); */
		/* Dbg_print(":"); */
		/* Dbg_print(nowPos[1]); */
		/* Dbg_print(":"); */
		/* Dbg_print(nowPos[2]); */
		/* Dbg_print(":"); */
		/* Dbg_println(nowPos[3]); */
	}

	void mtcGoto(int hh, int mm, int ss, int ff){
		setPosition(hh,mm,ss,ff);
		return sendStamp();
	}

	// return to the dawn of time
	void rewind(){
		return mtcGoto(0,0,0,0);
	}

		// send MTC frame if ready:
	void frameCheck() { 
		if (myClock > myClockTick) { // on interval 
			myClock -= myClockTick; 
			sendQuarterFrame(); 
		}
	}

};
