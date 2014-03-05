/*

    File: AQRecorder.mm
Abstract: n/a
 Version: 2.5

Disclaimer: IMPORTANT:  This Apple software is supplied to you by Apple
Inc. ("Apple") in consideration of your agreement to the following
terms, and your use, installation, modification or redistribution of
this Apple software constitutes acceptance of these terms.  If you do
not agree with these terms, please do not use, install, modify or
redistribute this Apple software.

In consideration of your agreement to abide by the following terms, and
subject to these terms, Apple grants you a personal, non-exclusive
license, under Apple's copyrights in this original Apple software (the
"Apple Software"), to use, reproduce, modify and redistribute the Apple
Software, with or without modifications, in source and/or binary forms;
provided that if you redistribute the Apple Software in its entirety and
without modifications, you must retain this notice and the following
text and disclaimers in all such redistributions of the Apple Software.
Neither the name, trademarks, service marks or logos of Apple Inc. may
be used to endorse or promote products derived from the Apple Software
without specific prior written permission from Apple.  Except as
expressly stated in this notice, no other rights or licenses, express or
implied, are granted by Apple herein, including but not limited to any
patent rights that may be infringed by your derivative works or by other
works in which the Apple Software may be incorporated.

The Apple Software is provided by Apple on an "AS IS" basis.  APPLE
MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND
OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.

IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2012 Apple Inc. All Rights Reserved.


*/

#include "AQRecorder.h"

// ____________________________________________________________________________________
// Determine the size, in bytes, of a buffer necessary to represent the supplied number
// of seconds of audio data.
int AQRecorder::ComputeRecordBufferSize(const AudioStreamBasicDescription *format, float seconds)
{
	int packets, frames, bytes = 0;
	try {
		frames = (int)ceil(seconds * format->mSampleRate);
		
		if (format->mBytesPerFrame > 0)
			bytes = frames * format->mBytesPerFrame;
		else {
			UInt32 maxPacketSize;
			if (format->mBytesPerPacket > 0)
				maxPacketSize = format->mBytesPerPacket;	// constant packet size
			else {
				UInt32 propertySize = sizeof(maxPacketSize);
				XThrowIfError(AudioQueueGetProperty(mQueue, kAudioQueueProperty_MaximumOutputPacketSize, &maxPacketSize,
												 &propertySize), "couldn't get queue's maximum output packet size");
			}
			if (format->mFramesPerPacket > 0)
				packets = frames / format->mFramesPerPacket;
			else
				packets = frames;	// worst-case scenario: 1 frame in a packet
			if (packets == 0)		// sanity check
				packets = 1;
			bytes = packets * maxPacketSize;
		}
	} catch (CAXException e) {
		char buf[256];
		fprintf(stderr, "Error: %s (%s)\n", e.mOperation, e.FormatError(buf));
		return 0;
	}	
	return bytes;
}

// ____________________________________________________________________________________
// AudioQueue callback function, called when an input buffers has been filled.
void AQRecorder::MyInputBufferHandler(	void *								inUserData,
										AudioQueueRef						inAQ,
										AudioQueueBufferRef					inBuffer,
										const AudioTimeStamp *				inStartTime,
										UInt32								inNumPackets,
										const AudioStreamPacketDescription*	inPacketDesc)
{
	AQRecorder *aqr = (AQRecorder *)inUserData;

	try {
        
        short *coreAudioBuffer;
        coreAudioBuffer = (short*) inBuffer->mAudioData;        
        //unsigned int k=0;
        unsigned i=0;
        short currentValue=0, prevValue=0;
        
        if(DEBUG_ALGORITHM) {
//      printf("sizeof short = %ld bytes\n", sizeof(short));
            //printf("valid data size = %ld bytes\n", inBuffer->mAudioDataByteSize);
//      printf("buffer size = %ld bytes\n", inBuffer->mAudioDataBytesCapacity);
//      //if(inBuffer->mPacketDescriptionCapacity != NULL) {
//          printf("desc capacity = %ld\n", inBuffer->mPacketDescriptionCapacity);
//      //}
        }
        
        if(DEBUG_ALGORITHM) {
            printf("max signal value = %d\n", aqr->maxSigValue);
            printf("min signal value = %d\n", aqr->minSigValue);
            printf("starting processing data...\n");
        }
       
        // every audio sample is 2 bytes
        for(i=0; i<inBuffer->mAudioDataByteSize/2; i++) {
            
            if(aqr->isInitiating) {
                if(i==0) {
                    aqr->tempValues[0] = coreAudioBuffer[i];
                } else if (i==1) {
                    aqr->tempValues[1] = coreAudioBuffer[i];
                } else if (i==2) {
                    aqr->tempValues[2] = coreAudioBuffer[i];
                } else if (i==3) {
                    aqr->tempValues[3] = coreAudioBuffer[i];
                } else {
                    aqr->isInitiating = false;
                }
                continue;
            } else {
                aqr->tempValues[0] = aqr->tempValues[1];
                aqr->tempValues[1] = aqr->tempValues[2];
                aqr->tempValues[2] = aqr->tempValues[3];
                aqr->tempValues[3] = coreAudioBuffer[i];
            }
            
            /*
            if(i%300==0) {
                aqr->maxSigValue--;
                aqr->minSigValue++;
            }
             */
            
            if(aqr->maxSigValue < coreAudioBuffer[i]) {
                aqr->maxSigValue = coreAudioBuffer[i];
                aqr->peakThreshold = aqr->maxSigValue/100*35;   // about 35% of max value
            }
            if(aqr->minSigValue > coreAudioBuffer[i]) {
                aqr->minSigValue = coreAudioBuffer[i];
            }            
            
            prevValue = aqr->tempValues[0];
            currentValue = aqr->tempValues[3];
            
            if(aqr->lookForPacketSync) {
                
                if(abs(currentValue) < SYNC_THRESHOLD) {
                    aqr->syncCounter++;
                } else {
                    aqr->syncCounter = 0;
                }
                if(aqr->syncCounter > SYNC_SAMPLES) {
                    if(DEBUG_ALGORITHM) {
                        printf("packet sync found!\n");
                    }
                    aqr->end = [NSDate date];
                    NSTimeInterval executionTime = [aqr->end timeIntervalSinceDate:aqr->start];
                    if(DEBUG_ALGORITHM) {
                        printf("Packet sync to packet sync time: %f seconds (%f Hz)\n", executionTime, 1.0/executionTime);
                    }
                    aqr->start = [NSDate date];
                    
                    aqr->audioDataIndex = 0;
                    aqr->syncCounter = 0;
                    aqr->iChange = 0;
                    aqr->iStart = 0;
                    aqr->bitValue = 0;
                    aqr->startDetected = false;
                    aqr->currentByte = 0;
                    aqr->lookForPacketSync = false;

                }
            } else {
            
                if((i > aqr->iChange+MIN_PEAK_DISTANCE) && (aqr->bitValue == 0) && ((currentValue - prevValue) > aqr->peakThreshold)) { // found a max peak
                    aqr->bitValue = 1;
                    aqr->iChange = i;
                    if (!aqr->startDetected) {
                        aqr->startDetected = true;
                        aqr->iStart = i;
                    }
                }
                
                if((i > aqr->iChange+MIN_PEAK_DISTANCE) && (aqr->bitValue == 1) && ((currentValue - prevValue) < -aqr->peakThreshold)) {    // found a min peak
                    aqr->bitValue = 0;
                    aqr->iChange = i;
                }
                
                if(aqr->startDetected) {
                    if(i == aqr->iStart+33) {
                        aqr->currentByte = aqr->bitValue*128;
                    }
                    if(i == aqr->iStart+55) {
                        aqr->currentByte += aqr->bitValue*64;
                    }
                    if(i == aqr->iStart+77) {
                        aqr->currentByte += aqr->bitValue*32;
                    }
                    if(i == aqr->iStart+99) {
                        aqr->currentByte += aqr->bitValue*16;
                    }
                    if(i == aqr->iStart+121) {
                        aqr->currentByte += aqr->bitValue*8;
                    }
                    if(i == aqr->iStart+143) {
                        aqr->currentByte += aqr->bitValue*4;
                    }
                    if(i == aqr->iStart+165) {
                        aqr->currentByte += aqr->bitValue*2;
                    }
                    if(i == aqr->iStart+187) {
                        aqr->currentByte += aqr->bitValue*1;
                    }
                    if(i == aqr->iStart+209) {
                        aqr->startDetected = false;
                        
                        if(aqr->bitValue == 0) {
                            
                        } else {    // stop bit not well detected
                            if(DEBUG_ALGORITHM) {
                                printf("Error in interpretation!!!\n");
                            }
                        }
                        
                        aqr->audioData[aqr->audioDataIndex] = aqr->currentByte;
                        if(DEBUG_ALGORITHM) {
                            printf("%d) %d\n", aqr->audioDataIndex, aqr->currentByte);
                        }
                
                        
                        // testing purpose
                        /*
                        if(aqr->waitZero == 1) {    // wait data=0 to start from a known expected value
                            if(aqr->audioDataIndex == 0) {
                                if(aqr->currentByte == 0) {
                                    aqr->waitZero = 0;
                                    aqr->expectedByte = 1;
                                }
                            }
                        } else {
                            if(aqr->audioDataIndex == 0) {  // only first packet byte is used for testing
//                                printf("%d) %d\n", aqr->audioDataIndex, aqr->currentByte);
//                                printf("expected = %d\n", aqr->expectedByte);
                                if(aqr->currentByte != aqr->expectedByte) {
                                    if(aqr->bitValue == 1) {
                                        aqr->numBytesWrong++;
                                    } else {
                                        aqr->numBytesWrongNotDetected++;
                                    }
                                }
                                aqr->numBytesReceived++;
                                if(aqr->expectedByte < 255) {
                                    aqr->expectedByte++;
                                } else {
                                    aqr->expectedByte = 0;
                                }
                            }
                        }
                        */
                        
                        aqr->currentByte = 0;
                        
                        aqr->audioDataIndex++;
                        if(aqr->audioDataIndex == AUDIO_PACKET_SIZE) {
                            aqr->lookForPacketSync = true;
                            
                            [[NSNotificationCenter defaultCenter] postNotificationName:@"sensorsUpdate" object:nil userInfo:nil];
                            
                            // statistics
                            /*
                            aqr->numPacketsReceived++;
//                            printf("bytes wrong = %lld\n", aqr->numBytesWrong);
//                            printf("bytes wrong not detected = %lld\n", aqr->numBytesWrongNotDetected);
//                            printf("total bytes received = %lld\n", aqr->numBytesReceived);
                            printf("corrected bytes received = %f\n", (1.0 - (float)(aqr->numBytesWrong+aqr->numBytesWrongNotDetected)/(float)aqr->numBytesReceived)*100.0);
                             */
                        }
                        
                        
                    }
                    
                }
                
            }
            
        } // end for
        
        
//        printf("data size = %ld\n", inBuffer->mAudioDataByteSize/2);
//        printf("iChange=%lld, iStart=%lld\n", aqr->iChange, aqr->iStart);        
        aqr->iChange -= inBuffer->mAudioDataByteSize/2;
        aqr->iStart -= inBuffer->mAudioDataByteSize/2;
//        printf("iChange=%lld, iStart=%lld\n", aqr->iChange, aqr->iStart);
        
        if(aqr->numPacketsReceived >= 100) {
            aqr->end = [NSDate date];
            NSTimeInterval executionTime = [aqr->end timeIntervalSinceDate:aqr->start];
            printf("Average packet time: %f seconds (%f Hz)\n", executionTime/aqr->numPacketsReceived, 1.0/(executionTime/aqr->numPacketsReceived));
            aqr->start = [NSDate date];
            aqr->numPacketsReceived = 0;
        }
         
//        printf("processing end...\n");
		
		// if we're not stopping, re-enqueue the buffer so that it gets filled again
		if (aqr->IsRunning())
			XThrowIfError(AudioQueueEnqueueBuffer(inAQ, inBuffer, 0, NULL), "AudioQueueEnqueueBuffer failed");
	} catch (CAXException e) {
		char buf[256];
		fprintf(stderr, "Error: %s (%s)\n", e.mOperation, e.FormatError(buf));
	}
}

AQRecorder::AQRecorder() {
    
	mIsRunning = false;
	mRecordPacket = 0;    
    iChange = 0;
    iStart = 0;
    bitValue = 0;
    startDetected = false;
    currentByte = 0;
    expectedByte = 0;
    numBytesReceived = 0;
    numBytesWrong = 0;
    numBytesWrongNotDetected = 0;
    audioDataIndex = 0;
    memset(audioData, 0, AUDIO_PACKET_SIZE);
    waitZero = 1;
    lookForPacketSync = true;
    syncCounter = 0;
    isInitiating = true;
    start = [NSDate date];
    end = [NSDate date];
    numPacketsReceived = 0;
    maxSigValue=0;
    minSigValue=0;
    peakThreshold=0;
    
}

AQRecorder::~AQRecorder() {
    
	AudioQueueDispose(mQueue, TRUE);
    
}

void AQRecorder::SetupAudioFormat(UInt32 inFormatID)
{
	memset(&mRecordFormat, 0, sizeof(mRecordFormat));

	UInt32 size = sizeof(mRecordFormat.mSampleRate);
	XThrowIfError(AudioSessionGetProperty(	kAudioSessionProperty_CurrentHardwareSampleRate,
										&size, 
										&mRecordFormat.mSampleRate), "couldn't get hardware sample rate");

	size = sizeof(mRecordFormat.mChannelsPerFrame);
	XThrowIfError(AudioSessionGetProperty(	kAudioSessionProperty_CurrentHardwareInputNumberChannels, 
										&size, 
										&mRecordFormat.mChannelsPerFrame), "couldn't get input channel count");
			
	mRecordFormat.mFormatID = inFormatID;
	if (inFormatID == kAudioFormatLinearPCM)
	{
		// if we want pcm, default to signed 16-bit little-endian
		mRecordFormat.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger | kLinearPCMFormatFlagIsPacked;
		mRecordFormat.mBitsPerChannel = 16;
		mRecordFormat.mBytesPerPacket = mRecordFormat.mBytesPerFrame = (mRecordFormat.mBitsPerChannel / 8) * mRecordFormat.mChannelsPerFrame;
		mRecordFormat.mFramesPerPacket = 1;
	}
}

void AQRecorder::StartRecord() {
    
	int i, bufferByteSize;
	UInt32 size;
	
	try {

		// specify the recording format
		SetupAudioFormat(kAudioFormatLinearPCM);
		
		// create the queue
		XThrowIfError(AudioQueueNewInput(
									  &mRecordFormat,
									  MyInputBufferHandler,
									  this /* userData */,
									  NULL /* run loop */, NULL /* run loop mode */,
									  0 /* flags */, &mQueue), "AudioQueueNewInput failed");
		
		// get the record format back from the queue's audio converter --
		// the file may require a more specific stream description than was necessary to create the encoder.
		mRecordPacket = 0;

		size = sizeof(mRecordFormat);
		XThrowIfError(AudioQueueGetProperty(mQueue, kAudioQueueProperty_StreamDescription,	
										 &mRecordFormat, &size), "couldn't get queue's format");
        
		// allocate and enqueue buffers
		bufferByteSize = ComputeRecordBufferSize(&mRecordFormat, kBufferDurationSeconds);	// enough bytes for half a second
		for (i = 0; i < kNumberRecordBuffers; ++i) {
			XThrowIfError(AudioQueueAllocateBuffer(mQueue, bufferByteSize, &mBuffers[i]),
					   "AudioQueueAllocateBuffer failed");
			XThrowIfError(AudioQueueEnqueueBuffer(mQueue, mBuffers[i], 0, NULL),
					   "AudioQueueEnqueueBuffer failed");
		}
        
		// start the queue
		mIsRunning = true;
		XThrowIfError(AudioQueueStart(mQueue, NULL), "AudioQueueStart failed");
        
	}
	catch (CAXException e) {
		char buf[256];
		fprintf(stderr, "Error: %s (%s)\n", e.mOperation, e.FormatError(buf));
	}
	catch (...) {
		fprintf(stderr, "An unknown error occurred\n");;
	}	

}

void AQRecorder::StopRecord() {
    
	// end recording
	mIsRunning = false;
	XThrowIfError(AudioQueueStop(mQueue, true), "AudioQueueStop failed");	
	AudioQueueDispose(mQueue, true);

}

void AQRecorder::getPacketData(UInt8* arr) {
    int i=0;
    for(i=0; i<AUDIO_PACKET_SIZE; i++) {
        arr[i] = audioData[i];
    }
}




