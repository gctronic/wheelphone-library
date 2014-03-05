//
//  WheelphoneRobot.m
//  ios-wheelphone-library
//
//  Created by Stefano Morgani on 17.10.13.
//  Copyright (c) 2013 Stefano Morgani. All rights reserved.
//

#import "WheelphoneRobot.h"
#import "AQRecorder.h"
#import "OpenALHelper.h"
#import "MHAudioBufferPlayer.h"

@interface WheelphoneRobot () {
    AQRecorder* recorder;
    MHAudioBufferPlayer *player;
}
@end

@implementation WheelphoneRobot

// COMMUNICATION
#define DTMF_0 0
#define DTMF_1 1
#define DTMF_2 2
#define DTMF_3 3
#define DTMF_4 4
#define DTMF_5 5
#define DTMF_6 6
#define DTMF_7 7
#define DTMF_8 8
#define DTMF_9 9
#define DTMF_STAR 10
#define DTMF_HASH 11
#define DTMF_SPEED_STEP 1

// ROBOT STATE (robot => phone)
int const NOT_CHARGING = 0;
int const CHARGING = 1;
int const CHARGED = 2;

// ROBOT CONTROL (phone => robot)
int const MIN_SPEED_RAW = -127;
int const MAX_SPEED_RAW = 127;
int const MIN_SPEED_REAL = -350;	// 350 mm/s
int const MAX_SPEED_REAL = 350;
int const AVG_DIFF_SPEED_TO_STOP = 35;  // if the difference between the current and previous average speed is greter than this value then the robot is stopped before sending commands to reach the desired speeds; this is to get faster reaction to big change
int const ROT_SPEED_TO_STOP = 10;   // same as AVG_DIFF_SPEED_TO_STOP but for the rotational speed

// VARIOUS
NSString * const debugString = @"WheelphoneRobot";
double const MM_S_TO_BYTE = 2.8;	// scale the speed given in mm/s to a byte sent to the microcontroller
int const SPEED_THR = 3;            // under this value the received measured speed is set to 0 to avoid noisy measure affecting odometry

// ODOMETRY
int const X_ODOM = 0;
int const Y_ODOM = 1;
int const THETA_ODOM = 2;

+ (NSBundle *)frameworkBundle {
    static NSBundle* frameworkBundle = nil;
    static dispatch_once_t predicate;
    dispatch_once(&predicate, ^{
        NSString* mainBundlePath = [[NSBundle mainBundle] resourcePath];
        NSString* frameworkBundlePath = [mainBundlePath stringByAppendingPathComponent:@"ios-wheelphone-library.bundle"];
        frameworkBundle = [NSBundle bundleWithPath:frameworkBundlePath];
    });
    return frameworkBundle;
}

- (id)init
{
    self = [super init];
    if (self) {
        maxBatteryValue = 152;
        chargeState = NOT_CHARGING;
        [self enableSpeedControl]; // enable speed control by default
        [self enableSoftAcceleration];  // enable soft acceleration by default
        debug = false;
        leftDiamCoeff = 1.0;
        rightDiamCoeff = 1.0;
        leftDist = 0.0;
        rightDist = 0.0;
        leftDistPrev = 0.0;
        rightDistPrev = 0.0;
        wheelBase = 0.087;
        logEnabled = false;
        audioFilePath[DTMF_0] = [[[self class] frameworkBundle] pathForResource:@"dtmf0" ofType:@"wav"];
        audioFilePath[DTMF_1] = [[[self class] frameworkBundle] pathForResource:@"dtmf1" ofType:@"wav"];
        audioFilePath[DTMF_2] = [[[self class] frameworkBundle] pathForResource:@"dtmf2" ofType:@"wav"];
        audioFilePath[DTMF_3] = [[[self class] frameworkBundle] pathForResource:@"dtmf3" ofType:@"wav"];
        audioFilePath[DTMF_4] = [[[self class] frameworkBundle] pathForResource:@"dtmf4" ofType:@"wav"];
        audioFilePath[DTMF_5] = [[[self class] frameworkBundle] pathForResource:@"dtmf5" ofType:@"wav"];
        audioFilePath[DTMF_6] = [[[self class] frameworkBundle] pathForResource:@"dtmf6" ofType:@"wav"];
        audioFilePath[DTMF_7] = [[[self class] frameworkBundle] pathForResource:@"dtmf7" ofType:@"wav"];
        audioFilePath[DTMF_8] = [[[self class] frameworkBundle] pathForResource:@"dtmf8" ofType:@"wav"];
        audioFilePath[DTMF_9] = [[[self class] frameworkBundle] pathForResource:@"dtmf9" ofType:@"wav"];
        audioFilePath[DTMF_STAR] = [[[self class] frameworkBundle] pathForResource:@"dtmf_star" ofType:@"wav"];
        audioFilePath[DTMF_HASH] = [[[self class] frameworkBundle] pathForResource:@"dtmf_hash" ofType:@"wav"];
        audioFileURL[DTMF_0] = [NSURL fileURLWithPath:audioFilePath[DTMF_0]];
        audioFileURL[DTMF_1] = [NSURL fileURLWithPath:audioFilePath[DTMF_1]];
        audioFileURL[DTMF_2] = [NSURL fileURLWithPath:audioFilePath[DTMF_2]];
        audioFileURL[DTMF_3] = [NSURL fileURLWithPath:audioFilePath[DTMF_3]];
        audioFileURL[DTMF_4] = [NSURL fileURLWithPath:audioFilePath[DTMF_4]];
        audioFileURL[DTMF_5] = [NSURL fileURLWithPath:audioFilePath[DTMF_5]];
        audioFileURL[DTMF_6] = [NSURL fileURLWithPath:audioFilePath[DTMF_6]];
        audioFileURL[DTMF_7] = [NSURL fileURLWithPath:audioFilePath[DTMF_7]];
        audioFileURL[DTMF_8] = [NSURL fileURLWithPath:audioFilePath[DTMF_8]];
        audioFileURL[DTMF_9] = [NSURL fileURLWithPath:audioFilePath[DTMF_9]];
        audioFileURL[DTMF_STAR] = [NSURL fileURLWithPath:audioFilePath[DTMF_STAR]];
        audioFileURL[DTMF_HASH] = [NSURL fileURLWithPath:audioFilePath[DTMF_HASH]];
        commTimeout = 0;
        commTimeoutLimit = 50;
        isConnected = false;
        stopSent = false;
        
        //NSString *soundPath = [[[self class] frameworkBundle] pathForResource:@"dtmf0" ofType:@"wav"];
        //NSURL *soundURL = [NSURL fileURLWithPath:soundPath];
        //AudioServicesCreateSystemSoundID((__bridge CFURLRef)soundURL, &_calibrateSound);
        
        //[OpenALHelper loadSoundNamed:@"dtmf0" withFileName:@"dtmf0" andExtension:@"wav"];
        //[OpenALHelper loadSoundNamed:@"dtmf5" withFileName:@"dtmf5" andExtension:@"wav"];
        //[OpenALHelper loadSoundNamed:@"sosumi" withFileName:@"Sosumi" andExtension:@"caf"];

        [self initAudioCommunication];
        
        [[NSNotificationCenter defaultCenter] addObserver: self selector: @selector(updateSensorsNotification:) name: @"sensorsUpdate" object: nil];
        
        [NSThread detachNewThreadSelector:@selector(handleCommandsToRobotTask) toTarget:self withObject:nil];
        
        if(debug) {
            NSLog(@"Library initialized!\n");
        }
        
    }
    
    return self;
}

- (void)initAudioCommunication {
    
    // Allocate our singleton instance for the recorder object
    recorder = new AQRecorder();
    
    OSStatus error = AudioSessionInitialize(NULL, NULL, interruptionListener, (__bridge void *)self);
    if (error) printf("ERROR INITIALIZING AUDIO SESSION! %d\n", (int)error);
    else
    {
        UInt32 category = kAudioSessionCategory_PlayAndRecord;
        //UInt32 category = kAudioSessionCategory_RecordAudio;
        error = AudioSessionSetProperty(kAudioSessionProperty_AudioCategory, sizeof(category), &category);
        if (error) printf("couldn't set audio category!");
        
        //category = kAudioSessionMode_VoiceChat;
        //error = AudioSessionSetProperty(kAudioSessionProperty_Mode, sizeof(category), &category);
        //if (error) printf("couldn't set audio mode!");
        
        //UInt32 allowMixing = true;
        //AudioSessionSetProperty(kAudioSessionProperty_OverrideCategoryMixWithOthers, sizeof(allowMixing), &allowMixing);
        //AudioSessionSetProperty(kAudioSessionProperty_OtherMixableAudioShouldDuck, sizeof(allowMixing), &allowMixing);
        
        error = AudioSessionAddPropertyListener(kAudioSessionProperty_AudioRouteChange, propListener, (__bridge void *)self);
        if (error) printf("ERROR ADDING AUDIO SESSION PROP LISTENER! %d\n", (int)error);
        UInt32 inputAvailable = 0;
        UInt32 size = sizeof(inputAvailable);
        
        // we do not want to allow recording if input is not available
        error = AudioSessionGetProperty(kAudioSessionProperty_AudioInputAvailable, &size, &inputAvailable);
        if (error) printf("ERROR GETTING INPUT AVAILABILITY! %d\n", (int)error);
        
        // we also need to listen to see if input availability changes
        error = AudioSessionAddPropertyListener(kAudioSessionProperty_AudioInputAvailable, propListener, (__bridge void *)self);
        if (error) printf("ERROR ADDING AUDIO SESSION PROP LISTENER! %d\n", (int)error);
        
        error = AudioSessionSetActive(true);
        if (error) printf("AudioSessionSetActive (true) failed");
    }
    
    // Start the recorder
    recorder->StartRecord();
    
	player = [[MHAudioBufferPlayer alloc] initWithSampleRate:8000.0 channels:1 bitsPerChannel:16 packetsPerBuffer:NUM_PACKETS];
	player.gain = 1.0f;
    // This callback runs on an internal Audio Queue thread
	player.block = ^(AudioQueueBufferRef buffer, AudioStreamBasicDescription audioFormat) {
            
        // Calculate how many packets fit into this buffer. Remember that a
        // packet equals one frame because we are dealing with uncompressed
        // audio; a frame is a set of left+right samples for stereo sound,
        // or a single sample for mono sound. Each sample consists of one
        // or more bytes. So for 16-bit mono sound, each packet is 2 bytes.
        // For stereo it would be 4 bytes.
        int packetsPerBuffer = buffer->mAudioDataBytesCapacity / audioFormat.mBytesPerPacket;
        
        // fill up buffers
        audioState = AUDIO_SEND_START;
        SInt16* p = (SInt16 *)buffer->mAudioData;
        int parityValue = 0;
        int counter = 0;
        int currentBitValue = 0;
        int bitIndex = 0;
        int currPacket=0;
        audioSeqIndex = 0;
        checkSum = (((audioSeq[0]+audioSeq[1]+audioSeq[2])&0x00FF)^0xFF)+0x01;
        audioSeq[3] = checkSum;
        while(1) {
            
            if(audioState == AUDIO_SEND_START) {
                currentBitValue = BIT_VALUE;
                bitIndex = 7;
                audioState = SENDING_START;
                counter = 0;
            } else if(audioState == SENDING_START) {
                p[currPacket] = currentBitValue;
                currPacket = currPacket + 1;
                if(currPacket >= packetsPerBuffer) {
                    break;
                }
                counter = counter + 1;
                if(counter == SAMPLES_PER_BIT_500_US) {
                    counter = 0;
                    audioState = AUDIO_SEND_DATA;
                }
            } else if(audioState == AUDIO_SEND_DATA) {
                if(audioSeq[audioSeqIndex] & (1<<bitIndex)) {
                    currentBitValue = BIT_VALUE;
                    parityValue = parityValue^1;
                } else {
                    currentBitValue = -BIT_VALUE;
                    parityValue = parityValue^0;
                }
                bitIndex = bitIndex - 1;
                audioState = SENDING_DATA;
            } else if(audioState == SENDING_DATA) {
                p[currPacket] = currentBitValue;
                currPacket = currPacket + 1;
                if(currPacket >= packetsPerBuffer) {
                    break;
                }
                counter = counter + 1;
                if(counter == SAMPLES_PER_BIT_500_US) {
                    counter = 0;
                    if(bitIndex < 0) {
                        //audioState = AUDIO_SEND_PARITY;
                        audioState = AUDIO_SEND_STOP;
                    } else {
                        audioState = AUDIO_SEND_DATA;
                    }
                }
            } /*else if(audioState == AUDIO_SEND_PARITY) {
                if(parityValue == 1) {    // even parity
                    currentBitValue = BIT_VALUE;
                } else {
                    currentBitValue = -BIT_VALUE;
                }
                parityValue = 0;
                audioState = SENDING_PARITY;
            } else if(audioState == SENDING_PARITY) {
                p[currPacket] = currentBitValue;
                currPacket = currPacket + 1;
                if(currPacket >= packetsPerBuffer) {
                    break;
                }
                counter = counter + 1;
                if(counter == SAMPLES_PER_BIT_500_US) {
                    counter = 0;
                    audioState = AUDIO_SEND_STOP;
                }
            }*/ else if(audioState == AUDIO_SEND_STOP) {
                currentBitValue = -BIT_VALUE;
                if(audioSeqIndex < AUDIO_SEQ_NUM_BYTES-1) {
                    audioSeqIndex = audioSeqIndex + 1;
                } else {
                    audioSeqIndex = 0;
                }
                audioState = SENDING_STOP;
            } else if(audioState == SENDING_STOP) {
                p[currPacket] = currentBitValue;
                currPacket = currPacket + 1;
                if(currPacket >= packetsPerBuffer) {
                    break;
                }
                counter = counter + 1;
                if(counter == SAMPLES_PER_BIT_500_US) {
                    counter = 0;
                    if(audioSeqIndex == 0) { // complete packet sent
                        bitIndex = SYNC_BITS;
                        audioState = AUDIO_SEND_SYNC_PAUSE;
                    } else {
                        audioState = AUDIO_SEND_START;
                    }
                }
            } else if(audioState == AUDIO_SEND_SYNC_PAUSE) { // send n bytes of low signal between each packet in order to sync
                bitIndex = bitIndex - 1;
                if(bitIndex < 0) {
                    audioState = AUDIO_SEND_START;
                } else {
                    audioState = SENDING_SYNC;
                }
            } else if(audioState == SENDING_SYNC) {
                p[currPacket] = currentBitValue;
                currPacket = currPacket + 1;
                if(currPacket >= packetsPerBuffer) {
                    break;
                }
                counter = counter + 1;
                if(counter == SAMPLES_PER_BIT_500_US) {
                    counter = 0;
                    audioState = AUDIO_SEND_SYNC_PAUSE;
                }
            }
                
            
        }
        
        // We have to tell the buffer how many bytes we wrote into it.
        buffer->mAudioDataByteSize = packetsPerBuffer * audioFormat.mBytesPerPacket;
        
        flagPhoneToRobotSent = 1;
        
        flagPhoneToRobot &= ~(1 << 4);      // stop sending calibration flag after it is sent once
        audioSeq[2] = flagPhoneToRobot;
        
	};
    
    [self registerForBackgroundNotifications];
    
    /*
    while(1) {
        [self sendSerialAudioSequence];
        [NSThread sleepForTimeInterval:0.100];
        [player start];
        if([self enableObstacleAvoidance] == 0) {
            break;
        } else {
            [player stop];
            [NSThread sleepForTimeInterval:0.350];
        }
    }
    [self disableObstacleAvoidance];
    */
    
	//[player start];
    
    startTime = [NSDate date];  // initialize startTime otherwise "timeIntervalSinceDate" will returns "nan" when it
                                // will be called for calculating odometry. It is initialized here after the communication
                                // is started.

}

- (void)sendSerialAudioSequence {
    
    // we need to send a special sequence of DTMF commands in order to pass to "serial audio data" transmission
    
    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_STAR] error:nil];
    [testAudioPlayer prepareToPlay];
    [testAudioPlayer play];
    [NSThread sleepForTimeInterval:0.120];
    
    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_HASH] error:nil];
    [testAudioPlayer prepareToPlay];
    [testAudioPlayer play];
    [NSThread sleepForTimeInterval:0.120];
    
    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_STAR] error:nil];
    [testAudioPlayer prepareToPlay];
    [testAudioPlayer play];
    [NSThread sleepForTimeInterval:0.120];
    
    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_HASH] error:nil];
    [testAudioPlayer prepareToPlay];
    [testAudioPlayer play];
    [NSThread sleepForTimeInterval:0.120];
    
    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_5] error:nil];
    [testAudioPlayer prepareToPlay];
    [testAudioPlayer play];
    [NSThread sleepForTimeInterval:0.120];
    
}

- (void)stopRecord {
    recorder->StopRecord();
}

- (void)handleCommandsToRobotTask {
    
    //int counter=0;
    //int tempSpeed=0;
    BOOL sleepDone=false;
    double pause=0.045; // wait 45 ms to let the command be interpreted
    
    //NSDate *start = [NSDate date];
    //NSDate *stop = [NSDate date];
    //NSTimeInterval executionTime = [stop timeIntervalSinceDate:start];
    
    //NSDate *start1 = [NSDate date];
    //NSDate *stop1 = [NSDate date];
    //NSTimeInterval executionTime1 = [stop timeIntervalSinceDate:start];
    
    while(1) {
        
        commTimeout++;
        if(commTimeout == commTimeoutLimit) {		// about "50*commTimeoutLimit" ms is passed without any answer from the robot
            isConnected = false; // robot disconnected
        }
        
        //start1 = [NSDate date];
        
        sleepDone = false;
        
        /*
         if(recorder->isFollowingEnabled()) {
         //counter++;
         //if(counter>=1) {
         //    counter=0;
         if(tempSpeed <= (125-125)) {
         tempSpeed+=125;
         } else {
         tempSpeed = 125;
         }
         //}
         desiredLeftSpeed = tempSpeed;
         desiredRightSpeed = 0;
         }
         */
        
        /*
        noStopCount++;
        if(noStopCount >= 10) {
            noStopCount = 10;
            stopSent = false;
        }
        */
        
        if(lSpeed==0 && rSpeed==0 && stopSent==false) {
            currentLeftSpeed=0;
            currentRightSpeed=0;
            
            //played=false;
            //[testAudioPlayer[DTMF_5] prepareToPlay];
            //[testAudioPlayer[DTMF_5] play];
            //while(!played);
            
            //[testAudioPlayer[DTMF_5] prepareToPlay];
            //[testAudioPlayer[DTMF_5] play];
            NSError * error = NULL;
            testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_5] error:&error];
            if(testAudioPlayer == NULL) {
                NSLog( @"error in creating AVAudioPlayer - %@ %@", [error domain], [error localizedDescription] );
            }
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            if(debug) {
                printf("DTMF_5 sent\n");
            }
            
            /*
            NSError * error = nil ;
            NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_5] options:NSDataReadingMapped error:&error ] ;
            if (!data) {
                continue;
                //if (error) { @throw error ;}
            }
            AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
            if (!audioPlayer) {
                continue;
                //if ( error ) { @throw error ; }
            }
            [audioPlayer prepareToPlay];
            [audioPlayer play];
            */
            
            //[OpenALHelper playSoundNamed:@"dtmf5"];
            //[OpenALHelper playSoundNamed:@"sosumi"];
            
            //printf("5\n");
            [NSThread sleepForTimeInterval:pause];
            
            // play twice the stop to be sure it is received
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            [NSThread sleepForTimeInterval:pause];
            if(debug) {
                printf("DTMF_5 sent\n");
            }
            
            sleepDone = true;
            stopSent = true;
        } else if((lSpeed*currentLeftSpeed)<0 && (rSpeed*currentRightSpeed)<0) {   // inverted direction for both motors
            currentLeftSpeed=0;
            currentRightSpeed=0;
            
            //played=false;
            //[testAudioPlayer[DTMF_5] prepareToPlay];
            //[testAudioPlayer[DTMF_5] play];
            //while(!played);
            
            //[testAudioPlayer[DTMF_5] prepareToPlay];
            //[testAudioPlayer[DTMF_5] play];
            
            testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_5] error:nil];
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            if(debug) {
                printf("DTMF_5 sent\n");
            }
            
            /*
            NSError * error = nil ;
            NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_5] options:NSDataReadingMapped error:&error ] ;
            if (!data) {
                continue;
                //if (error) { @throw error ;}
            }
            AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
            if (!audioPlayer) {
                continue;
                //if ( error ) { @throw error ; }
            }
            [audioPlayer prepareToPlay];
            [audioPlayer play];
            */
            
            //printf("5\n");
            [NSThread sleepForTimeInterval:pause];
            
            // play twice the stop to be sure it is received
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            [NSThread sleepForTimeInterval:pause];
            if(debug) {
                printf("DTMF_5 sent\n");
            }
            
            sleepDone = true;
            stopSent = false;
        } /*else if((abs(avgSpeedPrev)-abs(avgSpeed))>AVG_DIFF_SPEED_TO_STOP && stopSent==false) {   // big change in average speed and stop not already just sent
            stopSent = true;
            noStopCount = 0;
            
            currentLeftSpeed=0;
            currentRightSpeed=0;
            
            testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_5] error:nil];
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            
            //printf("5\n");
            [NSThread sleepForTimeInterval:pause];
            
            // play twice the stop to be sure it is received
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            [NSThread sleepForTimeInterval:pause];
            
            sleepDone = true;
        } else if(abs(rotSpeed)>ROT_SPEED_TO_STOP && stopSent==false) {
            stopSent = true;
            noStopCount = 0;
           
            currentLeftSpeed=0;
            currentRightSpeed=0;
            
            testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_5] error:nil];
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            
            //printf("5\n");
            [NSThread sleepForTimeInterval:pause];
            
            // play twice the stop to be sure it is received
            [testAudioPlayer prepareToPlay];
            [testAudioPlayer play];
            [NSThread sleepForTimeInterval:pause];
            
            sleepDone = true;
        }*/ else {
            
            int diffLeft = lSpeed-currentLeftSpeed;
            int diffRight = rSpeed-currentRightSpeed;
            
            if(diffLeft>=DTMF_SPEED_STEP && diffRight>=DTMF_SPEED_STEP) {   // current speed is lower than desired for both motors
                currentLeftSpeed+=DTMF_SPEED_STEP;
                currentRightSpeed+=DTMF_SPEED_STEP;
                
                //played=false;
                //[testAudioPlayer[DTMF_2] prepareToPlay];
                //[testAudioPlayer[DTMF_2] play];
                //while(!played);
                
                //[testAudioPlayer[DTMF_2] prepareToPlay];
                //[testAudioPlayer[DTMF_2] play];
                
                testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_2] error:nil];
                [testAudioPlayer prepareToPlay];
                [testAudioPlayer play];
                if(debug) {
                    printf("DTMF_2 sent\n");
                }

                /*
                NSError * error = nil ;
                NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_2] options:NSDataReadingMapped error:&error ] ;
                if (!data) {
                    continue;
                    //if (error) { @throw error ;}
                }
                AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                if (!audioPlayer) {
                    continue;
                    //if ( error ) { @throw error ; }
                }
                [audioPlayer prepareToPlay];
                [audioPlayer play];
                */
                
                //printf("2 (diffL=%d, diffR=%d)\n", diffLeft, diffRight);
                [NSThread sleepForTimeInterval:pause];
                
                stopSent = false;
                sleepDone = true;
            } else if(diffLeft<=-DTMF_SPEED_STEP && diffRight<=-DTMF_SPEED_STEP) {    // current speed is higher than desired for both motors
                currentLeftSpeed-=DTMF_SPEED_STEP;
                currentRightSpeed-=DTMF_SPEED_STEP;
                
                //played=false;
                //[testAudioPlayer[DTMF_8] prepareToPlay];
                //[testAudioPlayer[DTMF_8] play];
                //while(!played);
                
                //[testAudioPlayer[DTMF_8] prepareToPlay];
                //[testAudioPlayer[DTMF_8] play];
                
                testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_8] error:nil];
                [testAudioPlayer prepareToPlay];
                [testAudioPlayer play];
                if(debug) {
                    printf("DTMF_8 sent\n");
                }
                
                /*
                NSError * error = nil ;
                NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_8] options:NSDataReadingMapped error:&error ] ;
                if (!data) {
                    continue;
                    //if (error) { @throw error ;}
                }
                AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                if (!audioPlayer) {
                    continue;
                    //if ( error ) { @throw error ; }
                }
                [audioPlayer prepareToPlay];
                [audioPlayer play];
                */
                
                //printf("8 (diffL=%d, diffR=%d)\n", diffLeft, diffRight);
                [NSThread sleepForTimeInterval:pause];
                
                stopSent = false;
                sleepDone = true;
            } else {
                
                if(diffLeft>=DTMF_SPEED_STEP) { // current left speed is lower than desired
                    currentLeftSpeed+=DTMF_SPEED_STEP;
                    
                    //start = [NSDate date];
                    
                    //AudioServicesPlaySystemSound(_fwSound);
                    
                    //played=false;
                    //[testAudioPlayer[DTMF_1] prepareToPlay];
                    //[testAudioPlayer[DTMF_1] play];
                    //while(!played);
                    
                    //[testAudioPlayer[DTMF_1] prepareToPlay];
                    //[testAudioPlayer[DTMF_1] play];
                    
                    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_1] error:nil];
                    [testAudioPlayer prepareToPlay];
                    [testAudioPlayer play];
                    if(debug) {
                        printf("DTMF_1 sent\n");
                    }
                    
                    /*
                    NSError * error = nil ;
                    NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_1] options:NSDataReadingMapped error:&error ] ;
                    if (!data) {
                        continue;
                        //if (error) { @throw error ;}
                    }
                    AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                    if (!audioPlayer) {
                        continue;
                        //if ( error ) { @throw error ; }
                    }
                    [audioPlayer prepareToPlay];
                    [audioPlayer play];
                    */
                    
                    //stop = [NSDate date];
                    //executionTime = [stop timeIntervalSinceDate:start];
                    //printf("execution time 1 = %f\n", executionTime);
                    
                    //printf("1 (diff=%d)\n", diffLeft);
                    
                    //start = [NSDate date];
                    [NSThread sleepForTimeInterval:pause];
                    //[testAudioPlayer[DTMF_1] prepareToPlay];
                    //stop = [NSDate date];
                    //executionTime = [stop timeIntervalSinceDate:start];
                    //printf("execution time 2 = %f\n", executionTime);
                    
                    stopSent = false;
                    sleepDone = true;
                } else if(diffLeft<=-DTMF_SPEED_STEP) {  // current left speed is higher than desired
                    currentLeftSpeed-=DTMF_SPEED_STEP;
                    
                    //played=false;
                    //[testAudioPlayer[DTMF_7] prepareToPlay];
                    //[testAudioPlayer[DTMF_7] play];
                    //while(!played);
                    
                    //[testAudioPlayer[DTMF_7] prepareToPlay];
                    //[testAudioPlayer[DTMF_7] play];
                    
                    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_7] error:nil];
                    [testAudioPlayer prepareToPlay];
                    [testAudioPlayer play];
                    if(debug) {
                        printf("DTMF_7 sent\n");
                    }
                    
                    /*
                    NSError * error = nil ;
                    NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_7] options:NSDataReadingMapped error:&error ] ;
                    if (!data) {
                        continue;
                        //if (error) { @throw error ;}
                    }
                    AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                    if (!audioPlayer) {
                        continue;
                        //if ( error ) { @throw error ; }
                    }
                    [audioPlayer prepareToPlay];
                    [audioPlayer play];
                     */
                    
                    //printf("7 (diff=%d)\n", diffLeft);
                    [NSThread sleepForTimeInterval:pause];
                    
                    stopSent = false;
                    sleepDone = true;
                }
                
                if(diffRight>=DTMF_SPEED_STEP) { // current right speed is lower than desired
                    currentRightSpeed+=DTMF_SPEED_STEP;
                    
                    //played=false;
                    //[testAudioPlayer[DTMF_3] prepareToPlay];
                    //[testAudioPlayer[DTMF_3] play];
                    //while(!played);
                    
                    //[testAudioPlayer[DTMF_3] prepareToPlay];
                    //[testAudioPlayer[DTMF_3] play];
                    
                    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_3] error:nil];
                    [testAudioPlayer prepareToPlay];
                    [testAudioPlayer play];
                    if(debug) {
                        printf("DTMF_3 sent\n");
                    }
                    
                    /*
                    NSError * error = nil ;
                    NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_3] options:NSDataReadingMapped error:&error ] ;
                    if (!data) {
                        continue;
                        //if (error) { @throw error ;}
                    }
                    AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                    if (!audioPlayer) {
                        continue;
                        //if ( error ) { @throw error ; }
                    }
                    [audioPlayer prepareToPlay];
                    [audioPlayer play];
                    */
                    
                    //printf("3 (diff=%d)\n", diffRight);
                    [NSThread sleepForTimeInterval:pause];
                    
                    stopSent = false;
                    sleepDone = true;
                } else if(diffRight<=-DTMF_SPEED_STEP) {
                    currentRightSpeed-=DTMF_SPEED_STEP;
                    
                    //played=false;
                    //[testAudioPlayer[DTMF_9] prepareToPlay];
                    //[testAudioPlayer[DTMF_9] play];
                    //while(!played);
                    
                    //[testAudioPlayer[DTMF_9] prepareToPlay];
                    //[testAudioPlayer[DTMF_9] play];
                    
                    testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_9] error:nil];
                    [testAudioPlayer prepareToPlay];
                    [testAudioPlayer play];
                    if(debug) {
                        printf("DTMF_9 sent\n");
                    }
                    
                    /*
                    NSError * error = nil ;
                    NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_9] options:NSDataReadingMapped error:&error ] ;
                    if (!data) {
                        continue;
                        //if (error) { @throw error ;}
                    }
                    AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                    if (!audioPlayer) {
                        continue;
                        //if ( error ) { @throw error ; }
                    }
                    [audioPlayer prepareToPlay];
                    [audioPlayer play];
                    */
                    
                    //printf("9 (diff=%d)\n", diffRight);
                    [NSThread sleepForTimeInterval:pause];
                    
                    stopSent = false;
                    sleepDone = true;
                }
                
            }
        }
        
        //printf("des(%d,%d), curr(%d,%d), robot(%d,%d)\n", desiredLeftSpeed, desiredRightSpeed, currentLeftSpeed, currentRightSpeed, robotL, robotR);
        
        //}
        
        if(currFlagPhoneToRobot != flagPhoneToRobot) {
            // speed control and soft acceleration are always enabled in audio communication mode
            
            if((currFlagPhoneToRobot&0x04) != (flagPhoneToRobot&0x04)) {    // obstacle avoidance
                testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_STAR] error:nil];
                [testAudioPlayer prepareToPlay];
                [testAudioPlayer play];
                if(debug) {
                    printf("DTMF_STAR sent\n");
                }
                
                /*
                NSError * error = nil ;
                NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_STAR] options:NSDataReadingMapped error:&error ] ;
                if (!data) {
                    continue;
                    //if (error) { @throw error ;}
                }
                AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                if (!audioPlayer) {
                    continue;
                    //if ( error ) { @throw error ; }
                }
                [audioPlayer prepareToPlay];
                [audioPlayer play];
                */
                
                [NSThread sleepForTimeInterval:pause];
                sleepDone = true;
                if((flagPhoneToRobot&0x04) > 0) {
                    currFlagPhoneToRobot |= (1 << 2);
                } else {
                    currFlagPhoneToRobot &= ~(1 << 2);
                }
                
            }
            
            if((currFlagPhoneToRobot&0x08) != (flagPhoneToRobot&0x08)) {    // cliff avoidance
                testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_HASH] error:nil];
                [testAudioPlayer prepareToPlay];
                [testAudioPlayer play];
                if(debug) {
                    printf("DTMF_HASH sent\n");
                }
                
                /*
                NSError * error = nil ;
                NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_HASH] options:NSDataReadingMapped error:&error ] ;
                if (!data) {
                    continue;
                    //if (error) { @throw error ;}
                }
                AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                if (!audioPlayer) {
                    continue;
                    //if ( error ) { @throw error ; }
                }
                [audioPlayer prepareToPlay];
                [audioPlayer play];
                */
                
                [NSThread sleepForTimeInterval:pause];
                sleepDone = true;
                if((flagPhoneToRobot&0x08) > 0) {
                    currFlagPhoneToRobot |= (1 << 3);
                } else {
                    currFlagPhoneToRobot &= ~(1 << 3);
                }
                
            }
            
            if((flagPhoneToRobot&0x10) > 0) {    // calibrate sensor
                testAudioPlayer = [[AVAudioPlayer alloc] initWithContentsOfURL:audioFileURL[DTMF_0] error:nil];
                [testAudioPlayer prepareToPlay];
                //[testAudioPlayer setVolume:1.0];
                [testAudioPlayer play];
                if(debug) {
                    printf("DTMF_0 sent\n");
                }
                
                /*
                NSError * error = nil ;
                NSData * data = [ NSData dataWithContentsOfFile:audioFilePath[DTMF_0] options:NSDataReadingMapped error:&error ] ;
                if (!data) {
                    continue;
                    //if (error) { @throw error ;}
                }
                AVAudioPlayer *audioPlayer = data ? [[AVAudioPlayer alloc] initWithData:data error:&error ] : nil ;
                if (!audioPlayer) {
                    continue;
                    //if ( error ) { @throw error ; }
                }
                [audioPlayer prepareToPlay];
                [audioPlayer play];
                */
                
                //AudioServicesPlaySystemSound(_calibrateSound);
                
                //[OpenALHelper playSoundNamed:@"dtmf0"];
                
                [NSThread sleepForTimeInterval:pause];
                sleepDone = true;
                flagPhoneToRobot &= ~(1 << 4);                
            }
        
        }
        
        if(!sleepDone) {
            [NSThread sleepForTimeInterval:0.015];   // wait at least 15 ms to avoid running continuously
        }
        
        //stop1 = [NSDate date];
        //executionTime1 = [stop1 timeIntervalSinceDate:start1];
        //printf("total time = %f\n", executionTime1);
        
    }
    //[self performSelectorOnMainThread:@selector(makeMyProgressBarMoving) withObject:nil waitUntilDone:NO];
    
}

#pragma mark AudioSession listeners
void interruptionListener(	void *	inClientData, UInt32	inInterruptionState) {
	WheelphoneRobot *THIS = (__bridge WheelphoneRobot*)inClientData;
	if (inInterruptionState == kAudioSessionBeginInterruption)
	{
		if (THIS->recorder->IsRunning()) {
			[THIS stopRecord];
		}
        [THIS->player tearDownAudio];
	}
    else if (inInterruptionState == kAudioSessionEndInterruption)
	{
		[THIS->player setUpAudio];
		[THIS->player start];
	}
    
}

void propListener(	void *                  inClientData,
                  AudioSessionPropertyID	inID,
                  UInt32                  inDataSize,
                  const void *            inData)
{
	WheelphoneRobot *THIS = (__bridge WheelphoneRobot*)inClientData;
	if (inID == kAudioSessionProperty_AudioRouteChange)
	{
		CFDictionaryRef routeDictionary = (CFDictionaryRef)inData;
		//CFShow(routeDictionary);
		CFNumberRef reason = (CFNumberRef)CFDictionaryGetValue(routeDictionary, CFSTR(kAudioSession_AudioRouteChangeKey_Reason));
		SInt32 reasonVal;
		CFNumberGetValue(reason, kCFNumberSInt32Type, &reasonVal);
		if (reasonVal != kAudioSessionRouteChangeReason_CategoryChange)
		{
			/*CFStringRef oldRoute = (CFStringRef)CFDictionaryGetValue(routeDictionary, CFSTR(kAudioSession_AudioRouteChangeKey_OldRoute));
             if (oldRoute)
             {
             printf("old route:\n");
             CFShow(oldRoute);
             }
             else
             printf("ERROR GETTING OLD AUDIO ROUTE!\n");
             
             CFStringRef newRoute;
             UInt32 size; size = sizeof(CFStringRef);
             OSStatus error = AudioSessionGetProperty(kAudioSessionProperty_AudioRoute, &size, &newRoute);
             if (error) printf("ERROR GETTING NEW AUDIO ROUTE! %d\n", error);
             else
             {
             printf("new route:\n");
             CFShow(newRoute);
             }*/
            
			if (reasonVal == kAudioSessionRouteChangeReason_OldDeviceUnavailable)
			{
                //				if (THIS->player->IsRunning()) {
                //					[THIS pausePlayQueue];
                //					[[NSNotificationCenter defaultCenter] postNotificationName:@"playbackQueueStopped" object:THIS];
                //				}
			}
            
			// stop the queue if we had a non-policy route change
			if (THIS->recorder->IsRunning()) {
				[THIS stopRecord];
			}
		}
	}
	else if (inID == kAudioSessionProperty_AudioInputAvailable)
	{
		if (inDataSize == sizeof(UInt32)) {
			//UInt32 isAvailable = *(UInt32*)inData;
			// disable recording if input is not available
		}
	}
}

#pragma mark background notifications
- (void)registerForBackgroundNotifications
{
	[[NSNotificationCenter defaultCenter] addObserver:self
											 selector:@selector(resignActive)
												 name:UIApplicationWillResignActiveNotification
											   object:nil];
	
	[[NSNotificationCenter defaultCenter] addObserver:self
											 selector:@selector(enterForeground)
												 name:UIApplicationWillEnterForegroundNotification
											   object:nil];
}

- (void)resignActive
{
    if (recorder->IsRunning()) [self stopRecord];
}

- (void)enterForeground
{
    OSStatus error = AudioSessionSetActive(true);
    if (error) printf("AudioSessionSetActive (true) failed");
}

#pragma mark Cleanup
- (void)dealloc
{
	delete recorder;
    
}


-(void)updateSensorsNotification: (NSNotification*)notification {
    
    UInt8 currPacket[18]={0};
    recorder->getPacketData(currPacket);
    
    if ([[notification name] isEqualToString:@"sensorsUpdate"]) {
        
        proxValues[0] = (int)currPacket[0];
        proxValues[1] = (int)currPacket[1];
        proxValues[2] = (int)currPacket[2];
        proxValues[3] = (int)currPacket[3];
        
        proxAmbientValues[0] = (int)currPacket[4];
        proxAmbientValues[1] = (int)currPacket[5];
        proxAmbientValues[2] = (int)currPacket[6];
        proxAmbientValues[3] = (int)currPacket[7];
        
        groundValues[0] = (int)currPacket[8];
        groundValues[1] = (int)currPacket[9];
        groundValues[2] = (int)currPacket[10];
        groundValues[3] = (int)currPacket[11];
        
        battery = (int)currPacket[12];
        
        flagRobotToPhone = (int)currPacket[13];
        flagRobotToPhoneReceived = 1;
        printf("flagRobotToPhone=%d\n",flagRobotToPhone);
        
        leftMeasuredSpeed = (signed int)(currPacket[14] + currPacket[15]*256);
        rightMeasuredSpeed = (signed int)(currPacket[16] + currPacket[17]*256);
        if(abs(leftMeasuredSpeed) < SPEED_THR) {
            leftMeasuredSpeed = 0;
        }
        if(abs(rightMeasuredSpeed) < SPEED_THR) {
            rightMeasuredSpeed = 0;
        }
        
        leftDistPrev = leftDist;
        rightDistPrev = rightDist;
        finalTime = [NSDate date];
        totalTime = [finalTime timeIntervalSinceDate:startTime];    // in seconds
        leftDist += ((double)leftMeasuredSpeed)*totalTime*leftDiamCoeff;
        rightDist += ((double)rightMeasuredSpeed)*totalTime*rightDiamCoeff;
        deltaDist = ((rightDist-rightDistPrev)+(leftDist-leftDistPrev))/2.0;
        odometry[X_ODOM] += cos(odometry[THETA_ODOM])*deltaDist;
        odometry[Y_ODOM] += sin(odometry[THETA_ODOM])*deltaDist;
        odometry[THETA_ODOM] = ((rightDist-leftDist)/wheelBase)/1000.0;	// over 1000 because rightDist and leftDist are in mm
        
        if(logEnabled) {
            /*
            // odometry debugging
            printf("leftDistPrev=%f, rightDistPrev=%f\n", (double)leftDistPrev, (double)rightDistPrev);
            printf("totalTime=%f\n", totalTime);
            printf("leftDiamCoeff=%f\n", leftDiamCoeff);
            printf("leftSpeed=%f, rightSpeed=%f\n", (double)leftMeasuredSpeed, (double)rightMeasuredSpeed);
            printf("leftDist=%f, rightDist=%f, deltaDist=%f\n", (double)leftDist, (double)rightDist, (double)deltaDist);
            printf("x=%f, y=%f, theta=%f\n", odometry[X_ODOM], odometry[Y_ODOM], odometry[THETA_ODOM]);
            */
            
            /*
            logString = proxValues[0] + "," + proxValues[1] + "," + proxValues[2] + "," + proxValues[3] + ",";
            logString += proxAmbientValues[0] + "," + proxAmbientValues[1] + "," + proxAmbientValues[2] + "," + proxAmbientValues[3] + ",";
            logString += groundValues[0] + "," + groundValues[1] + "," + groundValues[2] + "," + groundValues[3] + ",";
            logString += groundAmbientValues[0] + "," + groundAmbientValues[1] + "," + groundAmbientValues[2] + "," + groundAmbientValues[3] + ",";
            logString += battery + ",";
            logString += flagRobotToPhone + ",";
            logString += leftMeasuredSpeed + "," + rightMeasuredSpeed + ",";
            logString += odometry[X_ODOM] + "," + odometry[Y_ODOM] + "," + odometry[THETA_ODOM];
            appendLog(logString);
             */
        }
        
        startTime = finalTime;
        
        if((flagRobotToPhone&0x20)==0x20) {
            if((flagRobotToPhone&0x40)==0x40) {
                chargeState = CHARGED;
            } else {
                chargeState = CHARGING;
            }
        } else {
            chargeState = NOT_CHARGING;
        }
        
        if((flagRobotToPhone&0x80)==0x80) {
            odomCalibFinish = true;
        } else {
            odomCalibFinish = false;
        }
        
        if(isCalibrating) {
            isCalibratingCounter--;
            if(debug) {
                //Log.d(TAG, "isCalibratingCounter = " + isCalibratingCounter);
            }
            if(isCalibratingCounter == 0) {
                isCalibrating = false;
                [self resetOdometry];	// reset odometry when calibration is done
            }
        }
        
        commTimeout = 0;
        isConnected = true;
        
        //Notify listener of an update
        [[NSNotificationCenter defaultCenter] postNotificationName:@"WPUpdate" object:nil userInfo:nil];
        
    }
    
}

#pragma mark wheelphone library functions
- (void) setSpeedleft: (int) l right: (int) r {
    if(l < MIN_SPEED_REAL) {
        l = MIN_SPEED_REAL;
    }
    if(l > MAX_SPEED_REAL) {
        l = MAX_SPEED_REAL;
    }
    if(r < MIN_SPEED_REAL) {
        r = MIN_SPEED_REAL;
    }
    if(r > MAX_SPEED_REAL) {
        r = MAX_SPEED_REAL;
    }
    lSpeed = (int) (l/MM_S_TO_BYTE);
    rSpeed = (int) (r/MM_S_TO_BYTE);
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[0] = lSpeed;
    audioSeq[1] = rSpeed;
}

- (void) setLeftSpeed: (int) l {
    if(debug) {
        printf("setLeftSpeed = %d\n", l);
    }
    if(l < MIN_SPEED_REAL) {
        l = MIN_SPEED_REAL;
    }
    if(l > MAX_SPEED_REAL) {
        l = MAX_SPEED_REAL;
    }
    lSpeed = (int) (l/MM_S_TO_BYTE);
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[0] = lSpeed;
}

- (void) setRightSpeed: (int) r {
    if(debug) {
        printf("setRightSpeed = %d\n", r);
    }
    if(r < MIN_SPEED_REAL) {
        r = MIN_SPEED_REAL;
    }
    if(r > MAX_SPEED_REAL) {
        r = MAX_SPEED_REAL;
    }
    rSpeed = (int) (r/MM_S_TO_BYTE);
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[1] = rSpeed;
}

- (void) setRawSpeedleft: (int) l right: (int) r {
    if(l < MIN_SPEED_RAW) {
        l = MIN_SPEED_RAW;
    }
    if(l > MAX_SPEED_RAW) {
        l = MAX_SPEED_RAW;
    }
    if(r < MIN_SPEED_RAW) {
        r = MIN_SPEED_RAW;
    }
    if(r > MAX_SPEED_RAW) {
        r = MAX_SPEED_RAW;
    }
    lSpeed = l;
    rSpeed = r;
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[0] = lSpeed;
    audioSeq[1] = rSpeed;
}

- (void) setRawLeftSpeed: (int) l {
    if(l < MIN_SPEED_RAW) {
        l = MIN_SPEED_RAW;
    }
    if(l > MAX_SPEED_RAW) {
        l = MAX_SPEED_RAW;
    }
    lSpeed = l;
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[0] = lSpeed;
}

- (void) setRawRightSpeed: (int) r {
    if(r < MIN_SPEED_RAW) {
        r = MIN_SPEED_RAW;
    }
    if(r > MAX_SPEED_RAW) {
        r = MAX_SPEED_RAW;
    }
    rSpeed = r;
    avgSpeedPrev = avgSpeed;
    avgSpeed = (lSpeed + rSpeed)/2;
    rotSpeed = (lSpeed - rSpeed)/2;
    audioSeq[1] = rSpeed;
}

- (void) setFlagsPhoneToRobot: (char) value {
    flagPhoneToRobot = value;
    audioSeq[2] = flagPhoneToRobot;
}

- (void) enableSpeedControl {
#warning "Speed control is enabled by default and cannot be disabled";
    flagPhoneToRobot |= (1 << 0);
    audioSeq[2] = flagPhoneToRobot;
}

- (void) disableSpeedControl {
#warning "Speed control is enabled by default and cannot be disabled";
    flagPhoneToRobot &= ~(1 << 0);
    audioSeq[2] = flagPhoneToRobot;
}

- (void) enableSoftAcceleration {
#warning "Soft acceleration is enabled by default and cannot be disabled";
    flagPhoneToRobot |= (1 << 1);
    audioSeq[2] = flagPhoneToRobot;
}

- (void) disableSoftAcceleration {
#warning "Soft acceleration is enabled by default and cannot be disabled";
    flagPhoneToRobot &= ~(1 << 1);
    audioSeq[2] = flagPhoneToRobot;
}

// return 0 if no error, otherwise return 1
- (int) enableObstacleAvoidance {
    unsigned long startTimeMs=0, currTimeMs=0;
    flagPhoneToRobot |= (1 << 2);
    audioSeq[2] = flagPhoneToRobot;
    flagPhoneToRobotSent = 0;
    startTimeMs = [self getTimeMs];
    while(flagPhoneToRobotSent == 0) {
        printf("a\n");
        currTimeMs = [self getTimeMs];
        if((currTimeMs - startTimeMs) > 500) {
            return 1;
        }
    }
    startTimeMs = [self getTimeMs];
    flagRobotToPhoneReceived = 0;
    while(flagRobotToPhoneReceived == 0) {
        printf("b\n");
        currTimeMs = [self getTimeMs];
        if((currTimeMs - startTimeMs) > 500) {
            return 1;
        }
    }
    printf("flagRobotToPhone = %d\n", flagRobotToPhone);
    if((flagRobotToPhone&0x01)==0x01) {
        return 0;
    } else {
        return 1;
    }
}

// return 0 if no error, otherwise return 1
- (int) disableObstacleAvoidance {
    unsigned long startTimeMs=0, currTimeMs=0;
    flagPhoneToRobot &= ~(1 << 2);
    audioSeq[2] = flagPhoneToRobot;
    flagPhoneToRobotSent = 0;
    startTimeMs = [self getTimeMs];
    while(flagPhoneToRobotSent == 0) {
        currTimeMs = [self getTimeMs];
        if((currTimeMs - startTimeMs) > 500) {
            return 1;
        }
    }
    startTimeMs = [self getTimeMs];
    flagRobotToPhoneReceived = 0;
    while(flagRobotToPhoneReceived == 0) {
        currTimeMs = [self getTimeMs];
        if((currTimeMs - startTimeMs) > 500) {
            return 1;
        }
    }
    if((flagRobotToPhone&0x01)==0x01) {
        return 1;
    } else {
        return 0;
    }
}

// return 0 if no error, otherwise return 1
- (int) enableCliffAvoidance {
    flagPhoneToRobot |= (1 << 3);
    audioSeq[2] = flagPhoneToRobot;
    flagPhoneToRobotSent = 0;
    while(flagPhoneToRobotSent == 0);
    while(flagRobotToPhoneReceived == 0);
    if((flagRobotToPhone&0x02)==0x02) {
        return 0;
    } else {
        return 1;
    }
}

// return 0 if no error, otherwise return 1
- (int) disableCliffAvoidance {
    flagPhoneToRobot &= ~(1 << 3);
    audioSeq[2] = flagPhoneToRobot;
    flagPhoneToRobotSent = 0;
    while(flagPhoneToRobotSent == 0);
    while(flagRobotToPhoneReceived == 0);
    if((flagRobotToPhone&0x02)==0x02) {
        return 1;
    } else {
        return 0;
    }
}

- (void) calibrateSensors {
    
    printf("calibrate sensors\n");
    
    //[OpenALHelper playSoundNamed:@"sosumi"];
    
    int i=0;
    for(i=0; i<4; i++) {
        proxValuesCalibration[i] = proxValues[i];
        groundValuesCalibration[i] = groundValues[i];
    }
    flagPhoneToRobot |= (1 << 4);
    audioSeq[2] = flagPhoneToRobot;
    isCalibratingCounter = 2;	// the calibration lasts about 43 ms (105(adc int)*26(adc states)*16(samples for calibration)=43680 us)
    // thus wait at least two cylces (100 ms) to be sure the calibration is done
    isCalibrating = true;
}

- (int) getBatteryRaw {
    return battery;
}

- (float) getBatteryVoltage {
    return (float) (4.2*(float)((battery+763)/915));	// 915 corresponds to the adc sampled value of the battery at 4.2 volts
}														// 763 corresponds to the adc sampled value of the battery at 3.5 volts
// the "battery" variable actually contains the "sampled value - 763"

- (int) getBatteryCharge {
    return (int)(100*battery/maxBatteryValue);
}

- (BOOL) batteryIsLow {
    if(battery < 23) {	// 23/152=15%
        return true;
    } else {
        return false;
    }
}

- (char) getFlagStatus {
    return flagRobotToPhone;
}

- (BOOL) isCharging {
    if(chargeState == CHARGING) {
        return true;
    } else {
        return false;
    }
}

- (BOOL) isCharged {
    if(chargeState == CHARGED) {
        return true;
    } else {
        return false;
    }
}

- (int) getLeftSpeed {
    return leftMeasuredSpeed;
}

- (int) getRightSpeed {
    return rightMeasuredSpeed;
}

- (void) getFrontProxs: (int*) arr {
    int i=0;
    for(i=0; i<4; i++) {        
        arr[i] = proxValues[i];
    }
}

- (int) getFrontProx:(int) ind {
    if(ind>=0 && ind<=3) {
        return proxValues[ind];
    }
    return 0;
}

- (void) getFrontAmbients: (int*) arr {
    int i=0;
    for(i=0; i<4; i++) {
        arr[i] = proxAmbientValues[i];
    }
}

- (int) getFrontAmbient: (int) ind {
    if(ind>=0 && ind<=3) {
        return proxAmbientValues[ind];
    }
    return 0;
}

- (void) getFrontProxCalibrationValues: (int*) arr {
    int i=0;
    for(i=0; i<4; i++) {
        arr[i] = proxValuesCalibration[i];
    }
}

- (void) getGroundProxs: (int*) arr {
    int i=0;
    for(i=0; i<4; i++) {
        arr[i] = groundValues[i];
    }
}

- (int) getGroundProx: (int) ind {
    if(ind>=0 && ind<=3) {
        return groundValues[ind];
    }
    return 0;
}

- (void) getGroundAmbients: (int*) arr {
#warning "Ground ambient not present on audio communication";
    int i=0;
    for(i=0; i<4; i++) {
        arr[i] = 0;
    }
}

- (int) getGroundAmbient: (int) ind {
#warning "Ground ambient not present on audio communication";
    return 0;
}

- (void) getGroundProxCalibrationValues: (int*) arr {
    int i=0;
    for(i=0; i<4; i++) {
        arr[i] = groundValuesCalibration[i];
    }
}

- (BOOL) isCalibrating {
    return isCalibrating;
}

- (void) getOdometry: (double*) arr {
    int i=0;
    for(i=0; i<3; i++) {
        arr[i] = odometry[i];
    }
}

- (double) getOdometryX {
    return odometry[X_ODOM];
}

- (double) getOdometryY {
    return odometry[Y_ODOM];
}

- (double) getOdometryTheta {
    return odometry[THETA_ODOM];
}

- (void) setOdometryx: (double) x y: (double) y theta: (double) t {
    odometry[X_ODOM] = x;
    odometry[Y_ODOM] = y;
    odometry[THETA_ODOM] = t;
}

- (void) setOdometryParametersleftDiamCoeff:(double)ldc rightDiamCoeff:(double)rdc wheelBase:(double)wb{
    leftDiamCoeff = ldc;
    rightDiamCoeff = rdc;
    wheelBase = wb;
}

- (void) calibrateOdometry {
#warning "Odometry calibration not available on audio communication";
    flagPhoneToRobot |= (1 << 5);
    audioSeq[2] = flagPhoneToRobot;
    odomCalibFinish = false;
}

- (BOOL) odometryCalibrationTerminated {
#warning "Odometry calibration not available on audio communication";    
    return odomCalibFinish;
}

- (void) resetOdometry {
    [self setOdometryx: 0.0 y: 0.0 theta: 0.0];
    leftDist = 0.0;
    rightDist = 0.0;
    leftDistPrev = 0.0;
    rightDistPrev = 0.0;
}

- (void) enableDataLog {
    logEnabled = true;
    //logString = "prox0,prox1,prox2,prox3,proxAmb0,proxAmb1,proxAmb2,proxAmb3,ground0,ground1,ground2,ground3,groundAmb0,groundAmb1,groundAmb2,groundAmb3,battery,flagRobotToPhone,leftSpeed,rightSpeed,x,y,theta";
    //appendLog(logString);
}

- (void) disableDataLog {
    logEnabled = false;
}

- (void) appendLog: (NSString*) text {
    // write log to file
}

- (BOOL) isRobotConnected {
    return isConnected;
}

- (void) setCommunicationTimeout: (int) ms {
    commTimeoutLimit = ms/50;
}

- (void) setMicGain: (float) value {
    CGFloat gain = value;
    NSError* error;
    AVAudioSession *audioSession = [AVAudioSession sharedInstance];
    if (audioSession.isInputGainSettable) {
        BOOL success = [audioSession setInputGain:gain error:&error];
        if (!success){ //error handling
            NSLog(@"Error setting mic gain");
            NSLog(@"Error: %@", error);
        } else {
            //NSLog(@"Set mic gain to: %f", gain);
        }
    } else {
        NSLog(@"Cannot set input gain");
    }
}

- (unsigned long) getTimeMs {
    struct timeval time;
    gettimeofday(&time, NULL);
    return (time.tv_sec*1000) + (time.tv_usec/1000);
}

@end
