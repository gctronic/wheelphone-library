package com.wheelphone.wheelphonelibrary;

/**
 * \file WheelphoneRobot.java
 * \brief Main Wheelphone class
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 29.05.13
 * \copyright GNU GPL v3


The WheelphoneRobot class is the main class that need to be instantiated in the application in order 
to communicate with the robot (receive sensors data and send commands).
The low-level communication with the robot (packets exchange) is handled internally by this class.

*/

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.Handler;
import android.os.Message;
import android.util.Log;


public class WheelphoneRobot {
	
	// USB communication
	private final static int packetLengthRecv	= 63;		// receiving packet length (this the maximum packet length, actually only 23 bytes are used)
	private final static int packetLengthSend	= 63;		// sending packet legnth (this the maximum packet length, actually only 3 bytes are used)
	private final static int USBAccessoryWhat	= 0;		// type of message received
	private static final int UPDATE_STATE		= 4;
	private static final int APP_CONNECT		= (int)0xFE;
	private static final int APP_DISCONNECT		= (int)0xFF;
	private int commTimeout = 0;							// timeout indicating that nothing is received from the robot for a while
	private int packetReceived = 1;							// flag indicating when a new packet is received from the robot; after this flag is set the next packet 
															// containing the new commands is sent to the robot (communication is synchronized)
    private boolean deviceAttached = false;					// flag indicating when the robot is attached
	private USBAccessoryManager accessoryManager;			// low-level USB communication class
	private boolean isConnected = false;					// flag indicating if the robot is connected (and exchanging packets) with the phone
	private int commTimeoutLimit = 20;						// based on communication timer task (repeatedly scheduled at 50 ms) => 300 ms
	
	// Robot state (robot => phone)
	private int[] proxValues = {0, 0, 0, 0};				// front proximity values (higher value means nearer object)	
	private int[] proxAmbientValues = {0, 0, 0, 0};			// front proximity ambient values
	private int[] proxValuesCalibration = {0, 0, 0, 0};		// front proximity values of the calibration
	private int[] groundValues = {0, 0, 0, 0};				// ground proximity values (darker surface/cliff => lower values)
	private int[] groundAmbientValues = {0, 0, 0, 0};		// ground proximity ambient values
	private int[] groundValuesCalibration = {0, 0, 0, 0};	// ground proximity values of the calibration
	public static final int maxBatteryValue = 152;
	private int battery = 0;	
	private int leftMeasuredSpeed=0, rightMeasuredSpeed=0;	// they contain the current measured motors speed in mm/s (speed measured using back EMF)
	private byte flagRobotToPhone = 0;						// bit 5: 1 => robot is charging, 0 => robot not charging
															// bit 6: 1 => robot completely charged, 0 => robot not completely charged
															// others bits not used
	private int chargeState = NOT_CHARGING;					
	private static final int NOT_CHARGING = 0;
	private static final int CHARGING = 1;
	private static final int CHARGED = 2;
	private boolean isCalibrating = false;
	private int isCalibratingCounter = 0;					// counter used to wait for the completion of the calibration
	private int firmwareVersion = 0;						// robot firmware version
	private boolean odomCalibFinish = false;
	
	// Robot control (phone => robot)
	private int lSpeed=0, rSpeed=0;
	private static final int MIN_SPEED_RAW = -127;
	private static final int MAX_SPEED_RAW = 127;
	private static final int MIN_SPEED_REAL = -350;			// 350 mm/s
	private static final int MAX_SPEED_REAL = 350;	
	private byte flagPhoneToRobot=1;						// bit 0 => controller On/Off
															// bit 1 => soft acceleration On/Off
															// bit 2 => obstacle avoidance On/Off
															// bit 3 => cliff avoidance On/Off
															// others bits not used
															
	// Various
	private static final String TAG = WheelphoneRobot.class.getName();
	private Timer timer = new Timer();						// timer used for scheduling the communication every 50 ms: this task poll a flag indicating whether a message 
															// was received, if this is the case a new command is sent to the robot and the flag is reset.
	private Context context;
	private Intent activityIntent;
	private boolean debug = false;
	private String logString;
	private static final double MM_S_TO_BYTE = 2.8;			// scale the speed given in mm/s to a byte sent to the microcontroller 
	private static final int SPEED_THR = 3;					// under this value the received measured speed is set to 0 to avoid noisy measure affecting odometry
	
	// odometry
	private double leftDiamCoeff = 1.0;
	private double rightDiamCoeff = 1.0;
	private double wheelBase = 0.087;						// meters
	private double [] odometry = {0.0, 0.0, 0.0};			// x (mm), y (mm), theta (radians) respectively
	private static final int X_ODOM = 0;
	private static final int Y_ODOM = 1;
	private static final int THETA_ODOM = 2;
	private double leftDist=0.0, rightDist=0.0;
	private double leftDistPrev=0.0, rightDistPrev=0.0;
	private double deltaDist=0.0;
	private double startTime=0.0, finalTime=0.0, totalTime=0.0;
	private boolean logEnabled = false;
	
	/*
	 * Interface that should be implemented by classes that would like to be notified by when the WheelphoneRobot state is updated. (Observer pattern)
	 */
	public interface WheelPhoneRobotListener{
		public void onWheelphoneUpdate();
	}
	private WheelPhoneRobotListener mEventListener;

	private class communicationTask extends TimerTask {          
		@Override        
		public void run() {             
			if(packetReceived==1) {
				packetReceived=0;
				sendCommandsToRobot();
				commTimeout = 0;
				if(isCalibrating) {
					isCalibratingCounter--;
					if(debug) {
						Log.d(TAG, "isCalibratingCounter = " + isCalibratingCounter);
					}
					if(isCalibratingCounter == 0) {
						isCalibrating = false;
						resetOdometry();	// reset odometry when calibration is done
					}
				}
			} else {
				commTimeout++;				
				if(commTimeout == commTimeoutLimit) {		// about "50*commTimeoutLimit" ms is passed without any answer from the robot					
					closeUSBCommunication();				// disconnect because probably the robot was turned off
				}
			}       
		}    
	}; 		
	
	// Handler for receiving messages from the USB Manager thread
    private Handler handler = new Handler() {
    	
    	@Override
    	public void handleMessage(Message msg) {
    		
    		byte[] commandPacket = new byte[packetLengthRecv];
    		byte[] commandPacket2 = new byte[2];
    		
			switch(msg.what) {			
			
				case USBAccessoryWhat:
					
					switch(((USBAccessoryManagerMessage)msg.obj).type) {
						case READ:
							if(debug) {
								Log.d(TAG, "message: READ");
							}
							if(accessoryManager.isConnected() == false) {
								return;
							}
					
							while(true) {

								if(accessoryManager.available() < packetLengthRecv) {							
									break;
								}
						
								accessoryManager.read(commandPacket);
												
								switch(commandPacket[0]) {
			    			
									case UPDATE_STATE:		
										proxValues[0] = 0x00<<24 | commandPacket[1]&0xFF;	// to get unsigned int
										//if(accessoryManager instanceof USBAccessoryManagerAddOnLib) {
										//	proxValues[0] = 1;
										//} else if(accessoryManager instanceof USBAccessoryManagerAndroidLib) {
										//	proxValues[0] = 2;
										//}
										proxValues[1] = 0x00<<24 | commandPacket[2]&0xFF;
										proxValues[2] = 0x00<<24 | commandPacket[3]&0xFF;
										proxValues[3] = 0x00<<24 | commandPacket[4]&0xFF;
										proxAmbientValues[0] = 0x00<<24 | commandPacket[5]&0xFF;
										proxAmbientValues[1] = 0x00<<24 | commandPacket[6]&0xFF;
										proxAmbientValues[2] = 0x00<<24 | commandPacket[7]&0xFF;
										proxAmbientValues[3] = 0x00<<24 | commandPacket[8]&0xFF;
										groundValues[0] = 0x00<<24 | commandPacket[9]&0xFF;
										groundValues[1] = 0x00<<24 | commandPacket[10]&0xFF;
										groundValues[2] = 0x00<<24 | commandPacket[11]&0xFF;
										groundValues[3] = 0x00<<24 | commandPacket[12]&0xFF;
										groundAmbientValues[0] = 0x00<<24 | commandPacket[13]&0xFF;
										groundAmbientValues[1] = 0x00<<24 | commandPacket[14]&0xFF;
										groundAmbientValues[2] = 0x00<<24 | commandPacket[15]&0xFF;
										groundAmbientValues[3] = 0x00<<24 | commandPacket[16]&0xFF;								
										battery = 0x00<<24 | commandPacket[17]&0xFF;
										flagRobotToPhone = commandPacket[18]; 										
										leftMeasuredSpeed = (commandPacket[19]&0xFF) + (commandPacket[20])*256;
										rightMeasuredSpeed = (commandPacket[21]&0xFF) + (commandPacket[22])*256;										
										if(Math.abs(leftMeasuredSpeed) < SPEED_THR) {
											leftMeasuredSpeed = 0;
										}
										if(Math.abs(rightMeasuredSpeed) < SPEED_THR) {
											rightMeasuredSpeed = 0;
										}
										
										leftDistPrev = leftDist;
										rightDistPrev = rightDist;
										finalTime = System.currentTimeMillis();
										totalTime = finalTime - startTime;
										leftDist += (leftMeasuredSpeed*totalTime/1000.0)*leftDiamCoeff;
										rightDist += (rightMeasuredSpeed*totalTime/1000.0)*rightDiamCoeff;											
										deltaDist = ((rightDist-rightDistPrev)+(leftDist-leftDistPrev))/2.0;
										odometry[X_ODOM] += Math.cos(odometry[THETA_ODOM])*deltaDist;				
										odometry[Y_ODOM] += Math.sin(odometry[THETA_ODOM])*deltaDist;
										odometry[THETA_ODOM] = ((rightDist-leftDist)/wheelBase)/1000.0;	// over 1000 because rightDist and leftDist are in mm									    	  								    	
								    			
										if(logEnabled) {
											logString = proxValues[0] + "," + proxValues[1] + "," + proxValues[2] + "," + proxValues[3] + ",";
											logString += proxAmbientValues[0] + "," + proxAmbientValues[1] + "," + proxAmbientValues[2] + "," + proxAmbientValues[3] + ",";
											logString += groundValues[0] + "," + groundValues[1] + "," + groundValues[2] + "," + groundValues[3] + ",";
											logString += groundAmbientValues[0] + "," + groundAmbientValues[1] + "," + groundAmbientValues[2] + "," + groundAmbientValues[3] + ",";
											logString += battery + ",";
											logString += flagRobotToPhone + ",";
											logString += leftMeasuredSpeed + "," + rightMeasuredSpeed + ",";
											logString += odometry[X_ODOM] + "," + odometry[Y_ODOM] + "," + odometry[THETA_ODOM];
											appendLog(logString);
										}
										
								    	if(debug) {
								    		//logString = lSpeed + "," + rSpeed + "," + leftMeasuredSpeed + "," + rightMeasuredSpeed + "," + leftDistPrev + "," + rightDistPrev + "," + leftDist + "," + rightDist + "," + startTime + "," + finalTime + "," + totalTime + "," + odometry[X_ODOM] + "," + odometry[Y_ODOM] + "," + odometry[THETA_ODOM] + "\n";		
								    		//logString = proxValues[0] + "," + proxValues[1] + "," + proxValues[2] + "," + proxValues[3] + "," + proxValues[1] + "," + groundValues[0] + "," + groundValues[1] + "," + groundValues[2] + "," + groundValues[3] + "," + battery + "," + leftMeasuredSpeed + "," + rightMeasuredSpeed + "\n";
								    		int j=0;
								    		//for(j=0; j<(commandPacket[57]&0xFF); j++) {
								    		for(j=0; j<7; j++) {	
								    			//debugControllerValues[j] = (commandPacket[21]&0xFF) + (commandPacket[22])*256;
//								    			logString = ((commandPacket[j*8+1]&0xFF) + (commandPacket[j*8+2])*256) + ",";
//								    			logString += ((commandPacket[j*8+3]&0xFF) + (commandPacket[j*8+4])*256) + ",";
//								    			logString += ((commandPacket[j*8+5]&0xFF) + (commandPacket[j*8+6])*256) + ",";
//								    			logString += ((commandPacket[j*8+7]&0xFF) + (commandPacket[j*8+8])*256) + ",";
//								    			logString += (commandPacket[57]&0xFF);
//								    			appendLog(logString);
								    		}
								    										    		
							    			logString = (((commandPacket[1]&0xFF) + (commandPacket[2])*256)&0x0000FFF) + ",";	// unsigned int
							    			logString += (commandPacket[3]&0xFF) + (commandPacket[4])*256 + ",";
							    			logString += (commandPacket[5]&0xFF) + (commandPacket[6])*256 + ",";
							    			logString += (commandPacket[7]&0xFF) + (commandPacket[8])*256 + ",";
							    			logString += (commandPacket[9]&0xFF) + (commandPacket[10])*256 + ",";
							    			logString += (commandPacket[11]&0xFF) + ",";
							    			logString += (commandPacket[12]&0xFF) + (commandPacket[13])*256 + ",";
							    			logString += (commandPacket[14]&0xFF) + (commandPacket[15])*256 + ",";
							    			logString += (commandPacket[16]&0xFF) + (commandPacket[17])*256 + ",";
							    			logString += (commandPacket[18]&0xFF) + (commandPacket[19])*256 + ",";
							    			logString += (commandPacket[20]&0xFF) + (commandPacket[21])*256;
							    			appendLog(logString);
								    		
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
										if(mEventListener!=null) {
											mEventListener.onWheelphoneUpdate(); //Notify listener of an update
										}

										break;
								}

								packetReceived=1;
							}	// while
							
							break;
							
						case CONNECTED:
							if(debug) {
								Log.d(TAG, "message: CONNECTED");
							}
							break;
							
						case READY:
							if(debug) {
								Log.d(TAG, "message: READY");
							}
					    	packetReceived = 1;
					        timer = new Timer();                                         
					        timer.schedule(new communicationTask(), 0, 50);
							isConnected = true;										
					    	
							String version = accessoryManager.getVersion();							
							firmwareVersion = getFirmwareVersion(version);
							
					        if(debug) {
					        	Log.d(TAG, "usb version = " + version);
					        	Log.d(TAG, "firmware version = " + firmwareVersion);
					        }
					        
							switch(firmwareVersion){
								case 2:
								case 3:
									deviceAttached = true;
									commandPacket2[0] = (byte) APP_CONNECT;
									commandPacket2[1] = 0;
									if(debug) {
										Log.d(TAG,"sending connect message.");
									}
									accessoryManager.write(commandPacket2);
									if(debug) {
										Log.d(TAG,"connect message sent.");
									}
									break;
								case 4:	// next protocol version...
									break;
								default:
									break;
							}
							break;
							
						case DISCONNECTED:
							if(debug) {
								Log.d(TAG, "message: DISCONNECTED");
							}
							endUSBCommunication();
							if(mEventListener!=null) {
								mEventListener.onWheelphoneUpdate(); //Notify listener of a disconnection
							}							
							break;
					}				
				
					break;
					
				default:
					break;
					
			}	//switch msg.what
			
    	} //handleMessage
    	
    }; //handler	
	
    /**
     * \brief Class constructor
     * \param a pass the main activity instance (this)
     * \return WheelphoneRobot instance
     */
	public WheelphoneRobot(Context c, Intent i) {
		context = c;
		activityIntent = i;
		if(debug) {
	    	//logString = "lSpeed,rSpeed,leftMeasuredSpeed,rightMeasuredSpeed,leftDistPrev,rightDistPrev,leftDist,rightDist,startTime,finalTime,totalTime,odom_x,odom_y,odom_theta\n";
			//logString = "desired,measured,errorSum,controllerOut,counter\n";
			//appendLog(logString);
		}
	}
	
    private int getFirmwareVersion(String version) {    	
    	String major = "0";    	
    	int positionOfDot;    	
    	positionOfDot = version.indexOf('.');
    	if(positionOfDot != -1) {
    		major = version.substring(0, positionOfDot);
    	}    	
    	return Integer.valueOf(major);
    }    
    
    /**
     * \brief Send the next packet to the robot containing the last left and right speeds and flag data.
     * \return none
     */
    private void sendCommandsToRobot() {
    	if(accessoryManager.isConnected() == false) {
    		return;
		}
		byte[] commandPacket = new byte[packetLengthSend];
		commandPacket[0] = (byte) UPDATE_STATE;
		commandPacket[1] = (byte)lSpeed;
		commandPacket[2] = (byte)rSpeed;
		commandPacket[3] = flagPhoneToRobot;
		accessoryManager.write(commandPacket);	
		flagPhoneToRobot &= 0xEF; //~(1 << 4);	// calibration flag sent only once
		flagPhoneToRobot &= ~(1 << 5);
    }    
    
    /**
     * \brief To be inserted into the "onStart" function of the main activity class.
     * \return none
     */
    public void startUSBCommunication() {
    	if(debug) {
    		Log.d(TAG, "startUSBCommunication");
    	}
    	   
    	if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB_MR1) {
    		accessoryManager = new USBAccessoryManagerAndroidLib(handler, USBAccessoryWhat);
    	} else {
    		accessoryManager = new USBAccessoryManagerAddOnLib(handler, USBAccessoryWhat);
    	}
   	
    }
    
    /**
     * \brief To be inserted into the "onResume" function of the main activity class.
     * \return none
     */
    public void resumeUSBCommunication() {
    	if(debug) {
    		Log.d(TAG, "resumeUSBCommunication");
    	}
		accessoryManager.enable(context, activityIntent);
    }
    
    /**
     * \brief To be inserted into the "onPause" function of the main activity class.
     * \return none
     */
    public void pauseUSBCommunication() {
    	if(debug) {
    		Log.d(TAG, "pauseUSBCommunication");
    	}
	    switch(firmwareVersion) {
	    	case 2:
	    	case 3:
	    		byte[] commandPacket = new byte[2];
	    		commandPacket[0] = (byte) APP_DISCONNECT;
	    		commandPacket[1] = 0;
	    		accessoryManager.write(commandPacket);	
	    		break;
	    }
    
		try {
			while(accessoryManager.isClosed() == false) {
				Thread.sleep(2000);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		accessoryManager.disable(context);
		endUSBCommunication();    
		
    }
    
    
	private void closeUSBCommunication() {
		if(debug) {
			Log.d(TAG, "closeUSBCommunication");
		}
    	accessoryManager.disable(context);
    	endUSBCommunication();
	}
	
	private void endUSBCommunication() {
		if(debug) {
			Log.d(TAG, "endUSBCommunication");
		}
		timer.cancel();
		isConnected = false;
    	if(deviceAttached == false) {
    		return;
    	}    	
    	if(debug) {
    		Log.d(TAG,"endUSBCommunication()");
    	}    	
    }	
	
    /**
     * \brief Set the new left and right speeds for the robot. The new data
     *  will be actually sent to the robot when "sendCommandsToRobot" is 
     * called the next time within the timer communication task (50 ms cadence). This means that the robot speed will be updated 
     * after at most 50 ms (if the task isn't delayed by the system).
     * \param l left speed given in mm/s
     * \param r right speed given in mm/s
     * \return none
     */
	public void setSpeed(int l, int r) {	// speed given in mm/s
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
	}
	
    /**
     * \brief Set the new left speed for the robot. For more details refer to "setSpeed".
     * \param l left speed given in mm/s
     * \return none
     */
	public void setLeftSpeed(int l) {	// speed given in mm/s
		if(l < MIN_SPEED_REAL) {
			l = MIN_SPEED_REAL;
		}
		if(l > MAX_SPEED_REAL) {
			l = MAX_SPEED_REAL;
		}		
		lSpeed = (int) (l/MM_S_TO_BYTE);
	}	
	
    /**
     * \brief Set the new right speed for the robot. For more details refer to "setSpeed".
     * \param r right speed given in mm/s
     * \return none
     */	
	public void setRightSpeed(int r) {	// speed given in mm/s
		if(r < MIN_SPEED_REAL) {
			r = MIN_SPEED_REAL;
		}
		if(r > MAX_SPEED_REAL) {
			r = MAX_SPEED_REAL;
		}		
		rSpeed = (int) (r/MM_S_TO_BYTE);
	}	
	
    /**
     * \brief Set the new left and right speeds for the robot. For more details refer to "setSpeed".
     * \param l left speed (range is from -127 to 127)
     * \param r right speed (range is from -127 to 127)
     * \return none
     */
	public void setRawSpeed(int l, int r) {
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
	}
	
    /**
     * \brief Set the new left speed for the robot. For more details refer to "setSpeed".
     * \param l left speed (range is from -127 to 127)
     * \return none
     */
	public void setRawLeftSpeed(int l) {
		if(l < MIN_SPEED_RAW) {
			l = MIN_SPEED_RAW;
		}
		if(l > MAX_SPEED_RAW) {
			l = MAX_SPEED_RAW;
		}		
		lSpeed = l;	
	}
	
    /**
     * \brief Set the new right speed for the robot. For more details refer to "setSpeed".
     * \param r right speed (range is from -127 to 127)
     * \return none
     */	
	public void setRawRightSpeed(int r) {
		if(r < MIN_SPEED_RAW) {
			r = MIN_SPEED_RAW;
		}
		if(r > MAX_SPEED_RAW) {
			r = MAX_SPEED_RAW;
		}		
		rSpeed = r;		
	}
	
    /**
     * \brief Flags setting directly with a byte value.
     * \param value representing the flags to enable (1) or disable (0):
     * - bit1: speed control enable/disable
     * - bit2: soft acceleration enable/disable
     * - bit3: obstacle avoidance enable/disable
     * - bit4: cliff avoidance enable/disable
     * - bit5: calibrate sensors
     * - bit6: calibrate odometry
     * - bit7/8: not used
     * \return none
     */	
	public void setFlagsPhoneToRobot(byte value) {
		flagPhoneToRobot = value;
	}
	
    /**
     * \brief Enable speed control on the robot (controller based on speed measure with back-emf).
     * \return none
     */	
	public void enableSpeedControl() {
		flagPhoneToRobot |= (1 << 0);
	}
	
    /**
     * \brief Disable speed control on the robot.
     * \return none
     */	
	public void disableSpeedControl() {
		flagPhoneToRobot &= ~(1 << 0);
	}
	
    /**
     * \brief Enable soft acceleration on the robot; this is useful when the robot is started fast (from standstill) 
     * because it avoid the robot to wheelie.
     * \return none
     */	
	public void enableSoftAcceleration() {
		flagPhoneToRobot |= (1 << 1);
	}
	
    /**
     * \brief Disable soft acceleration on the robot.
     * \return none
     */
	public void disableSoftAcceleration() {
		flagPhoneToRobot &= ~(1 << 1);
	}	
	
    /**
     * \brief Enable obstacle avoidance onboard.
     * \return none
     */
	public void enableObstacleAvoidance() {
		flagPhoneToRobot |= (1 << 2);
	}
	
    /**
     * \brief Disable obstacle avoidance onboard.
     * \return none
     */
	public void disableObstacleAvoidance() {
		flagPhoneToRobot &= ~(1 << 2);
	}	
	
    /**
     * \brief Enable cliff avoidance onboard; when a cliff is detected the robot is stopped until this flag is reset.
     * \return none
     */
	public void enableCliffAvoidance() {
		flagPhoneToRobot |= (1 << 3);
	}
	
    /**
     * \brief Disable cliff avoidance onboard.
     * \return none
     */
	public void disableCliffAvoidance() {
		flagPhoneToRobot &= ~(1 << 3);
	}
	
    /**
     * \brief Start the calibration of all the sensors. Use "isCalibrating" to know when the calibration is done.
     * \return none
     */
    public void calibrateSensors() {
    	int i=0;
    	for(i=0; i<4; i++) {
    		proxValuesCalibration[i] = proxValues[i];
    		groundValuesCalibration[i] = groundValues[i];
    	}
    	flagPhoneToRobot |= (1 << 4);
    	isCalibratingCounter = 2;	// the calibration lasts about 43 ms (105(adc int)*26(adc states)*16(samples for calibration)=43680 us)
    								// thus wait at least two cylces (100 ms) to be sure the calibration is done
    	isCalibrating = true;    	
    }
    
    /**
     * \brief Returns the sampled value of the battery.
     * \return battery level (from 0 to maxBatteryValue=152)
     */
    public int getBatteryRaw() {
    	return battery;
    }
    
    /**
     * \brief Returns the current battery voltage.
     * \return battery voltage (from 3.5 to 4.2 volts)
     */
    public float getBatteryVoltage() {
    	return (float) (4.2*(float)((battery+763)/915));	// 915 corresponds to the adc sampled value of the battery at 4.2 volts
    }														// 763 corresponds to the adc sampled value of the battery at 3.5 volts
    														// the "battery" variable actually contains the "sampled value - 763"
        
    /**
     * \brief Returns the percentage of charge remaining in the battery.
     * \return battery charge (from 0% to 100%)
     */
    public int getBatteryCharge() {
    	return (int)(100*battery/maxBatteryValue);
    }    

    /**
     * \brief Returns the battery charge low status.
     * \return true if low, false otherwise
     */    
    public boolean batteryIsLow() {
    	if(battery < 23) {	// 23/152=15%
    		return true;
    	} else {
    		return false;
    	}
    }
    
    /**
     * \brief Returns the flag byte that the robot set/clear itself.
     * \return flag byte:
     * - bit 5: 1 => robot is charging, 0 => robot not charging
     * - bit 6: 1 => robot completely charged, 0 => robot not completely charged
     * - others bits not used
     */
    public byte getFlagStatus() {
    	return flagRobotToPhone;
    }
	
    /**
     * \brief Returns the charging status.
     * \return true if charging, false otherwise
     */
    public boolean isCharging() {
    	if(chargeState == CHARGING) {
    		return true;
    	} else {
    		return false;
    	}
    }
    
    /**
     * \brief Returns the battery charged status.
     * \return true if battery charged, false otherwise
     */
    public boolean isCharged() {
    	if(chargeState == CHARGED) {
    		return true;
    	} else {
    		return false;
    	}
    }
    
    /**
     * \brief The value of the left motor speed returned from the robot. This value is the measured speed using back EMF.
     * \return left speed (positive or negative)
     */
    public int getLeftSpeed() {
    	return leftMeasuredSpeed;
    }
    
    /**
     * \brief The value of the right motor speed returned from the robot. For more details refer to "getLeftSpeed".
     * \return right speed (positive or negative)
     */
    public int getRightSpeed() {
    	return rightMeasuredSpeed;
    }
    
    /**
     * \brief The robot has 4 front proximity sensors positioned as follows:
     * \verbatim
     			1	2
     		0			3
       \endverbatim
     * The higher the value the nearer the object in front of the sensor.
     * \return array of size 4 containing the sensors values
     */
    public int[] getFrontProxs() {
    	return proxValues;
    }
    
    /**
     * \brief Return the corresponding front proximity sensor value. For more details refer to "getFrontProxs".
     * \return sensor value
     */
    public int getFrontProx(int ind) {
    	if(ind>=0 && ind<=3) {
    		return proxValues[ind];
    	}
    	return 0;
    }
    
    /**
     * \brief The robot has 4 front ambient sensors, actually they are the front proximity sensors that can measure also 
     *  the ambient light. The higher the value the lighter the environment.
     * \return array of size 4 containing the sensors values
     */
    public int[] getFrontAmbients() {
    	return proxAmbientValues;
    }
    
    /**
     * \brief Return the corresponding front ambient sensor value. For more details refer to "getFrontAmbients".
     * \return sensor value
     */
    public int getFrontAmbient(int ind) {
    	if(ind>=0 && ind<=3) {
    		return proxAmbientValues[ind];
    	}
    	return 0;
    }    
    
    /**
     * \brief When a calibration is done with "calibrateSensors" then the calibrated values are internally stored; these values 
     *  can be obtained with this function.
     * \return array of size 4 containing the sensors calibration values
     */
    public int[] getFrontProxCalibrationValues() {
    	return proxValuesCalibration;
    }    
    
    /**
     * \brief The robot has 4 ground proximity sensors positioned as follows:
     * \verbatim
     			1	2
     		0			3
       \endverbatim
     * The higher the value the darker the object in front of the sensor.
     * \return array of size 4 containing the sensors values
     */
    public int[] getGroundProxs() {
    	return groundValues;
    }
    
    /**
     * \brief Return the corresponding ground proximity sensor value. For more details refer to "getGroundProxs".
     * \return sensor value
     */
    public int getGroundProx(int ind) {
    	if(ind>=0 && ind<=3) {
    		return groundValues[ind];
    	}
    	return 0;
    }
    
    /**
     * \brief The robot has 4 ground ambient sensors, actually they are the ground proximity sensors that can measure also 
     *  the ambient light. The higher the value the lighter the environment.
     * \return array of size 4 containing the sensors values
     */
    public int[] getGroundAmbients() {
    	return groundAmbientValues;
    }
    
    /**
     * \brief Return the corresponding ground ambient sensor value. For more details refer to "getGroundAmbients".
     * \return sensor value
     */
    public int getGroundAmbient(int ind) {
    	if(ind>=0 && ind<=3) {
    		return groundAmbientValues[ind];
    	}
    	return 0;
    }   
    
    /**
     * \brief When a calibration is done with "calibrateSensors" then the calibrated values are internally stored; these values 
     *  can be obtained with this function.
     * \return array of size 4 containing the sensors calibration values
     */
    public int[] getGroundProxCalibrationValues() {
    	return groundValuesCalibration;
    }
    
    /**
    * \brief Indicate whether the robot is connected (and exchanging packets) with the phone or not.
    * \return true (if robot connected), false otherwise
    */
    public boolean isUSBConnected() {
    	return isConnected;
    }
    
    /**
    * \brief This timeout sets how much to wait for a response from the robot before changing to a disconnected state.
    * \param timeout in milliseconds
    * \return none
    */
    public void setUSBCommunicationTimeout(int ms) {
    	commTimeoutLimit = ms/50;
    }
    
    /**
    * \brief Tell whether the calibration is still in progress or not.
    * \return true (calibration in progress), false otherwise
    */
    public boolean isCalibrating() {
    	return isCalibrating;
    }
    
    /**
    * \brief Return the odometry information resulting from the measured speeds values received by the robot.
    * The positive x axis is pointing forward and the positive y axis is pointing to the left side of the robot.
    * \return array of length 3 containing sequentially x position (mm), y position (mm), theta (radians).
    */
    public double[] getOdometry() {
    	return odometry;
    }  
    
    /**
    * \brief Return the x absolute position in mm. For more information refer to "getOdometry".
    * \return x position (mm)
    */
    public double getOdometryX() {
    	return odometry[X_ODOM];
    }
    
    /**
    * \brief Return the y absolute position in mm. For more information refer to "getOdometry".
    * \return y position (mm)
    */
    public double getOdometryY() {
    	return odometry[Y_ODOM];
    }
    
    /**
    * \brief Return the theta absolute angle in radians. For more information refer to "getOdometry".
    * \return theta (radians)
    */
    public double getOdometryTheta() {
    	return odometry[THETA_ODOM];
    }
    
    /**
    * \brief Set/reset odometry components. For more information refer to "getOdometry".
    * \param x x position (mm)
    * \param y y position (mm)
    * \param t theta angle (radians)
    * \return none
    */
    public void setOdometry(double x, double y, double t) {
    	odometry[X_ODOM] = x;
    	odometry[Y_ODOM] = y;
    	odometry[THETA_ODOM] = t;
    }
    
    /**
    * \brief Set/reset odometry parameters.
    * \param dl left wheel diameter coefficient
    * \param dr right wheel diameter coefficient
    * \param wb wheels distance (m)
    * \return none
    */
    public void setOdometryParameters(double ldc, double rdc, double wb) {
    	leftDiamCoeff = ldc;
    	rightDiamCoeff = rdc;
    	wheelBase = wb;
    }
 
    public void calibrateOdometry() {
    	flagPhoneToRobot |= (1 << 5); 	
    	odomCalibFinish = false;
    }
    
    public boolean odometryCalibrationTerminated() {
    	return odomCalibFinish;
    }
    
    public void resetOdometry() {
    	setOdometry(0,0,0);
    	leftDist = 0;
    	rightDist = 0;
    	leftDistPrev = 0;
    	rightDistPrev = 0;
    }
    
    /**
    * \brief Return version of the firmware running on the robot. This is useful to know whether an update is available or not.
    * \return firmware version
    */
    public int getFirmwareVersion() {
    	return firmwareVersion;
    }
    
    /**
    * \brief Enable the logging of the sensors data received from the robot and the computed odometry.
    */
    public void enableDataLog() {
    	logEnabled = true;
		logString = "prox0,prox1,prox2,prox3,proxAmb0,proxAmb1,proxAmb2,proxAmb3,ground0,ground1,ground2,ground3,groundAmb0,groundAmb1,groundAmb2,groundAmb3,battery,flagRobotToPhone,leftSpeed,rightSpeed,x,y,theta";
		appendLog(logString);
    }
    
    
    /**
    * \brief Disable the logging.
    */
    public void disableDataLog() {
    	logEnabled = false;
    }
    
	void appendLog(String text)
	{       
	   File logFile = new File("sdcard/logFile.csv");
	   if (!logFile.exists())
	   {
	      try
	      {
	         logFile.createNewFile();
	      } 
	      catch (IOException e)
	      {
	         // TODO Auto-generated catch block
	         e.printStackTrace();
	      }
	   }
	   try
	   {
	      //BufferedWriter for performance, true to set append to file flag
	      BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true)); 
	      buf.append(text);
	      buf.newLine(); 
	      buf.close();
	   }
	   catch (IOException e)
	   {
	      // TODO Auto-generated catch block
	      e.printStackTrace();
	   }
	}
	
	/*
	 * Observer pattern glue code:
	 */
	public void setWheelPhoneRobotListener(WheelPhoneRobotListener eventListener) {
		mEventListener = eventListener;
	}

	public void removeWheelPhoneRobotListener() {
		mEventListener = null;
	}

    
}
