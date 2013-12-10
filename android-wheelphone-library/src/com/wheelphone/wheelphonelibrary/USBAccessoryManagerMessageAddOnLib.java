package com.wheelphone.wheelphonelibrary;

import android.os.Build;

/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************/


import com.android.future.usb.UsbAccessory;

/** Basic Message class for the USBAccessoryManager.  This is used
 *  to send messages from the USB Accessory's read thread to the
 *  GUI thread to notify the GUI thread of various USB Accessory
 *  events (like data available or device attachment).
 *  
 * @author Microchip Technologies Inc.
 *
 */
public class USBAccessoryManagerMessageAddOnLib extends USBAccessoryManagerMessage {
	
	/* Any text information that needs to be sent with data */
	public String text = null;
	/* Data send in the read MessageType */
	public byte[] data = null;
	/* A USB accessory that attached */	
	public static UsbAccessory accessory = null;
	
	/** Creates new message of specified type
	 * 
	 * @param type The type of this message
	 */
	public USBAccessoryManagerMessageAddOnLib(MessageType type) {
		this.type = type;
	}
	
	/** Creates a new message of specified type with specified data
	 * 
	 * @param type The type of this message
	 * @param data The data associated with this message
	 */
	public USBAccessoryManagerMessageAddOnLib(MessageType type, byte[] data) {
		this.type = type;
		this.data = data;
	}
	
	/** Creates a new message of specified type with specified data
	 * 
	 * @param type The type of this message
	 * @param data The data associated with this message
	 * @param accessory The accessory associated with this message
	 */
	public USBAccessoryManagerMessageAddOnLib(MessageType type, byte[] data, UsbAccessory accessory) {
		this.type = type;
		this.data = data;
		this.accessory = accessory;
	}
	
	/** Creates a new message of specified type with specified data
	 * 
	 * @param type The type of this message
	 * @param accessory The accessory associated with this message
	 */
	public USBAccessoryManagerMessageAddOnLib(MessageType type, UsbAccessory accessory) {
		this.type = type;
		this.accessory = accessory;
	}
	
	public static String getVersion() {
		return accessory.getVersion();
	}
}
