package com.wheelphone.wheelphonelibrary;

public abstract class USBAccessoryManagerMessage {
	/* Types of messages that can be sent */
	public enum MessageType {
		READ,
		ERROR,
		CONNECTED,
		DISCONNECTED,
		READY
	};

	/* The MessageType for this message instance */
	public MessageType type;
	
	public static String getVersion() {
		return "";
	}
	
}
