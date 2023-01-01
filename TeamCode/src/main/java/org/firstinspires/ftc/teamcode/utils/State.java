package org.firstinspires.ftc.teamcode.utils;

public interface State {
	enum Message {
		ClawDone, FourBarDone, DrivingDone, ElevatorDone
	}

	void tick();

	default void onReceiveMessage() {
	}
}
