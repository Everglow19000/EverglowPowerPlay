package org.firstinspires.ftc.teamcode.utils;

public interface State {
	enum Message {
		CLAW_DONE, FOUR_BAR_DONE, DRIVING_DONE, ELEVATOR_DONE
	}

	void tick();
}