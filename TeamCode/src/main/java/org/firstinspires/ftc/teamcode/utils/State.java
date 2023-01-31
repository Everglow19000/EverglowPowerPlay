package org.firstinspires.ftc.teamcode.utils;

/**
 * The basic interface for a state.
 */
public interface State {
	/**
	 * An enums containing all the different types of messages the systems can send.
	 */
	enum Message {
		CLAW_DONE, FOUR_BAR_DONE, DRIVING_DONE, ELEVATOR_DONE
	}

	/**
	 * Each state should be ticked in an epsilon amount of time.
	 */
	void tick();
}