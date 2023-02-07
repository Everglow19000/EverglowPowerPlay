package org.firstinspires.ftc.teamcode.utils.StateMachine;

/**
 * The basic interface for state creation.
 */
public interface State {
	/**
	 * Each state should be ticked in an epsilon amount of time.
	 */
	void tick();
}