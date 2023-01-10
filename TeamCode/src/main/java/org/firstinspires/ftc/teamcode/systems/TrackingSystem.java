package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Pose;

/**
 * A class for handling tracking the robot's position.
 */
public class TrackingSystem {
	private static final double WHEEL_RADIUS_CM = 4.8;
	private static final double TICKS_PER_ROTATION = 515;
	private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS_CM * 2 * PI;

	private final LinearOpMode opMode;
	private final SystemCoordinator systemCoordinator;

	private final DcMotorEx en1;
	private final DcMotorEx en2;
	private final DcMotorEx en3;

	private final Pose position = new Pose(0., 0., 0.);

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public TrackingSystem(LinearOpMode opMode, SystemCoordinator systemCoordinator) {
		this.opMode = opMode;
		this.systemCoordinator = systemCoordinator;

		//Get odometry pod interfaces
		en1 = opMode.hardwareMap.get(DcMotorEx.class, "en1");
		en2 = opMode.hardwareMap.get(DcMotorEx.class, "en2");
		en3 = opMode.hardwareMap.get(DcMotorEx.class, "en3");

		// Makes the motors break when their power is set to zero, so they can better stop in place.
		en1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		en2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		en3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		// Reset the distances measured by the motors
		resetDistance();
	}

	/**
	 * Resets the distance measured on all encoders.
	 * Should always be called before initializing the robot.
	 */
	private void resetDistance() {
		en1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		en2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		en3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		en1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		en2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		en3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * @return The robot's current position.
	 */
	public Pose getPosition() {
		return position;
	}

	/**
	 * Ticks the driving system.
	 */
	public void tick() {
		trackPosition();
		printPosition();
	}

	/**
	 * Tracks the robot's position.
	 */
	public void trackPosition() {
		//TODO: TRACK THE ROBOT
		position.x = 0;
		position.y = 0;
		position.angle = 0;
	}

	/**
	 * Prints the robot's current position to the telemetry.
	 * Must call telemetry.update() after using this method.
	 */
	public void printPosition() {
		opMode.telemetry.addData("x", position.x);
		opMode.telemetry.addData("y", position.y);
		opMode.telemetry.addData("rot", toDegrees(position.angle));
		//double cycleFrequency = 1e9 / lastCycleDuration;
		//opMode.telemetry.addData("Cycle Frequency [Hz]: ", cycleFrequency);
	}
}
