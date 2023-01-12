package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
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
	/**
	 * The distance between the front left and front right wheels.
	 */
	private static final double LATERAL_DISTANCE = 15; //TODO: Measure this
	/**
	 * The distance between the center of the robot and the front wheel.
	 */
	private static final double FORWARD_OFFSET = 0; //TODO: Measure this

	private final LinearOpMode opMode;
	private final SystemCoordinator systemCoordinator;

	private final DcMotorEx frontLeft;
	private final DcMotorEx frontRight;
	private final DcMotorEx back;

	private final Pose position = new Pose(0., 0., 0.);

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public TrackingSystem(LinearOpMode opMode, SystemCoordinator systemCoordinator) {
		this.opMode = opMode;
		this.systemCoordinator = systemCoordinator;

		//Get odometry pod interfaces
		frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
		frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
		back = opMode.hardwareMap.get(DcMotorEx.class, "back");

		// Makes the motors break when their power is set to zero, so they can better stop in place.
		frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		back.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		// Reset the distances measured by the motors
		resetDistance();
	}

	/**
	 * Resets the distance measured on all encoders.
	 * Should always be called before initializing the robot.
	 */
	private void resetDistance() {
		frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		back.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * @return The robot's current position as a Pose object.
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
		final double frontLeftDisplacement = position.y - frontLeft.getCurrentPosition() * CM_PER_TICK;
		final double frontRightDisplacement = position.y - frontRight.getCurrentPosition() * CM_PER_TICK;

		final double angleChange = (frontLeftDisplacement - frontRightDisplacement) / LATERAL_DISTANCE;
		final double centerDisplacement = (frontLeftDisplacement + frontRightDisplacement) / 2;
		final double horizontalDisplacement = position.y - FORWARD_OFFSET * angleChange;

		//Pose Exponentials https://gm0.org/en/latest/docs/software/concepts/odometry.html#using-pose-exponentials
		position.x += centerDisplacement * sin(angleChange) / angleChange
				+ horizontalDisplacement * (cos(angleChange) - 1) / angleChange;
		position.y += centerDisplacement * (1 - cos(angleChange)) / angleChange
				+ horizontalDisplacement * sin(angleChange) / angleChange;
		position.angle += angleChange;
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
		opMode.telemetry.update();
	}
}
