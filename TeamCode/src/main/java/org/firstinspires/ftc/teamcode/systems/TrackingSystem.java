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

	/**
	 * The front left odometry wheel.
	 */
	private final DcMotorEx frontLeft;
	/**
	 * The front right odometry wheel.
	 */
	private final DcMotorEx frontRight;
	/**
	 * The back odometry wheel.
	 */
	private final DcMotorEx back;

	// Keeps the robot's odometry wheel's previous positions
	private double flPreviousTicks = 0;
	private double frPreviousTicks = 0;
	private double bPreviousTicks = 0;

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
		opMode.telemetry.update();
	}

	/**
	 * Tracks the robot's position.
	 * Pose Exponentials Method and explanation taken from
	 * <a href="https://gm0.org/en/latest/docs/software/concepts/odometry.html#using-pose-exponentials">this site</a>.
	 */
	public void trackPosition() {
		// Get the current position of the odometry wheels
		double flCurrentTicks = frontLeft.getCurrentPosition();
		double frCurrentTicks = frontRight.getCurrentPosition();
		double bCurrentTicks = back.getCurrentPosition();

		// The displacement of each wheel
		final double frontLeftDisplacement = (flCurrentTicks - flPreviousTicks) * CM_PER_TICK;
		final double frontRightDisplacement = (frCurrentTicks - frPreviousTicks) * CM_PER_TICK;
		final double backDisplacement = (bCurrentTicks - bPreviousTicks) * CM_PER_TICK;

		// Calculating the robot's displacement and rotation
		final double angleChange = (frontLeftDisplacement - frontRightDisplacement) / LATERAL_DISTANCE;
		final double centerDisplacement = (frontLeftDisplacement + frontRightDisplacement) / 2;
		final double horizontalDisplacement = backDisplacement - FORWARD_OFFSET * angleChange;

		// Temp variable for readability
		final double angleCos = cos(position.angle), angleSin = sin(position.angle),
				angleChangeCos = cos(angleChange), angleChangeSin = sin(angleChange);

		// The angle was removed from the matrices and they were simplified to have only two rows
		// because it wasn't actually used in the calculates.
		final double[][] matrix1 = {{angleCos, -angleSin}, {angleSin, angleCos}};
		final double[][] matrix2 = {{angleChangeSin / angleChange, (angleChangeCos - 1) / angleChange},
				{(1 - angleChangeCos / angleChange), angleChangeSin / angleChange}};
		final double[] matrix3 = {centerDisplacement, horizontalDisplacement};

		// Multiplication of matrices 2 & 3
		final double[] matrix4 = {matrix2[0][0] * matrix3[0] + matrix2[0][1] * matrix3[1],
				matrix2[1][0] * matrix3[0] + matrix2[1][1] * matrix3[1]};

		// Multiplication of matrices 1 & 4
		final double[] resultMatrix = {matrix1[0][0] * matrix4[0] + matrix1[0][1] * matrix4[1],
				matrix1[1][0] * matrix4[0] + matrix1[1][1] * matrix4[1]};

		// Assignment of the new position
		position.x += resultMatrix[0];
		position.y += resultMatrix[1];
		position.angle += angleChange;

		// Update the previous ticks
		flPreviousTicks = flCurrentTicks;
		frPreviousTicks = frCurrentTicks;
		bPreviousTicks = bCurrentTicks;
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
