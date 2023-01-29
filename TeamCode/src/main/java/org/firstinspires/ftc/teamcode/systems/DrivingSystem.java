package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static org.firstinspires.ftc.teamcode.utils.Utils.normalizeAngle;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;
import org.firstinspires.ftc.teamcode.utils.Sequence;

/**
 * A class for handling moving the robot through space.
 */
public class DrivingSystem {
	private final double EPSILON = 1;
	/**
	 * The current opMode running on the robot.
	 */
	private final LinearOpMode opMode;

	/**
	 * The front left wheel.
	 */
	private final DcMotor frontLeft;
	/**
	 * The front right wheel.
	 */
	private final DcMotor frontRight;
	/**
	 * The back left wheel.
	 */
	private final DcMotor backLeft;
	/**
	 * The back right wheel.
	 */
	private final DcMotor backRight;

	/**
	 * The current state of the the DrivingSystem.
	 * Can be either RestingState or ActingState
	 * (temporarily also DriveStraightState and DriveSidewaysState).
	 */
	private State state = new RestingState();

	public class DriveStraightState implements State {
		private static final double ANGLE_DEVIATION_SCALAR = 0.05 * 180 / Math.PI;

		private final double targetY;
		private final double targetAngle;

		public DriveStraightState(double targetY, double targetAngle) {
			this.targetY = targetY;
			this.targetAngle = targetAngle;

		}

		@Override
		public void tick() {
			Pose position = SystemCoordinator.instance.trackingSystem.getPosition();
			double yDeviation = targetY - position.y;
			double angleDeviation = normalizeAngle(targetAngle - position.angle);
			double yPower = 0.03 * yDeviation + 0.15 * signum(yDeviation);
			double rotPower = ANGLE_DEVIATION_SCALAR * angleDeviation;
			if (abs(yDeviation) < EPSILON) {
				stop();
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.DRIVING_DONE);
			} else {
				driveMecanum(new Pose(0, yPower, rotPower));
			}
		}
	}

	public class DriveSidewaysState implements State {
		private static final double ANGLE_DEVIATION_SCALAR = 0.05 * 180 / Math.PI;

		private final double targetX;
		private final double targetAngle;

		public DriveSidewaysState(double targetX, double targetAngle) {
			this.targetX = targetX;
			this.targetAngle = targetAngle;

		}

		@Override
		public void tick() {
			Pose position = SystemCoordinator.instance.trackingSystem.getPosition();
			double xDeviation = targetX - position.x;
			double angleDeviation = normalizeAngle(targetAngle - position.angle);
			double xPower = 0.03 * xDeviation + 0.15 * signum(xDeviation);
			double rotPower = ANGLE_DEVIATION_SCALAR * angleDeviation;
			if (abs(xDeviation) < EPSILON) {
				stop();
				state = new RestingState();
				SystemCoordinator.instance.sendMessage(Message.DRIVING_DONE);
			} else {
				driveMecanum(new Pose(xPower, 0, rotPower));
			}

		}
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public DrivingSystem(LinearOpMode opMode) {
		this.opMode = opMode;

		// Get mecanum wheels interfaces
		frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
		frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
		backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
		backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");


		// Some motors are wired in reverse, so we must reverse them back.
		frontLeft.setDirection(DcMotor.Direction.REVERSE);
		backLeft.setDirection(DcMotor.Direction.REVERSE);

		// Reset the distances measured by the motors
		resetDistance();
	}

	/**
	 * Resets the distance measured on all encoders,
	 * should always be called before initializing the robot.
	 */
	private void resetDistance() {
		frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * Drives the robot and keeps track of it's position.
	 * Gets called multiple times per second.
	 *
	 * @param powers Velocity vector containing elements: x, y, and an azimuth angle.
	 *               x is this vertical power, positive is left, negative is right.
	 *               y is the horizontal power, positive is forward, negative is backwards.
	 *               angle rotational power, positive is counter clockwise, negative is clockwise.
	 *               -1 <= x, y, angle <= 1.
	 */
	public void driveMecanum(Pose powers) {
		// makes the motors float if their power is set to zero when driving.
		// calling this once each cycle shouldn't be a problem, as it seems the library keeps track of the previous value, and only updates if needed.
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Determine how much power each motor should receive.
		double frontRightPower = powers.y + powers.x + powers.angle;
		double frontLeftPower = powers.y - powers.x - powers.angle;
		double backRightPower = powers.y - powers.x + powers.angle;
		double backLeftPower = powers.y + powers.x - powers.angle;

		// The method motor.setPower() only accepts numbers between -1 and 1.
		// If any number that we want to give it is greater than 1,
		// we must divide all the numbers equally so the maximum is 1
		// and the proportions are preserved.
		double norm = max(max(abs(frontRightPower), abs(frontLeftPower)), max(abs(backRightPower), abs(backLeftPower)));
		if (norm > 1) {
			frontRightPower /= norm;
			frontLeftPower /= norm;
			backRightPower /= norm;
			backLeftPower /= norm;
		}

		frontRight.setPower(frontRightPower);
		frontLeft.setPower(frontLeftPower);
		backRight.setPower(backRightPower);
		backLeft.setPower(backLeftPower);
	}

	/**
	 * Makes the robot stop in place.
	 */
	private void stop() {
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRight.setPower(0);
		frontLeft.setPower(0);
		backRight.setPower(0);
		backLeft.setPower(0);
	}

	/**
	 * Ticks the driving system.
	 */
	public void tick() {
		state.tick();
	}

	public Sequence.SequenceItem driveStraightSequenceItem(double targetY, double targetAngle) {
		return new Sequence.SequenceItem(State.Message.DRIVING_DONE, () -> {
			state = new DriveStraightState(targetY, targetAngle);
		});
	}

	public Sequence.SequenceItem driveSidewaysSequenceItem(double targetX, double targetAngle) {
		return new Sequence.SequenceItem(State.Message.DRIVING_DONE, () -> {
			state = new DriveSidewaysState(targetX, targetAngle);
		});
	}
}
