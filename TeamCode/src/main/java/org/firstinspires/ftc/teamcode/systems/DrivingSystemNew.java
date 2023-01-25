package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.utils.Utils.normalizeAngle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.RestingState;
import org.firstinspires.ftc.teamcode.utils.Sequence;
import org.firstinspires.ftc.teamcode.utils.State;

/**
 * A class for handling moving the robot through space.
 */
public class DrivingSystemNew {
	private static final double ROTATION_EPSILON = toRadians(0.5);

	private final LinearOpMode opMode;
	private final SystemCoordinator systemCoordinator;

	private final DcMotor frontRight;
	private final DcMotor frontLeft;
	private final DcMotor backRight;
	private final DcMotor backLeft;

	private State state = new RestingState();

	/**
	 * A state used when the robot should be moving.
	 */
	public class ActingState implements State {
		private final double totalMovementTime;
		private final ElapsedTime timer;
		private final double velocityX;
		private final double velocityY;
		private final double angle;

		/**
		 * @param desiredPosition   The desired position the robot should move to, Pose.
		 * @param totalMovementTime The total time the movement should take.
		 */
		public ActingState(Pose desiredPosition, double totalMovementTime) {
			this.totalMovementTime = totalMovementTime;
			this.timer = new ElapsedTime();
			// Calculate position change per tick
			//TODO: USE ACCELERATION PROFILE - THIS DOES NOT CURRENTLY WORK
			Pose position = systemCoordinator.trackingSystem.getPosition();
			this.velocityX = (desiredPosition.x - position.x) / (totalMovementTime);
			this.velocityY = (desiredPosition.y - position.y) / (totalMovementTime);
			this.angle = (desiredPosition.angle - position.angle) / (totalMovementTime);
		}

		public void tick() {
			// The claw has reached its desired position
			if (timer.time() > totalMovementTime) {
				state = new RestingState();
				return;
			}

			// Otherwise, update the robot position
			driveMecanum(new Pose(velocityX, velocityY, angle));
		}

		public void onReceiveMessage(State.Message message) {
			// Do nothing
		}
	}



	public class DriveStraightState implements State{

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
			driveMecanum(new Pose(0, yPower, rotPower));
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
			driveMecanum(new Pose(xPower, 0, rotPower));
		}
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public DrivingSystemNew(LinearOpMode opMode, SystemCoordinator systemCoordinator) {
		this.opMode = opMode;
		this.systemCoordinator = systemCoordinator;

		// Get mecanum wheels interfaces
		frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
		frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
		backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
		backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

		// Makes the motors break when their power is set to zero, so they can better stop in place.
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
		// Determine how much power each motor should receive.
		double frontRightPower = powers.y + powers.x + powers.angle;
		double frontLeftPower = powers.y - powers.x - powers.angle;
		double backRightPower = powers.y - powers.x + powers.angle;
		double backLeftPower = powers.y + powers.x - powers.angle;

		// The method motor.setPower() only accepts numbers between -1 and 1.
		// If any number that we want to give it is greater than 1,
		// we must divide all the numbers equally so the maximum is 1
		// and the proportions are preserved.
		double norm = max(max(frontRightPower, frontLeftPower), max(backRightPower, backLeftPower));
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
	 * Drives the robot in the given orientation on the driver's axis and keeps track of it's position.
	 *
	 * @param powers Relative velocities vector.
	 */
	public void driveByAxis(Pose powers) {
		final double currentAngle = systemCoordinator.trackingSystem.getPosition().angle;
		final double cosAngle = cos(currentAngle);
		final double sinAngle = sin(currentAngle);

		Pose mecanumPowers = new Pose(
				cosAngle * powers.x - sinAngle * powers.y,
				cosAngle * powers.y + sinAngle * powers.x,
				powers.angle
		);

		driveMecanum(mecanumPowers);
	}

	public Sequence.SequenceItem driveStraightSequenceItem(double targetY, double targetAngle){
		return new Sequence.SequenceItem(State.Message.DRIVING_DONE, ()->{
			state = new DriveStraightState(targetY, targetAngle);
		});
	}

	public Sequence.SequenceItem driveSidewaysSequenceItem(double targetX, double targetAngle){
		return new Sequence.SequenceItem(State.Message.DRIVING_DONE, ()->{
			state = new DriveStraightState(targetX, targetAngle);
		});
	}



	/**
	 * Makes the robot stop in place.
	 */
	private void stop() {
		driveMecanum(new Pose());
	}


	/**
	 * Ticks the driving system.
	 */
	public void tick() {
		state.tick();
	}

}
