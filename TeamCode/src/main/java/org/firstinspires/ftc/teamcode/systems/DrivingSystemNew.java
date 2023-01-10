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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.RestingState;
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

	private State state;

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
	private void driveMecanum(Pose powers) {
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
	 * Makes the robot stop in place.
	 */
	private void stop() {
		driveMecanum(new Pose());
	}


	/**
	 * Changes the robot's state to make it go to a given position.
	 *
	 * @param powers       Velocity vector containing elements: x, y, and an azimuth angle.
	 * @param movementTime The time it should take the robot to reach the desired position.
	 */
	public void goTo(Pose powers, double movementTime) {
		state = new ActingState(powers, movementTime);
	}

	/**
	 * Changes the robot's state to make it go to a given position.
	 * The time to reach the position is static at 10 seconds.
	 *
	 * @param powers Velocity vector containing elements: x, y, and an azimuth angle.
	 */
	public void goTo(Pose powers) {
		goTo(powers, 10);
	}

	/**
	 * Ticks the driving system.
	 */
	public void tick() {
		state.tick();
	}

	/**
	 * Receives a message from all the other classes.
	 */
	public void receiveMessage() {
		state.onReceiveMessage();
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

	/**
	 * Drives the robot straight a given distance.
	 *
	 * @param distance How far the robot should go, measured in cm.
	 * @param power    How much power should be given to the motors, from 0 to 1.
	 */
	public void driveStraight(double distance, double power) {
		// If we're traveling a negative distance, that means traveling backwards,
		// so the power should be inverted and so should the distance.
		if (distance < 0) {
			distance *= -1;
			power *= -1;
		}

		final double ANGLE_DEVIATION_SCALAR = 0.05;

		resetDistance();
		double startAngle = systemCoordinator.trackingSystem.getPosition().angle;
		double forwardDistance = systemCoordinator.trackingSystem.getPosition().y;

		while (opMode.opModeIsActive() && Math.abs(forwardDistance) < distance) {
			Pose pose = systemCoordinator.trackingSystem.getPosition();
			forwardDistance = pose.y;
			double angleDeviation = AngleUnit.DEGREES.normalize(startAngle - pose.angle);
			double rotatePower = angleDeviation * ANGLE_DEVIATION_SCALAR;
			driveMecanum(new Pose(0, power, rotatePower));
			systemCoordinator.trackingSystem.printPosition();
			opMode.telemetry.update();
		}
		stop();
		resetDistance();
	}

	/**
	 * Drives the robot sideways a given distance. Positive direction is rightwards.
	 *
	 * @param distance How far the robot should go, measured in cm.
	 * @param power    How much power should be given to the motors, from 0 to 1.
	 */
	public void driveSideways(double distance, double power) {
		// If we're traveling a negative distance, that means traveling left,
		// so the power should be inverted and so should the distance.
		if (distance < 0) {
			distance *= -1;
			power *= -1;
		}

		double ANGLE_DEVIATION_SCALAR = 0.05;

		resetDistance();
		Pose pose = systemCoordinator.trackingSystem.getPosition();
		double startAngle = pose.angle;
		double sidewaysDistance = pose.x;
		while (opMode.opModeIsActive() && Math.abs(sidewaysDistance) < distance) {
			Pose distances = systemCoordinator.trackingSystem.getPosition();
			sidewaysDistance = distances.x;
			double angleDeviation = normalizeAngle(startAngle - distances.angle);
			driveMecanum(new Pose(power, 0, -angleDeviation * ANGLE_DEVIATION_SCALAR));
		}
		stop();
	}

	/**
	 * Rotates the robot a given number of radians.
	 * A positive angle means clockwise rotation, a negative angle is counterclockwise.
	 *
	 * @param angle Angle (in radians) to rotate the robot.
	 */
	public void rotate(double angle) {
		final double powerScalar = 0.007;
		final double minPower = 0.2;

		double currentAngle = systemCoordinator.trackingSystem.getPosition().angle;

		// Angles must always be between -PI and PI RADIANS.
		// The function used below adds or subtracts 2PI RADIANS from the angle
		// so that it's always in the good range.
		double targetAngle = normalizeAngle(currentAngle + angle);

		double deviation = normalizeAngle(targetAngle - currentAngle);

		while (abs(deviation) > ROTATION_EPSILON) { // once the angular error is less than ROTATION_EPSILON, we have arrived
			currentAngle = systemCoordinator.trackingSystem.getPosition().angle;
			deviation = normalizeAngle(targetAngle - currentAngle);
			// the power is proportional to the deviation, but may not go below minPower.
			double rotatePower = deviation * powerScalar + minPower * signum(deviation);
			driveMecanum(new Pose(0, 0, rotatePower));
		}
		stop();
	}

	/**
	 * Drives the robot in a straight line to a given position on the board (x, y, angle).
	 *
	 * @param targetLocation The location and orientation for the robot to reach.
	 */
	public void moveTo(Pose targetLocation) {
		final Pose powerScalar = new Pose(0.007, 0.008, 0.6);
		final Pose minPower = new Pose(0.12, 0.15, 0.08);
		final Pose epsilon = new Pose(0.5, 1, 0.0087);

		Pose deviation = Pose.difference(targetLocation, systemCoordinator.trackingSystem.getPosition());
		deviation.angle = normalizeAngle(deviation.angle);
		Pose actPowers = new Pose();

		while (abs(deviation.x) > epsilon.x ||
				abs(deviation.y) > epsilon.y ||
				abs(deviation.angle) > epsilon.angle) {

			if (abs(deviation.x) > epsilon.x) {
				actPowers.x = signum(deviation.x) * minPower.x + deviation.x * powerScalar.x;
			}
			if (abs(deviation.y) > epsilon.y) {
				actPowers.y = signum(deviation.y) * minPower.y + deviation.y * powerScalar.y;
			}
			if (abs(deviation.angle) > epsilon.angle) {
				actPowers.angle = signum(deviation.angle) * minPower.angle
						+ deviation.angle * powerScalar.angle;
			}

			driveByAxis(actPowers);

			actPowers.setValue(new Pose());
			deviation = Pose.difference(targetLocation, systemCoordinator.trackingSystem.getPosition());
			deviation.angle = normalizeAngle(deviation.angle);
		}
		stop();
	}
}
