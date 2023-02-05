package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.utils.Utils.normalizeAngle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PosePIDController;
import org.firstinspires.ftc.teamcode.utils.State;
import org.firstinspires.ftc.teamcode.utils.RestingState;
import org.firstinspires.ftc.teamcode.utils.Sequence;

/**
 * A class for handling moving the robot through space.
 */
public class DrivingSystem {
	private static final double EPSILON = 1;
	private static final double ROTATION_EPSILON = toRadians(0.5);
	private static final double DROP_OFF_DISTANCE = 25;

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

		frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

	public void driveByAxis(Pose powers) {
		final double currentAngle = SystemCoordinator.instance.trackingSystem.getPosition().angle;
		final double cosAngle = cos(currentAngle);
		final double sinAngle = sin(currentAngle);

		Pose mecanumPowers = new Pose(
				cosAngle * powers.x - sinAngle * powers.y,
				cosAngle * powers.y + sinAngle * powers.x,
				powers.angle
		);

		driveMecanum(mecanumPowers);
	}




	public void driveToClosestPole() {
		double Epsilon = 0.5;

		Point2D poleLocation = SystemCoordinator.instance.trackingSystem.closestPoleLocation();
		Pose divrePowers = new Pose();
		double angleToPole = SystemCoordinator.instance.trackingSystem.angleTo(poleLocation);
		Pose currentPosition = SystemCoordinator.instance.trackingSystem.getPosition();
		double angleDeviation = normalizeAngle(angleToPole - currentPosition.angle);
		double distance = hypot(poleLocation.x - currentPosition.x, poleLocation.y - currentPosition.y) - DROP_OFF_DISTANCE;
		while(abs(distance) < 0.5) {
			double power = distance * 0.007 + 0.05;
			driveByAxis(new Pose(power * sin(angleToPole), power * cos(angleToPole), angleDeviation * 0.3 + 0.07));

			angleToPole = SystemCoordinator.instance.trackingSystem.angleTo(poleLocation);
			currentPosition = SystemCoordinator.instance.trackingSystem.getPosition();
			angleDeviation = normalizeAngle(angleToPole - currentPosition.angle);
			distance = hypot(poleLocation.x - currentPosition.x, poleLocation.y - currentPosition.y) - DROP_OFF_DISTANCE;
		}
	}


	public void move2(Pose targetLocation) {
		final Pose Kp = new Pose(0.01, 0.01, 0.73);
		final Pose Ki = new Pose(0, 0, 0);
		final Pose Kd = new Pose(0.000001, 0.000001, 0.00002);

		final Pose epsilon = new Pose(-0.5, -1, -ROTATION_EPSILON);
		Pose Deviation = Pose.difference(targetLocation, SystemCoordinator.instance.trackingSystem.getPosition());
		Deviation.normalizeAngle();
		PosePIDController actPowers = new PosePIDController(Kp, Ki, Kd);

		while (opMode.opModeIsActive() && (
				abs(Deviation.x) > epsilon.x ||
						abs(Deviation.y) > epsilon.y ||
						abs(Deviation.angle) > epsilon.angle)) {

			driveByAxis(actPowers.powerByDeviation(Deviation));
			opMode.telemetry.update();
			Deviation = Pose.difference(targetLocation, SystemCoordinator.instance.trackingSystem.getPosition());
			Deviation.normalizeAngle();
		}
		stop();
	}

	/**
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis(Pose Powers) {
		final double K = 0.03;


		Point2D SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		Point2D squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();

		if (abs(SquareLocation.x) >= 3 && signum(squareDeviation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(squareDeviation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}

		if (abs(Powers.x) > abs(Powers.y)) {
			Powers.x *= 1 - abs(squareDeviation.y);
			Powers.y = -abs(squareDeviation.y) * squareDeviation.y * abs(Powers.x) * K;
		} else {
			Powers.y *= 1 - abs(squareDeviation.x);
			Powers.x = -abs(squareDeviation.x) * squareDeviation.x * abs(Powers.y) * K;
		}

		driveByAxis(Powers);
	}


	/**
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis2(Pose Powers) {
		final double K = 50.;

		Point2D SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		Point2D squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();

		if (abs(SquareLocation.x) >= 3 && signum(SquareLocation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(SquareLocation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}

		if (abs(Powers.x) > abs(Powers.y)) {
			Powers.x *= 1 - abs(squareDeviation.y);
			double xPartChange = squareDeviation.y * squareDeviation.y;
			double xChange = (1 - signum(Powers.x) * squareDeviation.x) / 2;
			Powers.x += Powers.x * xPartChange * (xChange - 1);
			Powers.y = -squareDeviation.y * abs(squareDeviation.y) * abs(Powers.x) * K;
		} else {
			Powers.y *= 1 - abs(squareDeviation.x);
			double yPartChange = squareDeviation.x * squareDeviation.x;
			double yChange = (1 - signum(Powers.y) * squareDeviation.y) / 2;
			Powers.y += Powers.y * yPartChange * (yChange - 1);
			Powers.x = -squareDeviation.x * abs(squareDeviation.x) * abs(Powers.y) * K;
		}

		driveByAxis(Powers);
	}

	/**
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis3(Pose Powers) {

		Point2D SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		Point2D squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();

		if (abs(SquareLocation.x) >= 3 && signum(squareDeviation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(squareDeviation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}

		if (abs(Powers.x) > abs(Powers.y)) {
			Powers.y = signum(squareDeviation.y) * abs(Powers.x) * (1 - abs(squareDeviation.x)) / (1 - abs(squareDeviation.y));
		} else {
			Powers.x = signum(squareDeviation.x) * abs(Powers.y) * (1 - abs(squareDeviation.y)) / (1 - abs(squareDeviation.x));
		}

		driveByAxis(Powers);
	}

	public void driveX(double distance) {
		final double epsilon = 0.5;
		final double xTarget = SystemCoordinator.instance.trackingSystem.getPosition().x + distance;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = xTarget - SystemCoordinator.instance.trackingSystem.getPosition().x;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && opMode.opModeIsActive()) {
			actPowers.x = 0.003 * deviation + 0.15 * signum(deviation);
			driveByAxis(actPowers);
			deviation = xTarget - SystemCoordinator.instance.trackingSystem.getPosition().x;
		}
		stop();
	}

	public void driveY(double distance) {
		final double epsilon = 0.5;
		final double yTarget = SystemCoordinator.instance.trackingSystem.getPosition().y + distance;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = yTarget - SystemCoordinator.instance.trackingSystem.getPosition().y;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && opMode.opModeIsActive()) {
			actPowers.y = 0.003 * deviation + 0.15 * signum(deviation);
			driveByAxis(actPowers);
			deviation = yTarget - SystemCoordinator.instance.trackingSystem.getPosition().y;
		}
		stop();
	}



	/**
	 * drives the Robot to location
	 *
	 * @param Distances the relative distances to the by.
	 */
	public void moveRelativeToRobot(Pose Distances) {
		Pose targetLocation = new Pose();
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
