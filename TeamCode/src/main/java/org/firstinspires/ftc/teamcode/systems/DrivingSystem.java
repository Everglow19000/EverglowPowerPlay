package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.DRIVE_Y_FACTOR;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.DROP_OFF_DISTANCE;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_a_accelerating;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_a_decelerating;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_d_error;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_error;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_v_x;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.k_v_y;
import static org.firstinspires.ftc.teamcode.utils.Utils.normalizeAngle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DriveByPath.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.PDFFController;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PID.PosePIDController;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.Trajectory;

/**
 * A class for handling moving the robot through space.
 */
public class DrivingSystem {
	public boolean enabled = true;
	private static final double FRONT_SCALAR = 1.16;

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
	 * Used in drive by path.
	 */
	private Pose targetPosition = new Pose();

	public Pose getTargetPosition() {
		return new Pose(targetPosition);
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public DrivingSystem(LinearOpMode opMode) {

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
		if (!enabled){
			stop();
			return;
		}
		// makes the motors float if their power is set to zero when driving.
		// calling this once each cycle shouldn't be a problem, as it seems the library keeps track of the previous value, and only updates if needed.
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		powers = new Pose(powers);
		powers.y *= DRIVE_Y_FACTOR;

		// Determine how much power each motor should receive.
		double frontRightPower = powers.y + powers.x * FRONT_SCALAR + powers.angle;
		double frontLeftPower = powers.y - powers.x * FRONT_SCALAR - powers.angle;
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
	public void stop() {
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
	 * Drives by the axes of the field.
	 *
	 * @param powers Velocity vector containing elements: x, y, and an azimuth angle.
	 */
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

	/**
	 * Drives to the dropoff distance from the pole that the robot is closest to.
	 */
	public void driveToClosestPole() {
		PointD poleLocation = SystemCoordinator.instance.trackingSystem.getClosestPoleLocation();
		Pose currentPosition = SystemCoordinator.instance.trackingSystem.getPosition();

		double angleToPole = SystemCoordinator.instance.trackingSystem.angleTo(poleLocation);
		double angleDeviation = normalizeAngle(angleToPole - currentPosition.angle);
		double distance = hypot(poleLocation.x - currentPosition.x, poleLocation.y - currentPosition.y) - DROP_OFF_DISTANCE;

		while (abs(distance) < 0.5) {
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

		final Pose epsilon = new Pose(-0.5, -1, -toRadians(0.5));
		Pose deviation = Pose.difference(targetLocation, SystemCoordinator.instance.trackingSystem.getPosition());
		deviation.normalizeAngle();
		PosePIDController actPowers = new PosePIDController(Kp, Ki, Kd);

		while (SystemCoordinator.instance.opMode.opModeIsActive() && (
				abs(deviation.x) > epsilon.x ||
						abs(deviation.y) > epsilon.y ||
						abs(deviation.angle) > epsilon.angle)) {

			SystemCoordinator.instance.tick();
			deviation = Pose.difference(targetLocation, SystemCoordinator.instance.trackingSystem.getPosition());
			deviation.normalizeAngle();
			driveByAxis(actPowers.powerByDeviation(deviation));
		}
		stop();
	}

	public void move3(Pose targetPosition){
		Pose k = new Pose(0.02, 0.02, 1);
		Pose minPower = new Pose(0.2, 0.2, 0.2);
		Pose epsilon = new Pose(2, 2, toRadians(3));
		boolean xArrived = false;
		boolean yArrived = false;
		boolean angleArrived = false;

		while (SystemCoordinator.instance.opMode.opModeIsActive() && (!xArrived || !yArrived || !angleArrived)){
			SystemCoordinator.instance.tick();
			Pose deviation = Pose.difference(targetPosition, SystemCoordinator.instance.trackingSystem.getPosition());
			deviation.normalizeAngle();
			xArrived = abs(deviation.x) < epsilon.x;
			yArrived = abs(deviation.y) < epsilon.y;
			angleArrived = abs(deviation.angle) < epsilon.angle;

			Pose powers = new Pose();
			if (!xArrived){
				powers.x = deviation.x * k.x + minPower.x * signum(deviation.x);
			}
			if (!yArrived){
				powers.y = deviation.y * k.y + minPower.y * signum(deviation.y);
			}
			if (!angleArrived){
				powers.angle = deviation.angle * k.angle + minPower.angle * signum(deviation.angle);
			}
			driveByAxis(powers);
		}
	}



	/**
	 * Drives the robot in the given orientation in the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis(Pose Powers) {
		final double K = 0.03;
		PointD SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		PointD squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();
		/*
		if (abs(SquareLocation.x) >= 3 && signum(squareDeviation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(squareDeviation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}*/

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
	 * Drives the robot in the given orientation in the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis2(Pose Powers) {
		final double K = 50.;

		PointD SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		PointD squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();
		/*
		if (abs(SquareLocation.x) >= 3 && signum(SquareLocation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(SquareLocation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}*/

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
	 * Drives the robot in the given orientation in the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis3(Pose Powers) {

		PointD SquareLocation = SystemCoordinator.instance.trackingSystem.getTileLocation();
		PointD squareDeviation = SystemCoordinator.instance.trackingSystem.getTileDeviation();
		/*
		if (abs(SquareLocation.x) >= 3 && signum(squareDeviation.x) == signum(Powers.x)) {
			Powers.x = 0;
		}
		if (abs(SquareLocation.y) >= 3 && signum(squareDeviation.y) == signum(Powers.y)) {
			Powers.y = 0;
		}*/

		if (abs(Powers.x) > abs(Powers.y)) {
			Powers.y = -squareDeviation.y * abs(Powers.x) * (1 - abs(squareDeviation.x)) / (1 - abs(squareDeviation.y));
		} else {
			Powers.x = -squareDeviation.x * abs(Powers.y) * (1 - abs(squareDeviation.y)) / (1 - abs(squareDeviation.x));
		}

		driveByAxis(Powers);
	}

	public void driveX(double distance) {
		final double epsilon = 0.5;
		final double xTarget = SystemCoordinator.instance.trackingSystem.getPosition().x + distance;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = xTarget - SystemCoordinator.instance.trackingSystem.getPosition().x;
		double angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && SystemCoordinator.instance.opMode.opModeIsActive()) {
			SystemCoordinator.instance.tick();
			actPowers.x = 0.003 * deviation + 0.4 * signum(deviation);
			actPowers.angle = 1 * angleDeviation;
			driveByAxis(actPowers);
			deviation = xTarget - SystemCoordinator.instance.trackingSystem.getPosition().x;
			angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		}
		stop();
	}

	public void driveToX(double targetX) {
		final double epsilon = 0.5;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = targetX - SystemCoordinator.instance.trackingSystem.getPosition().x;
		double angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && SystemCoordinator.instance.opMode.opModeIsActive()) {
			SystemCoordinator.instance.tick();
			actPowers.x = 0.003 * deviation + 0.4 * signum(deviation);
			actPowers.angle = 1 * angleDeviation;
			driveByAxis(actPowers);
			deviation = targetX - SystemCoordinator.instance.trackingSystem.getPosition().x;
			angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		}
		stop();
	}

	public void driveY(double distance) {
		final double epsilon = 1;
		final double yTarget = SystemCoordinator.instance.trackingSystem.getPosition().y + distance;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = yTarget - SystemCoordinator.instance.trackingSystem.getPosition().y;
		double angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && SystemCoordinator.instance.opMode.opModeIsActive()) {
			SystemCoordinator.instance.tick();
			actPowers.y = 0.003 * deviation + 0.4 * signum(deviation);
			actPowers.angle = 1 * angleDeviation;
			driveByAxis(actPowers);
			deviation = yTarget - SystemCoordinator.instance.trackingSystem.getPosition().y;
			angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		}
	}

	public void driveToY(double targetY) {
		final double epsilon = 1;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = targetY - SystemCoordinator.instance.trackingSystem.getPosition().y;
		double angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && SystemCoordinator.instance.opMode.opModeIsActive()) {
			SystemCoordinator.instance.tick();
			actPowers.y = 0.003 * deviation + 0.4 * signum(deviation);
			actPowers.angle = 1 * angleDeviation;
			driveByAxis(actPowers);
			deviation = targetY - SystemCoordinator.instance.trackingSystem.getPosition().y;
			angleDeviation = - SystemCoordinator.instance.trackingSystem.getPosition().angle;
		}
	}

	public void driveForwardByProfile(AccelerationProfile accelerationProfile) {
		resetDistance();
		ElapsedTime elapsedTime = new ElapsedTime();
		PDFFController controller = new PDFFController(k_v_y, k_a_accelerating, k_a_decelerating, k_error, k_d_error);
		while (SystemCoordinator.instance.opMode.opModeIsActive() && elapsedTime.seconds() < accelerationProfile.finalTime()) {
			SystemCoordinator.instance.tick();
			Pose pose = SystemCoordinator.instance.trackingSystem.getPosition();
			final double currentTime = elapsedTime.seconds();
			double targetPosition = accelerationProfile.getPosition(currentTime);
			double targetVelocity = accelerationProfile.getVelocity(currentTime);
			double targetAcceleration = accelerationProfile.acceleration(currentTime);
			double error = targetPosition - pose.y;

			double forwardPower = controller.getPower(currentTime, error, targetVelocity, targetAcceleration);
			driveMecanum(new Pose(0, forwardPower, 0));
			this.targetPosition = new Pose(0, targetPosition, 0);
		}
		stop();
	}

	public void driveSidewaysByProfile(AccelerationProfile accelerationProfile) {
		resetDistance();
		ElapsedTime elapsedTime = new ElapsedTime();
		PDFFController controller = new PDFFController(k_v_x, k_a_accelerating, k_a_decelerating, k_error, k_d_error);
		while (SystemCoordinator.instance.opMode.opModeIsActive() && elapsedTime.seconds() < accelerationProfile.finalTime()) {
			SystemCoordinator.instance.tick();
			Pose pose = SystemCoordinator.instance.trackingSystem.getPosition();
			final double currentTime = elapsedTime.seconds();
			double targetPosition = accelerationProfile.getPosition(currentTime);
			double targetVelocity = accelerationProfile.getVelocity(currentTime);
			double targetAcceleration = accelerationProfile.acceleration(currentTime);
			double error = targetPosition - pose.x;

			double forwardPower = controller.getPower(currentTime, error, targetVelocity, targetAcceleration);
			driveMecanum(new Pose(forwardPower, 0, 0));
			this.targetPosition = new Pose(targetPosition, 0, 0);
		}
		stop();
	}


	public void driveByPath(Trajectory traj, double targetAngle, double anglePowerScalar) {
		resetDistance();

		ElapsedTime elapsedTime = new ElapsedTime();
		PDFFController xController = new PDFFController(k_v_x, k_a_accelerating, k_a_decelerating, k_error, k_d_error);
		PDFFController yController = new PDFFController(k_v_y, k_a_accelerating, k_a_decelerating, k_error, k_d_error);

		double prev_t = 0;
		double prev_v_x = 0;
		double prev_v_y = 0;
		double prev_v_rot = 0;

		while (SystemCoordinator.instance.opMode.opModeIsActive() && elapsedTime.seconds() < traj.getTotalTime()) {
			SystemCoordinator.instance.tick();
			final double time = elapsedTime.seconds();
			final double dt = time - prev_t;

			// Getting currentPose, targetPose, and error
			Pose currentPose = SystemCoordinator.instance.trackingSystem.getPosition();
			Pose targetPose = traj.getPose(time);
			if (targetPose == null) return;

			targetPose.angle = targetAngle;
			this.targetPosition = targetPose;
			Pose error = new Pose(targetPose.x - currentPose.x, targetPose.y - currentPose.y,
					normalizeAngle(targetPose.angle - currentPose.angle));

			// Velocity and acceleration poses
			Pose velocity = traj.getVelocity(time);
			Pose acceleration = new Pose((velocity.x - prev_v_x) / dt, (velocity.y - prev_v_y) / dt,
					(velocity.angle - prev_v_rot) / dt);

			// Using PDFFController to get power and driving
			double xPower = xController.getPower(time, error.x, velocity.x, acceleration.x);
			double yPower = yController.getPower(time, error.y, velocity.y, acceleration.y);
			double rotationPower = error.angle * anglePowerScalar;

			driveByAxis(new Pose(xPower, yPower, rotationPower));

			// Setting the 'previous' variables
			prev_t = time;
			prev_v_x = velocity.x;
			prev_v_y = velocity.y;
			prev_v_rot = velocity.angle;
		}
		stop();
	}

}
