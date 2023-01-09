package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import android.graphics.Point;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PosePIDController;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.firstinspires.ftc.teamcode.utils.Trajectory;
import org.junit.rules.Stopwatch;

import java.util.concurrent.TimeUnit;

/**
 * A class for handling the driving of the robot.
 */
public class DrivingSystem {

	/**
	 * Marker marking PID activated methods.
	 */
	@interface PID {
	}

	/**
	 * Enum to indicate which robot we are currently running. The IMU initialization is
	 * unique for each robot, since the Control Hub's orientation is different.
	 */
	private enum Robot {
		ARMADILLO, NEW_ROBOT
	}

	private static final Robot robot = Robot.NEW_ROBOT;

	private static final double WHEEL_RADIUS_CM = 4.8;
	private static final double TICKS_PER_ROTATION = 515;
	private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS_CM * 2 * PI;
	private static final double ROTATION_EPSILON = toRadians(0.5);

	// in order to make the robot drive at an equal velocity for the same power when driving in the x and y directions,
	// we multiply its speed in the y direction by this constant.
	private static final double DRIVE_Y_FACTOR = RobotParameters.MAX_V_X/ RobotParameters.MAX_V_Y;


	private final LinearOpMode opMode;

	private final BNO055IMU imu;
	private final DcMotor frontRight;
	private final DcMotor frontLeft;
	private final DcMotor backRight;
	private final DcMotor backLeft;

	private double flPreviousTicks = 0;
	private double frPreviousTicks = 0;
	private double blPreviousTicks = 0;
	private double brPreviousTicks = 0;

	private final Pose positionCM = new Pose(0., 0., 0.);

	public final PositionLogger positionLogger; // Needs to be public to save the file from the opMode.
	private long lastCycleTime; // The time, in nanoseconds since the program began of the last time trackPosition was called.
	private long lastCycleDuration; // The duration, in nanoseconds, of the time between when trackPosition was called the last 2 times.

	public long getLastCycleTime(){
		return lastCycleTime;
	}
	public long getLastCycleDuration(){
		return lastCycleDuration;
	}
	public Pose getPosition(){
		return new Pose(positionCM);
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public DrivingSystem(LinearOpMode opMode) {
		this.opMode = opMode;
		imu = initializeImu(opMode);
		lastCycleTime = System.nanoTime();

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
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

		// Reset the distances measured by the motors
		positionLogger = new PositionLogger(this, this.opMode);
		resetDistance();
	}

	/**
	 * Creates an IMU object and calibrates it correctly to the current orientation.
	 *
	 * @param opMode the current opMode.
	 * @return an BNO055IMU object.
	 */
	private static BNO055IMU initializeImu(LinearOpMode opMode) {
		// Create the IMU
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		parameters.loggingEnabled = false;

		// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
		// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
		// and named "imu".
		BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		// armadillo and new robot require extra configuration for its IMU.
		// copied from https://ftcforum.firstinspires.org/forum/ftc-technology/53812-mounting-the-revhub-vertically

		byte axisMapConfigByte; //This is what to write to the AXIS_MAP_CONFIG register to swap the needed axis

		byte X_AXIS = 0b0;
		byte Y_AXIS = 0b01;
		byte Z_AXIS = 0b10;

		// IMU configuration explained in: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf, page 24

		if (robot == Robot.ARMADILLO) {
			axisMapConfigByte = 0x6; // swap x and z axis
		} else {
			axisMapConfigByte = (byte) (X_AXIS | Z_AXIS << 2 | Y_AXIS << 4); // swap z and y axis
		}

		byte AXIS_MAP_SIGN_BYTE = 0x0; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
		// Need to be in CONFIG mode to write to registers
		imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
		opMode.sleep(100); //Changing modes requires a delay before doing anything else
		//Write to the AXIS_MAP_CONFIG register
		imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfigByte);
		//Write to the AXIS_MAP_SIGN register
		imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE);
		//Need to change back into the IMU mode to use the gyro
		imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal);
		opMode.sleep(100); // Changing modes again requires a delay

		ElapsedTime elapsedTime = new ElapsedTime();

		while (!imu.isGyroCalibrated() && elapsedTime.milliseconds() < 2000) {
			// wait for the gyroscope calibration
			opMode.sleep(10);
		}
		return imu;
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
	 * Given any angle, normalizes it such that it is between PI and PI radians,
	 * increasing or decreasing by 2PI radians to make it so.
	 *
	 * @param angle Given angle in radians.
	 * @return The angle normalized (-PI < angle < PI).
	 */
	private static double normalizeAngle(double angle) {
		while (angle >= PI) angle -= 2.0 * PI;
		while (angle < -PI) angle += 2.0 * PI;
		return angle;
	}

	/**
	 * Gets the robots current azimuth and returns it.
	 * Angle is measured relative to the robot's starting angle, with positive angles being counterclockwise.
	 *
	 * @return double representing the robot's current angle.
	 */
	public double getCurrentAngle() {
		Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
		return orientation.firstAngle;
	}

	public Orientation getOrientation(){
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
	}

	/**
	 * returns how far sideways and forward, in centimeters,
	 * the robot has moved and rotated since the last time that resetDistance() was called.
	 * Assumes the robot hasn't moved in any other directions.
	 */
	public Pose getDistancesOld() {
		Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

		final double fL = frontLeft.getCurrentPosition();
		final double fR = frontRight.getCurrentPosition();
		final double bL = backLeft.getCurrentPosition();
		final double bR = backRight.getCurrentPosition();

		Pose movementChange = new Pose();
		movementChange.y = (fL + fR + bL + bR) / 4. * CM_PER_TICK;
//        movementChange.x = (fR - fL + bR - bL) / 4. * CM_PER_TICK;
		movementChange.x = (fR - fL + bL - bR) / 4. * CM_PER_TICK;
//        -fLChange + fRChange + bLChange - bRChange
		movementChange.angle = orientation.firstAngle;

		return movementChange;
	}

	/**
	 * Gets the movement of the robot in the robot's axis since the last tracked position.
	 *
	 * @return Point2D: Sum of movement Sideways, Sum of movement Forward; in cm.
	 */
	public PointD getDistances() {
		final double flChange = frontLeft.getCurrentPosition() - flPreviousTicks;
		final double frChange = frontRight.getCurrentPosition() - frPreviousTicks;
		final double blChange = backLeft.getCurrentPosition() - blPreviousTicks;
		final double brChange = backRight.getCurrentPosition() - brPreviousTicks;

		PointD movementChange = new PointD();

		movementChange.x = (-flChange + frChange + blChange - brChange) / 4. * CM_PER_TICK;
		movementChange.y = (flChange + frChange + blChange + brChange) / 4. * CM_PER_TICK;

		return movementChange;
	}

	/**
	 * Gets the movement of the robot in the robot's axis since the last tracked position and resets it.
	 * should be called only by trackPosition.
	 *
	 * @return Point2D: Sum of movement Sideways, Sum of movement Forward; in cm.
	 */
	public PointD updateDistances() {
		double flTicks = frontLeft.getCurrentPosition();
		double frTicks = frontRight.getCurrentPosition();
		double blTicks = backLeft.getCurrentPosition();
		double brTicks = backRight.getCurrentPosition();

		final double fLChange = flTicks - flPreviousTicks;
		final double fRChange = frTicks - frPreviousTicks;
		final double bLChange = blTicks - blPreviousTicks;
		final double bRChange = brTicks - brPreviousTicks;

		flPreviousTicks = flTicks;
		frPreviousTicks = frTicks;
		blPreviousTicks = blTicks;
		brPreviousTicks = brTicks;

		PointD movementChange = new PointD();
		movementChange.x = (-fLChange + fRChange + bLChange - bRChange) / 4. * CM_PER_TICK;
		movementChange.y = (fLChange + fRChange + bLChange + bRChange) / 4. * CM_PER_TICK;

		long currentTime = System.nanoTime();
		this.lastCycleDuration = currentTime - lastCycleTime;
		this.lastCycleTime = currentTime;

		return movementChange;
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

		// in order to make the driving the same velocity for the same power in the x and y directions,
		// reduce the y power slightly
		double y = powers.y * DRIVE_Y_FACTOR;
		double x = powers.x;
		double angle = powers.angle;
		// Determine how much power each motor should receive.
		double frontRightPower = y + x + angle;
		double frontLeftPower = y - x - angle;
		double backRightPower = y - x + angle;
		double backLeftPower = y + x - angle;

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
        trackPosition();
	}

	/**
	 * Makes the robot stop in place.
	 */
	public void stop() {
		driveMecanum(new Pose());
	}

	/**
	 * Drives the robot in the given orientation on the driver's axis and keeps track of it's position.
	 *
	 * @param powers Relative velocities vector.
	 */
	public void driveByAxis(Pose powers) {
		final double currentAngle = positionCM.angle;
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
	 * Keeps track of robot's position on the field.
	 */
	private void trackPosition() {
		final PointD positionChange = updateDistances();

		final double currentAngle = getCurrentAngle();
		final double angleAverage = (currentAngle + positionCM.angle) / 2;
		positionCM.angle = currentAngle;

		positionCM.x += positionChange.y * sin(angleAverage) + positionChange.x * cos(angleAverage);
		positionCM.y += positionChange.y * cos(angleAverage) - positionChange.x * sin(angleAverage);
	}

	/**
	 * Prints the robot's current position to the telemetry.
	 * Must call telemetry.update() after using this method.
	 */
	public void printPosition() {
		opMode.telemetry.addData("x", positionCM.x);
		opMode.telemetry.addData("y", positionCM.y);
		opMode.telemetry.addData("rot", toDegrees(positionCM.angle));
		double cycleFrequency = 1e9 / lastCycleDuration;
		opMode.telemetry.addData("Cycle Frequency [Hz]: ", cycleFrequency);
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
			distance = -distance;
			power = -power;
		}

		final double ANGLE_DEVIATION_SCALAR = 0.05;

		resetDistance();
		double startAngle = getDistancesOld().angle;
		double forwardDistance = getDistances().y;

		while (opMode.opModeIsActive() && Math.abs(forwardDistance) < distance) {
			Pose pose = getDistancesOld();
			forwardDistance = pose.y;
			double angleDeviation = AngleUnit.DEGREES.normalize(startAngle - pose.angle);
			double rotatePower = angleDeviation * ANGLE_DEVIATION_SCALAR;
			driveMecanum(new Pose(0, power, rotatePower));
			positionLogger.update();
			printPosition();
			opMode.telemetry.update();
		}
		stop();
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
			distance = -distance;
			power = -power;
		}

		double ANGLE_DEVIATION_SCALAR = 0.05;

		resetDistance();
		Pose pose = getDistancesOld();
		double startAngle = pose.angle;
		double sidewaysDistance = pose.x;

		while (opMode.opModeIsActive() && Math.abs(sidewaysDistance) < distance) {
			Pose distances = getDistancesOld();
			sidewaysDistance = distances.x;
			double angleDeviation = normalizeAngle(startAngle - distances.angle);
			driveMecanum(new Pose(power, 0, -angleDeviation * ANGLE_DEVIATION_SCALAR));
			positionLogger.update();
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
		final double powerScalar = 0.007 * 180/ PI;
		final double minPower = 0.2;

		double currentAngle = getCurrentAngle();

		// Angles must always be between -PI and PI RADIANS.
		// The function used below adds or subtracts 2PI RADIANS from the angle
		// so that it's always in the good range.
		double targetAngle = normalizeAngle(currentAngle + angle);

		double deviation = normalizeAngle(targetAngle - currentAngle);

		while (abs(deviation) > ROTATION_EPSILON) { // once the angular error is less than ROTATION_EPSILON, we have arrived
			currentAngle = getCurrentAngle();
			deviation = normalizeAngle(targetAngle - currentAngle);
			// the power is proportional to the deviation, but may not go below minPower.
			double rotatePower = deviation * powerScalar + minPower * signum(deviation);
			driveMecanum(new Pose(0, 0, rotatePower));
			positionLogger.update();
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

		Pose Deviation = Pose.difference(targetLocation, positionCM);
		Deviation.normalizeAngle();
		Pose actPowers = new Pose();

		while (abs(Deviation.x) > epsilon.x ||
				abs(Deviation.y) > epsilon.y ||
				abs(Deviation.angle) > epsilon.angle) {

			if (abs(Deviation.x) > epsilon.x) {
				actPowers.x = signum(Deviation.x) * minPower.x + Deviation.x * powerScalar.x;
			}
			if (abs(Deviation.y) > epsilon.y) {
				actPowers.y = signum(Deviation.y) * minPower.y + Deviation.y * powerScalar.y;
			}
			if (abs(Deviation.angle) > epsilon.angle) {
				actPowers.angle = signum(Deviation.angle) * minPower.angle
						+ Deviation.angle * powerScalar.angle;
			}

			driveByAxis(actPowers);

			actPowers.setValue(new Pose());
			Deviation = Pose.difference(targetLocation, positionCM);
			Deviation.normalizeAngle();
		}
		stop();
	}

	/**
	 * Drives the robot straight a given distance.
	 *
	 * @param distance How far the robot should go, measured in cm.
	 */
	@PID
	public void driveY(double distance) {
		final double ANGLE_DEVIATION_SCALAR = 0.05;
		final double epsilon = 0.5;

		final double yTarget = positionCM.y + distance;
		final double startAngle = getCurrentAngle();

		PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = yTarget - positionCM.y;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon) {
			double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
			actPowers.y = myPIDController.powerByDeviation(deviation);
			actPowers.angle = ANGLE_DEVIATION_SCALAR * angleDeviation;

			driveMecanum(actPowers);
			deviation = yTarget - positionCM.y;
		}
		stop();
	}

	/**
	 * Drives the robot sideways a given distance.
	 *
	 * @param distance How far the robot should go, measured in cm.
	 */
	@PID
	public void driveX(double distance) {
		final double ANGLE_DEVIATION_SCALAR = 0.05;
		final double epsilon = 0.5;

		final double xTarget = positionCM.x + distance;
		final double startAngle = getCurrentAngle();

		PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = xTarget - positionCM.x;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon) {
			double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
			actPowers.x = myPIDController.powerByDeviation(deviation);
			actPowers.angle = ANGLE_DEVIATION_SCALAR * angleDeviation;

			driveMecanum(actPowers);
			deviation = xTarget - positionCM.y;
		}
		stop();
	}

	/**
	 * Rotates the robot a given number of radians.
	 * A positive angle means clockwise rotation, a negative angle is counterclockwise.
	 *
	 * @param angleDifference An angle to rotate the robot by.
	 */
	@PID
	public void driveAngle(double angleDifference) {
		final double targetAngle = normalizeAngle(positionCM.angle + angleDifference);
		double deviation = normalizeAngle(targetAngle - positionCM.angle);

		PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		Pose actPowers = new Pose();

		while (abs(deviation) > ROTATION_EPSILON) {
			actPowers.angle = myPIDController.powerByDeviation(deviation);
			driveMecanum(actPowers);
			deviation = normalizeAngle(targetAngle - positionCM.angle);
		}
		stop();
	}

	/**
	 * Drives the robot to a given location on the field.
	 *
	 * @param targetLocation The location and orientation for the robot to reach.
	 */
	@PID
	public void move2(Pose targetLocation) {
		final Pose Kp = new Pose(0.01, 0.01, 0.73);
		final Pose Ki = new Pose(0, 0, 0);
		final Pose Kd = new Pose(0.000001, 0.000001, 0.00002);

		final Pose epsilon = new Pose(-0.5, -1, -ROTATION_EPSILON);

		Pose Deviation = Pose.difference(targetLocation, positionCM);
		Deviation.normalizeAngle();
		PosePIDController actPowers = new PosePIDController(Kp, Ki, Kd);

		while (opMode.opModeIsActive() && (
				abs(Deviation.x) > epsilon.x ||
						abs(Deviation.y) > epsilon.y ||
						abs(Deviation.angle) > epsilon.angle)) {

			driveByAxis(actPowers.powerByDeviation(Deviation));
			printPosition();
			opMode.telemetry.update();
			Deviation = Pose.difference(targetLocation, positionCM);
			Deviation.normalizeAngle();
		}
		stop();
	}

	/**
	 * Drives the robot to a location relative to itself.
	 *
	 * @param distances The relative location and orientation for the robot to reach.
	 */
	@PID
	public void moveRelativeToRobot(Pose distances) {
		Pose targetLocation = new Pose();

		final double cosAngle = cos(positionCM.angle);
		final double sinAngle = sin(positionCM.angle);

		targetLocation.x = positionCM.x + distances.x * cosAngle + distances.y * sinAngle;
		targetLocation.y = positionCM.y - distances.x * sinAngle + distances.y * cosAngle;
		targetLocation.angle = normalizeAngle(positionCM.angle + distances.angle);

		move2(targetLocation);
	}

	public void driveForwardByProfile(AccelerationProfile accelerationProfile){
		ElapsedTime elapsedTime = new ElapsedTime();
	}

	public void driveByPath(Trajectory traj){

		ElapsedTime elapsedTime = new ElapsedTime();

		while (opMode.opModeIsActive() && elapsedTime.seconds() < traj.getTotalTime()) {
			final double currentTime = elapsedTime.seconds();
			final double k_pointDeviation = 0.5;
			final double k_angleDeviation = 0.1;
//			Vectors:
			Pose realPose = new Pose(positionCM.x,positionCM.y,getCurrentAngle());
			Pose targetPose = traj.getPose(currentTime);

			Pose deviation = new Pose(
					(targetPose.x - realPose.x)*k_pointDeviation,
					(targetPose.y - realPose.y)*k_pointDeviation,
					(realPose.angle - normalizeAngle(targetPose.angle))*k_angleDeviation
			);

			Pose powers = traj.getPowers(elapsedTime.seconds());
			driveMecanum(new Pose(powers.x+deviation.x, powers.y+deviation.y, powers.angle+deviation.angle)
			);
			positionLogger.update();
			printPosition();
			opMode.telemetry.addData("track position", positionCM.x + "," + positionCM.y);
			opMode.telemetry.update();
		}
		stop();
	}
}
