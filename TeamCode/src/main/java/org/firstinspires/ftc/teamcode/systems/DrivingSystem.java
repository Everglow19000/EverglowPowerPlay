package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import java.util.List;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PdffController;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PosePIDController;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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
	 * The square tile's side length in centimeters.
	 */
	public static final double TILE_SIZE = 71; // 60.5
	/**
	 * The robot's length in centimeters.
	 */
	private static final double ROBOT_LENGTH = 44.;
	/**
	 * The robot's width in centimeters.
	 */
	private static final double ROBOT_WIDTH = 35.;
	/**
	 * The robot's wheel radius in centimeters.
	 */
	private static final double WHEEL_RADIUS = 4.8;
	/**
	 * The amount of wheel ticks per one full revolution of the encoder.
	 */
	private static final double TICKS_PER_ROTATION = 515;
	/**
	 * The amount of wheel ticks per one centimeter of wheel travel.
	 */
	private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS * 2 * PI;
	private static final double ROTATION_EPSILON = toRadians(0.5);

	// Variables used in the acceleration profile.
	private static final double k_a_accelerating = 1. / 500.;
	private static final double k_a_decelerating = 1. / 1000.;
	private static final double k_error = 0.1;
	private static final double k_d_error = 0.005;
	private static final double k_v = 1 / RobotParameters.MAX_V_X;


	private static final double k_v_rot = 1 / RobotParameters.MAX_V_ROT;
	private static final double k_a_rot_accelerating = 55./500;
	private static final double k_a_rot_decelerating = 0;
	private static final double k_error_rot = 25.*0.1;
	private static final double k_d_error_rot = 10 * 0.005;


	/**
	 * A constant which the speed in the y direction is multiplied by
	 * in order to make the robot drive at an equal velocity for the same power
	 * when driving in the x and y directions
	 */
	private static final double DRIVE_Y_FACTOR = RobotParameters.MAX_V_X / RobotParameters.MAX_V_Y;

	private static final double[][] cornersRelativePosition = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};


	private final LinearOpMode opMode;

	/**
	 * Variable used to add multiple add to the telemetry dashboard.
	 */
	private final MultipleTelemetry multipleTelemetry;
	/**
	 * The instance of the robot's IMU.
	 */
	private final BNO055IMU imu;
	/**
	 * The motor which controls the front right wheel.
	 */
	private final DcMotor frontRight;
	/**
	 * The motor which controls the front left wheel.
	 */
	private final DcMotor frontLeft;
	/**
	 * The motor which controls the back right wheel.
	 */
	private final DcMotor backRight;
	/**
	 * The motor which controls the back left wheel.
	 */
	private final DcMotor backLeft;

	private double flPreviousTicks = 0;
	private double frPreviousTicks = 0;
	private double blPreviousTicks = 0;
	private double brPreviousTicks = 0;

	private final Pose positionCM = new Pose(0., 0., 0.);

	public double maxDrivePower = 1;

	public double wantedPosition;

	public final PositionLogger positionLogger; // Needs to be public to save the file from the opMode.
	private long lastCycleTime; // The time, in nanoseconds since the program began of the last time trackPosition was called.
	private long lastCycleDuration; // The duration, in nanoseconds, of the time between when trackPosition was called the last 2 times.

	private Pose targetPose = new Pose();

	public Pose getTargetPosition(){
		return new Pose(targetPose);
	}

	public long getLastCycleTime() {
		return lastCycleTime;
	}

	public long getLastCycleDuration() {
		return lastCycleDuration;
	}

	public Pose getPosition() {
		return new Pose(positionCM);
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public DrivingSystem(LinearOpMode opMode) {
		this.opMode = opMode;
		imu = initializeImu(opMode);
		multipleTelemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
		lastCycleTime = System.nanoTime();

		// Enable bulk reads
		List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
		for (LynxModule hub : allHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		// Get mecanum wheels interfaces
		frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
		frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
		backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
		backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

		// Makes the motors break when their power is set to zero, so they can better stop in place.
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Some motors are wired in reverse, so we must reverse them back.
		frontLeft.setDirection(DcMotor.Direction.REVERSE);
		backLeft.setDirection(DcMotor.Direction.REVERSE);

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

		axisMapConfigByte = (byte) (X_AXIS | Z_AXIS << 2 | Y_AXIS << 4); // swap z and y axis

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

//		frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//		frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//		backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//		backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	/**
	 * Resets the position of the robot to a given value.
	 *
	 * @program The new and correct real location of the robot in the board.
	 */
	public void resetStartLocation(PointD realLocation) {
		positionCM.x = realLocation.x;
		positionCM.y = realLocation.y;
	}

	/**
	 * Converts the robot Location to squares and
	 * calculates the deviation from the the center of the square that the robot is on.
	 *
	 * @program The Robot's Location in squares,
	 * the deviation of the robot from the center of the square it's on (0 <= x <= 1).
	 */
	public Pair<Pose, PointD> getSquareInformation() {
		Pair<Pose, PointD> SquareInformation = new Pair<>(new Pose(), new PointD());
		SquareInformation.first.x = positionCM.x / TILE_SIZE;
		SquareInformation.first.y = positionCM.y / TILE_SIZE;
		SquareInformation.first.angle = positionCM.angle;

		SquareInformation.second.x = SquareInformation.first.x % 1;
		SquareInformation.second.x -= signum(SquareInformation.second.x) / 2;
		SquareInformation.second.x *= 2;

		SquareInformation.second.y = SquareInformation.first.y % 1;
		SquareInformation.second.y -= signum(SquareInformation.second.y) / 2;
		SquareInformation.second.y *= 2;

		return SquareInformation;
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
	 * @return A double representing the robot's current angle.
	 */
	public double getCurrentAngle() {
		Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
		return orientation.firstAngle;
	}

	public Orientation getOrientation() {
		return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
	}

	/**
	 * Returns how far sideways and forward, in centimeters,
	 * the robot has moved and rotated since the last time that resetDistance() was called.
	 * Assumes the robot hasn't moved in any other directions.
	 *
	 * @return A Pose representing the robot's current position and angle.
	 */
	public Pose getDistancesOld() {
		Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);// TODO: bring back to previous version. This is just a test for performance to see the angle.

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
	 * @return PointD: Sum of movement Sideways, Sum of movement Forward; in cm.
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
	 * @return PointD: Sum of movement Sideways, Sum of movement Forward; in cm.
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
		double norm = max(max(abs(frontRightPower), abs(frontLeftPower)), max(abs(backRightPower), abs(backLeftPower))) / maxDrivePower;
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
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis(Pose Powers) {
		final double K = 0.03;

		Pair<Pose, PointD> mySquareInformation = getSquareInformation();
		Pose SquareLocation = mySquareInformation.first;
		PointD squareDeviation = mySquareInformation.second;

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
			;
		}

		multipleTelemetry.addData("xPos", squareDeviation.x);
		multipleTelemetry.addData("yPos", squareDeviation.y);
		multipleTelemetry.addData("Powers.x", Powers.x);
		multipleTelemetry.addData("Powers.y", Powers.y);
		multipleTelemetry.addData("rot", toDegrees(positionCM.angle));

		driveByAxis(Powers);
	}

	/**
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis2(Pose Powers) {
		final double K = 50.;

		Pair<Pose, PointD> mySquareInformation = getSquareInformation();
		Pose SquareLocation = mySquareInformation.first;
		PointD squareDeviation = mySquareInformation.second;

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

		multipleTelemetry.addData("squarePosition.x", SquareLocation.x);
		multipleTelemetry.addData("squarePosition.y", SquareLocation.y);
		multipleTelemetry.addData("squareDeviation.x", squareDeviation.x);
		multipleTelemetry.addData("squareDeviation.y", squareDeviation.y);


		driveByAxis(Powers);
	}

	/**
	 * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
	 */
	public void controlledDriveByAxis3(Pose Powers) {
		Pair<Pose, PointD> mySquareInformation = getSquareInformation();
		Pose SquareLocation = mySquareInformation.first;
		PointD squareDeviation = mySquareInformation.second;

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

		multipleTelemetry.addData("squarePosition.x", SquareLocation.x);
		multipleTelemetry.addData("squarePosition.y", SquareLocation.y);
		multipleTelemetry.addData("squareDeviation.x", squareDeviation.x);
		multipleTelemetry.addData("squareDeviation.y", squareDeviation.y);

		driveByAxis(Powers);
	}

	public PointD closestJunctionLocation() {
		Pair<Pose, PointD> mySquareInformation = getSquareInformation();
		Pose SquareLocation = mySquareInformation.first;
		PointD squareDeviation = mySquareInformation.second;
		PointD squareCenter = new PointD(SquareLocation.x - squareDeviation.x, SquareLocation.y - squareDeviation.y);

		PointD bestCornerPosition = new PointD();
		double bestRating = 0;
		for (int corner = 0; corner < 4; corner++) {
			PointD cornerSquarePosition = new PointD();
			cornerSquarePosition.x = squareCenter.x + cornersRelativePosition[corner][0] / 2;
			cornerSquarePosition.x = round(cornerSquarePosition.x);
			cornerSquarePosition.y = squareCenter.y + cornersRelativePosition[corner][1] / 2;
			cornerSquarePosition.y = round(cornerSquarePosition.y);


			if (abs(cornerSquarePosition.x) > 2 || abs(cornerSquarePosition.x) > 2) {
				continue;
			}

			PointD cornerDistance = new PointD();
			cornerDistance.x = cornerSquarePosition.x - SquareLocation.x;
			cornerDistance.y = cornerSquarePosition.y - SquareLocation.y;

			double rating = sqrt(cornerDistance.x * cornerDistance.x + cornerDistance.y * cornerDistance.y); // distance to the Pole
			rating *= (PI - normalizeAngle(SquareLocation.angle - atan(cornerDistance.x / cornerDistance.y))) / PI; // how much the robot is facing the Pole

			if (rating > bestRating) {
				cornerSquarePosition.x *= TILE_SIZE;
				cornerSquarePosition.y *= TILE_SIZE;
				bestCornerPosition = cornerSquarePosition;
			}
		}

		return bestCornerPosition;
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
		double CM_TO_INCH = 0.393701;
		double cycleFrequency = 1e9 / lastCycleDuration;
		FtcDashboard ftcDashboard = FtcDashboard.getInstance();
		TelemetryPacket packet = new TelemetryPacket();
		packet.fieldOverlay()
				.setFill("blue")
				.fillCircle(positionCM.x * CM_TO_INCH, positionCM.y * CM_TO_INCH, 5);
		packet.put("Is Alive", 1);
		ftcDashboard.sendTelemetryPacket(packet);

		multipleTelemetry.addData("x", positionCM.x);
		multipleTelemetry.addData("y", positionCM.y);
		multipleTelemetry.addData("rot", toDegrees(positionCM.angle));
		multipleTelemetry.addData("Cycle Frequency [Hz]: ", cycleFrequency);
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
		double forwardDistance = getDistancesOld().y;

		while (opMode.opModeIsActive() && Math.abs(forwardDistance) < distance) {
			Pose pose = getDistancesOld();
			forwardDistance = pose.y;
			double angleDeviation = AngleUnit.DEGREES.normalize(startAngle - pose.angle);
			double rotatePower = angleDeviation * ANGLE_DEVIATION_SCALAR;
			driveMecanum(new Pose(0, power, rotatePower));
			positionLogger.update();
//			printPosition();
//			multipleTelemetry.update();
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
			driveMecanum(new Pose(-power, 0, -angleDeviation * ANGLE_DEVIATION_SCALAR));
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
		final double powerScalar = 0.007 * 180 / PI;
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
		final double epsilon = 0.5;
		final double yTarget = positionCM.y + distance;
		;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = yTarget - positionCM.y;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && opMode.opModeIsActive()) {
			actPowers.y = 0.01 * deviation + 0.15 * signum(deviation);
			driveByAxis(actPowers);
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
		final double epsilon = 0.5;
		final double xTarget = positionCM.x + distance;
		;

		//PIDController myPIDController = new PIDController(0.1, 0.05, 0.2);
		double deviation = xTarget - positionCM.x;
		Pose actPowers = new Pose();

		while (abs(deviation) > epsilon && opMode.opModeIsActive()) {
			actPowers.x = 0.01 * deviation + 0.15 * signum(deviation);
			driveByAxis(actPowers);
			deviation = xTarget - positionCM.x;
		}
		stop();
	}

	public void driveToY(double Position) {
		driveY(Position - positionCM.y);
	}

	public void driveToX(double Position) {
		driveX(Position - positionCM.x);
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
			multipleTelemetry.update();
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

	public void driveForwardByProfile(AccelerationProfile accelerationProfile) {
		resetDistance();

		ElapsedTime elapsedTime = new ElapsedTime();


		PdffController controller = new PdffController(k_v, k_a_accelerating, k_a_decelerating, k_error, k_d_error);
		while (opMode.opModeIsActive() && elapsedTime.seconds() < accelerationProfile.finalTime()) {
			Pose pose = getDistancesOld();
			final double currentTime = elapsedTime.seconds();
			lastCycleTime = elapsedTime.nanoseconds();
			double targetPosition = accelerationProfile.getPosition(currentTime);
			double targetVelocity = accelerationProfile.getVelocity(currentTime);
			double targetAcceleration = accelerationProfile.acceleration(currentTime);
			double error = targetPosition - pose.y;

			double forwardPower = controller.getPower(currentTime, error, targetVelocity, targetAcceleration);
			driveMecanum(new Pose(0, forwardPower, 0));
			positionCM.y = pose.y;
			wantedPosition = targetPosition;
			positionLogger.update();
			printPosition();
			multipleTelemetry.addData("wantedPosition: ", wantedPosition);
			multipleTelemetry.update();
		}
		stop();
	}

	public void driveSidewaysByProfile(AccelerationProfile accelerationProfile){
		final double ANGLE_DEVIATION_SCALAR = 0.;
		resetDistance();
		double startAngle = getDistancesOld().angle;

		ElapsedTime elapsedTime = new ElapsedTime();

		double prevError = 0;
		double prevTime = 0;
		while (opMode.opModeIsActive() && elapsedTime.seconds() < accelerationProfile.finalTime()) {
			Pose pose = getDistancesOld();
			final double currentTime = elapsedTime.seconds();
			final double dt = currentTime - prevTime;
			lastCycleTime = elapsedTime.nanoseconds();
			double angleDeviation = AngleUnit.DEGREES.normalize(startAngle - pose.angle);
			double rotatePower = angleDeviation * ANGLE_DEVIATION_SCALAR;

			double targetPosition = accelerationProfile.getPosition(currentTime);
			double targetVelocity = accelerationProfile.getVelocity(currentTime);
			double targetAcceleration = accelerationProfile.acceleration(currentTime);
			double error = targetPosition - pose.x;
			double d_error_dt = (error - prevError) / dt;
			double k_a = targetAcceleration > 0 ? k_a_accelerating : k_a_decelerating;

			double sidewaysPower = k_v * targetVelocity + k_a * targetAcceleration + k_error * error + k_d_error * d_error_dt;
			driveMecanum(new Pose(sidewaysPower, 0, rotatePower));
			positionCM.x = pose.x;
			wantedPosition = targetPosition;
			positionLogger.update();
			printPosition();
			multipleTelemetry.addData("wantedPosition: ", wantedPosition);
			multipleTelemetry.update();

			prevError = error;
			prevTime = currentTime;
		}
		stop();
	}



	public void rotateByAccelerationProfile(AccelerationProfile accelerationProfile){
		final double ANGLE_DEVIATION_SCALAR = 0.;
		resetDistance();
		double startAngle = getDistancesOld().angle;

		ElapsedTime elapsedTime = new ElapsedTime();

		double prevError = 0;
		double prevTime = 0;
		while (opMode.opModeIsActive() && elapsedTime.seconds() < accelerationProfile.finalTime()) {
			Pose pose = getDistancesOld();
			pose.angle = Math.toRadians(pose.angle);
			final double currentTime = elapsedTime.seconds();
			final double dt = currentTime - prevTime;
			lastCycleTime = elapsedTime.nanoseconds();

			double targetAngle = accelerationProfile.getPosition(currentTime);
			double targetVelocity = accelerationProfile.getVelocity(currentTime);
			double targetAcceleration = accelerationProfile.acceleration(currentTime);
			double error = normalizeAngle(targetAngle - pose.angle);
			double d_error_dt = (error - prevError) / dt;
			double k_a = targetAcceleration > 0 ? k_a_rot_accelerating : k_a_rot_decelerating;

			double rotatePower = k_v_rot * targetVelocity + k_a * targetAcceleration + k_error_rot * error + k_d_error_rot * d_error_dt;
			driveMecanum(new Pose(0, 0, rotatePower));
			positionCM.angle = pose.angle;
			wantedPosition = Math.toDegrees(targetAngle);
			positionLogger.update();
			printPosition();
			multipleTelemetry.addData("wantedPosition: ", wantedPosition);
			multipleTelemetry.update();

			prevError = error;
			prevTime = currentTime;
		}
		stop();
	}

	public void driveByPath(Trajectory traj){
		ElapsedTime elapsedTime = new ElapsedTime();

		while (opMode.opModeIsActive() && elapsedTime.seconds() < traj.getTotalTime()) {
			final double currentTime = elapsedTime.seconds();
			final double k_pointDeviation = 0;
			final double k_angleDeviation = 1/toRadians(5);

			Pose currentPose = new Pose(positionCM.x,positionCM.y,getCurrentAngle());
			Pose targetPose = traj.getPose(currentTime);
			Pose deviation = new Pose(
					(targetPose.x - currentPose.x)*k_pointDeviation,
					(targetPose.y - currentPose.y)*k_pointDeviation,
					normalizeAngle(0 - currentPose.angle)*k_angleDeviation
			);

			this.targetPose = targetPose;

			Pose powers = traj.getPowers(elapsedTime.seconds());
			driveByAxis(new Pose(
					powers.x + deviation.x,
					powers.y + deviation.y,
					powers.angle + deviation.angle));
//			driveMecanum(new Pose((powers.x+deviation.x), (powers.y+deviation.y), (powers.angle+deviation.angle)));
			positionLogger.update();
			printPosition();
			opMode.telemetry.addData("track position", positionCM.x + "," + positionCM.y);
			opMode.telemetry.update();
		}
		stop();
	}


}
