package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.utils.RobotParameters.CM_PER_TICK;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;

/**
 * A class for handling tracking the robot's position.
 */
public class TrackingSystem {

	public boolean enabled;
	public boolean doPrint = false;

	/**
	 * The front left odometry wheel.
	 */
	private DcMotor frontLeft;
	/**
	 * The front right odometry wheel.
	 */
	private DcMotor frontRight;
	/**
	 * The back odometry wheel.
	 */
	private DcMotor back;

	/**
	 * The robot's current position.
	 */
	private final Pose position = new Pose(0., 0., 0.);

	/**
	 * Position matrix for the tile corners.
	 */
	private final double[][] cornersRelativePosition = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

	// Keeps the robot's odometry wheel's previous positions
	private double flPreviousTicks;
	private double frPreviousTicks;
	private double bPreviousTicks;

	private BNO055IMU imu;

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
	 * @param opMode The current opMode running on the robot.
	 */
	public TrackingSystem(LinearOpMode opMode, boolean enabled) {
		this.enabled = enabled;
		if (enabled) {
			// Get odometry pod interfaces (names because of cable management)
			frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
			frontRight = opMode.hardwareMap.get(DcMotor.class, "back_right");
			back = opMode.hardwareMap.get(DcMotor.class, "gWheel");

			// Reset the distances measured by the motors
			flPreviousTicks = frontLeft.getCurrentPosition();
			frPreviousTicks = frontRight.getCurrentPosition();
			bPreviousTicks = back.getCurrentPosition();

			imu = initializeImu(opMode);
		}
	}

	public double getImuAngle() {
		Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
		return orientation.firstAngle;
	}

	/**
	 * @return The robot's current position as a Pose object.
	 * Flips the x and y values because the functions which use this method expect the x value to be the first value.
	 */
	@SuppressWarnings("SuspiciousNameCombination")
	public Pose getPosition() {
		return new Pose(position.y, position.x, position.angle);
	}

	/**
	 * Ticks the tracking system.
	 */
	public void tick() {
		if (enabled) {
			trackPosition();
		}

		if (doPrint) {
			printPosition();
		}
	}

	public void resetPosition(Pose location) {
		// flipping x and y, because x and y are swapped in our internal structure
		position.x = location.y;
		position.y = location.x;
		position.angle = location.angle;
	}

	/**
	 * Tracks the robot's position and updates the position variable.
	 * Pose Exponentials Method and explanation from
	 * <a href="https://gm0.org/en/latest/docs/software/concepts/odometry.html#using-pose-exponentials">this site</a>.
	 */
	private void trackPosition() {
		// Get the current position of the odometry wheels
		final double flCurrentTicks = frontLeft.getCurrentPosition();
		final double frCurrentTicks = frontRight.getCurrentPosition();
		final double bCurrentTicks = back.getCurrentPosition();

		//Log all info
		//opMode.telemetry.addData("fl location: ", flCurrentTicks * CM_PER_TICK);
		//opMode.telemetry.addData("fr location: ", frCurrentTicks * CM_PER_TICK);
		//opMode.telemetry.addData("b location: ", bCurrentTicks * CM_PER_TICK);

		// The displacement of each wheel
		final double frontLeftDisplacement = (flCurrentTicks - flPreviousTicks) * CM_PER_TICK;
		final double frontRightDisplacement = (frCurrentTicks - frPreviousTicks) * CM_PER_TICK;
		final double backDisplacement = (bCurrentTicks - bPreviousTicks) * CM_PER_TICK;

		// Calculating the robot's displacement and rotation
//		final double angleChange = (frontLeftDisplacement - frontRightDisplacement) / LATERAL_DISTANCE;

		final double currentAngle = getImuAngle();
		final double angleChange = currentAngle - position.angle;

		final double centerDisplacement = -(frontLeftDisplacement + frontRightDisplacement) / 2;
		final double horizontalDisplacement = backDisplacement - FORWARD_OFFSET * angleChange;

		// Temp variable for readability
		final double angleCos = cos(position.angle), angleSin = sin(position.angle);

		// The currentAngle was removed from the matrices and they were simplified to have only two rows
		// because it wasn't actually used in the calculates.
		final double[][] matrix1 = {{angleCos, -angleSin}, {angleSin, angleCos}};
		final double[][] matrix2 = {
				{sinc(angleChange), cosc(angleChange)},
				{-cosc(angleChange), sinc(angleChange)}
		};
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
		position.angle = currentAngle;
//		position.angle += angleChange;

		// Update the previous ticks
		flPreviousTicks = flCurrentTicks;
		frPreviousTicks = frCurrentTicks;
		bPreviousTicks = bCurrentTicks;
	}

	/**
	 * Calculates the sin() of a number then divides the result by the number
	 * if the number isn't equal to zero.
	 *
	 * @param num any number.
	 * @return a positive number under 1 or zero.
	 */
	private double sinc(double num) {
		if (num == 0)
			return 1;
		return sin(num) / num;
	}

	/**
	 * Calculates the cos() of a number then divides the result by the number
	 * if the number isn't equal to zero.
	 *
	 * @param num any number.
	 * @return a positive number under 1 or zero.
	 */
	private double cosc(double num) {
		if (num == 0)
			return 0;
		return (cos(num) - 1) / num;
	}

	/**
	 * Get the angle of a point on the board in relation to the robot.
	 *
	 * @param distance The distance between the robot and the target in the field axis.
	 * @return The angle of the target relative to the robot in the board axis.
	 */
	public double angleTo(PointD distance) {
		double angle = atan(distance.x / distance.y);
		if (distance.x < 0) angle -= PI;
		return angle;
	}

	/**
	 * @return The robot's current position as a Point2D object, measured in tiles.
	 */
	public PointD getTileLocation() {
		Pose corPosition = getPosition();
		return new PointD(corPosition.x / TILE_SIZE, corPosition.y / TILE_SIZE);
	}

	/**
	 * @return The center of the tile the robot is currently on.
	 */
	public PointD getTileCenter() {
		final PointD tileLocation = getTileLocation();

		PointD tileCenter = new PointD(tileLocation.x + 0.5 * signum(tileLocation.x), tileLocation.y + 0.5 * signum(tileLocation.y));
		tileCenter.x = round(tileCenter.x);
		tileCenter.y = round(tileCenter.y);
		tileCenter.x -= 0.5 * signum(tileLocation.x);
		tileCenter.y -= 0.5 * signum(tileLocation.y);

		return tileCenter;
	}

	/**
	 * @return The robot's deviation from the center of the tile it is currently on.
	 */
	public PointD getTileDeviation() {
		final PointD tileLocation = getTileLocation();
		final PointD tileCenter = getTileCenter();

		return new PointD(tileLocation.x - tileCenter.x, tileLocation.y - tileCenter.y);
	}

	/**
	 * @return The location of the closest pole to the robot.
	 */
	public PointD getClosestPoleLocation() {
		PointD tileLocation = getTileLocation();
		PointD squareCenter = getTileCenter();
		PointD bestCornerPosition = new PointD();

		double smallestAngle = 2 * PI;
		for (int corner = 0; corner < 4; corner++) {
			PointD cornerSquarePosition = new PointD();
			cornerSquarePosition.x = squareCenter.x + cornersRelativePosition[corner][0] / 2;
			cornerSquarePosition.x = round(cornerSquarePosition.x);

			cornerSquarePosition.y = squareCenter.y + cornersRelativePosition[corner][1] / 2;
			cornerSquarePosition.y = round(cornerSquarePosition.y);

			if (abs(cornerSquarePosition.x) > 2 || abs(cornerSquarePosition.x) > 2 ||
					(cornerSquarePosition.x % 2 == 0 && cornerSquarePosition.y % 2 == 0)) {
				continue;
			}

			PointD cornerDistance = new PointD(cornerSquarePosition.x - tileLocation.x, cornerSquarePosition.y - tileLocation.y);

			double angleDiff = abs(angleTo(cornerDistance) - position.angle);
			if (angleDiff < smallestAngle) {
				bestCornerPosition = cornerSquarePosition;
				smallestAngle = angleDiff;
			}
			/*
			opMode.telemetry.addData("CornerInd", corner);
			opMode.telemetry.addData("cornerSquarePosition.x", cornerSquarePosition.x);
			opMode.telemetry.addData("cornerSquarePosition.y", cornerSquarePosition.y);
			opMode.telemetry.addData("cornerDistance.x", cornerDistance.x);
			opMode.telemetry.addData("cornerDistance.y", cornerDistance.y);
			opMode.telemetry.addData("angleTo(cornerDistance)", angleTo(cornerDistance));
			opMode.telemetry.addData("angleDiff", angleDiff);
			*/

		}

		SystemCoordinator.instance.opMode.telemetry.update();

		bestCornerPosition.x *= TILE_SIZE;
		bestCornerPosition.y *= TILE_SIZE;

		return bestCornerPosition;
	}

	/**
	 * Prints the robot's current position to the telemetry and the FTCDashboard.
	 */
	private void printPosition() {
		final double INCH_TO_CM = 0.39;
		final double robot_width = 38 * INCH_TO_CM;
		final double robot_height = 47 * INCH_TO_CM;

		Pose currentPosition = getPosition();
		// Pose currentPosition = position;

		// Note that our x and y coordinate system is flipped because we define forward as x and sideways as y, so we must unflip now
		final double y = currentPosition.y * INCH_TO_CM;
		final double x = currentPosition.x * INCH_TO_CM;

		SystemCoordinator.instance.opMode.telemetry.addData("x: ", currentPosition.x);
		SystemCoordinator.instance.opMode.telemetry.addData("y: ", currentPosition.y);
		SystemCoordinator.instance.opMode.telemetry.addData("Angle: ", toDegrees(currentPosition.angle));
//		opMode.telemetry.addData("Imu Angle: ", toDegrees(getImuAngle()));

		TelemetryPacket packet = new TelemetryPacket();

		double dx1 = robot_width * cos(currentPosition.angle);
		double dx2 = -robot_height * sin(currentPosition.angle);
		double dy1 = robot_width * sin(currentPosition.angle);
		double dy2 = robot_height * cos(currentPosition.angle);

		packet.fieldOverlay().fillPolygon(new double[]{
						x + dx1 + dx2,
						x + dx1 - dx2,
						x - dx1 - dx2,
						x - dx1 + dx2
				},
				new double[]{
						y + dy1 + dy2,
						y + dy1 - dy2,
						y - dy1 - dy2,
						y - dy1 + dy2
				});
		FtcDashboard.getInstance().sendTelemetryPacket(packet);
	}
}


