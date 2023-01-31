package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.Pose;

/**
 * A class for handling tracking the robot's position.
 */
public class BADTrackingSystem {
	/**
	 * The radius of the wheels in centimeters.
	 */
	private static final double WHEEL_RADIUS = 4.8;
	/**
	 * The number of ticks per full revolution of the odometry wheels.
	 */
	private static final double TICKS_PER_ROTATION = 515;
	/**
	 * Conversion factor from ticks to centimeters.
	 */
	private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS * 2 * PI;

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
	 * The IMU.
	 */
	private final BNO055IMU imu;

	// Keeps the robot's odometry wheel's previous positions
	private double flPreviousTicks = 0;
	private double frPreviousTicks = 0;
	private double blPreviousTicks = 0;
	private double brPreviousTicks = 0;

	/**
	 * The robot's current position.
	 */
	private final Pose position = new Pose(0., 0., 0.);

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public BADTrackingSystem(LinearOpMode opMode) {
		this.opMode = opMode;
		imu = initializeImu(opMode);

		//Get odometry pod interfaces
		frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
		frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
		backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
		backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

		// Reset the distances measured by the motors
		resetDistance();
	}

	/**
	 * @param opMode The current opMode running on the robot.
	 * @return An initialized IMU.
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
	 * Resets the distance measured on all encoders.
	 * Should always be called before initializing the robot.
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
	 * Tracks the robot's position and updates the position variable.
	 * Pose Exponentials Method and explanation from
	 * <a href="https://gm0.org/en/latest/docs/software/concepts/odometry.html#using-pose-exponentials">this site</a>.
	 */
	public void trackPosition() {
		// Get the current position of the odometry wheels and the current angle
		double flCurrentTicks = frontLeft.getCurrentPosition();
		double frCurrentTicks = frontRight.getCurrentPosition();
		double blCurrentTicks = backLeft.getCurrentPosition();
		double brCurrentTicks = backRight.getCurrentPosition();
		Orientation orientation = imu.getAngularOrientation(
				AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);

		// The displacement of each wheel and the angle change
		final double frontLeftDisplacement = (flCurrentTicks - flPreviousTicks) * CM_PER_TICK;
		final double frontRightDisplacement = (frCurrentTicks - frPreviousTicks) * CM_PER_TICK;
		final double backLeftDisplacement = (blCurrentTicks - blPreviousTicks) * CM_PER_TICK;
		final double backRightDisplacement = (brCurrentTicks - brPreviousTicks) * CM_PER_TICK;
		final double angleChange = position.angle - orientation.firstAngle;

		// Calculating the robot's displacement and rotation
		final double centerDisplacement = (frontLeftDisplacement + frontRightDisplacement +
				backLeftDisplacement + backRightDisplacement) / 4;
		final double horizontalDisplacement = (frontRightDisplacement - frontLeftDisplacement +
				backLeftDisplacement - backRightDisplacement) / 4;

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
		blPreviousTicks = blCurrentTicks;
		brPreviousTicks = brCurrentTicks;
	}

	/**
	 * Prints the robot's current position to the telemetry.
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
