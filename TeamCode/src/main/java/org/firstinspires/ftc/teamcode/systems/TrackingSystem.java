package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.round;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.CM_PER_TICK;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.LATERAL_DISTANCE;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;

/**
 * A class for handling tracking the robot's position.
 */
public class TrackingSystem {
	/**
	 * The current opMode running on the robot.
	 */
	private final LinearOpMode opMode;

	/**
	 * The front left odometry wheel.
	 */
	private final DcMotor frontLeft;
	/**
	 * The front right odometry wheel.
	 */
	private final DcMotor frontRight;
	/**
	 * The back odometry wheel.
	 */
	private final DcMotor back;

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

	/**
	 * @param opMode The current opMode running on the robot.
	 */
	public TrackingSystem(LinearOpMode opMode) {
		this.opMode = opMode;

		// Get odometry pod interfaces (names because of cable management)
		frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
		frontRight = opMode.hardwareMap.get(DcMotor.class, "back_right");
		back = opMode.hardwareMap.get(DcMotor.class, "gWheel");

		// Reset the distances measured by the motors
		flPreviousTicks = frontLeft.getCurrentPosition();
		frPreviousTicks = frontRight.getCurrentPosition();
		bPreviousTicks = back.getCurrentPosition();
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
		trackPosition();
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
		opMode.telemetry.addData("fl location: ", flCurrentTicks * CM_PER_TICK);
		opMode.telemetry.addData("fr location: ", frCurrentTicks * CM_PER_TICK);
		opMode.telemetry.addData("b location: ", bCurrentTicks * CM_PER_TICK);
		printPosition();

		// The displacement of each wheel
		final double frontLeftDisplacement = (flCurrentTicks - flPreviousTicks) * CM_PER_TICK;
		final double frontRightDisplacement = (frCurrentTicks - frPreviousTicks) * CM_PER_TICK;
		final double backDisplacement = (bCurrentTicks - bPreviousTicks) * CM_PER_TICK;

		// Calculating the robot's displacement and rotation
		final double angleChange = (frontLeftDisplacement - frontRightDisplacement) / LATERAL_DISTANCE;
		final double centerDisplacement = (frontLeftDisplacement + frontRightDisplacement) / 2;
		final double horizontalDisplacement = backDisplacement - FORWARD_OFFSET * angleChange;

		// Temp variable for readability
		final double angleCos = cos(position.angle), angleSin = sin(position.angle);

		// The angle was removed from the matrices and they were simplified to have only two rows
		// because it wasn't actually used in the calculates.
		final double[][] matrix1 = {{angleCos, -angleSin}, {angleSin, angleCos}};
		final double[][] matrix2 = {{sinc(angleChange), cosc(angleChange)},
				{-cosc(angleChange), sinc(angleChange)}};
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
		return new PointD(position.x / TILE_SIZE, position.y / TILE_SIZE);
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

			PointD cornerDistance = new PointD(cornerSquarePosition.x - position.x, cornerSquarePosition.y - position.y);

			double angleTo = abs(angleTo(cornerDistance) - position.angle);
			if (angleTo < smallestAngle) {
				bestCornerPosition = cornerSquarePosition;
				smallestAngle = angleTo;
			}
		}

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

		// Note that our x and y coordinate system is flipped because we define forward as x and sideways as y, so we must unflip now
		final double y = position.x * INCH_TO_CM;
		final double x = position.y * INCH_TO_CM;

		opMode.telemetry.addData("x: ", position.x);
		opMode.telemetry.addData("y: ", position.y);
		opMode.telemetry.addData("Angle: ", toDegrees(position.angle));

		TelemetryPacket packet = new TelemetryPacket();

		double dx1 = robot_width * cos(position.angle);
		double dx2 = -robot_height * sin(position.angle);
		double dy1 = robot_width * sin(position.angle);
		double dy2 = robot_height * cos(position.angle);

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


