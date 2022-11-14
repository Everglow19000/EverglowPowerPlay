package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.PointD;

import static java.lang.Double.max;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.PI;

public class DrivingSystem {
    public static final double RADIANS_TO_DEGREES = 180.0 / PI;
    private static final double WHEEL_RADIUS_CM = 4.8;
    private static final double TICKS_PER_ROTATION = 515;
    private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS_CM * 2 * PI;

    private final LinearOpMode opMode;

    private final BNO055IMU imu;
    private final DcMotor frontRight;
    private final DcMotor frontLeft;
    private final DcMotor backRight;
    private final DcMotor backLeft;

    private double previousAngle = 0;
    private PointD PreviousDistances = new PointD(0., 0.);
    private PointD PositionCM = new PointD(0., 0.);


    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        imu = initializeImu(opMode);

        // Creates objects to control the motors
        frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
        backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

        // Makes the motors break when their power is set to zero, so they can better stop in place.
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Some motors are wired in reverse, so we must reverse them back
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset Distance
        resetDistance();
    }

    /**
     * Creates an IMU object and calibrates it correctly to its actual face.
     *
     * @param opMode the current opMode
     * @return an BNO055IMU object
     */
    private static BNO055IMU initializeImu(LinearOpMode opMode) {
        // Create IMU
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

        // copied from https://ftcforum.firstinspires.org/forum/ftc-technology/53812-mounting-the-revhub-vertically
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        opMode.sleep(100); //Changing modes requires a delay before doing anything else
        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        opMode.sleep(100); //Changing modes again requires a delay

        while (!imu.isGyroCalibrated()) {
            // wait for the gyroscope calibration
            opMode.sleep(10);
        }
        return imu;
    }


    /*
     * Resets the distance measured on all encoders,
     * should always called before initializing the robot.
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
     * Given any angle, normalizes it such that it is between -180 and 180 RADIANS,
     * increasing or decreasing by 360 RADIANS to make it so.
     *
     * @param angle Random angle.
     * @return The angle normalized (-180° < angle < 180°).
     */
    public static double normalizeAngle(double angle) {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }


    /**
     * Gets the robots current angle and returns it. Angle is measured relative to the robot's starting angle, with positive angles being counterclockwise.
     *
     * @return double representing the robot's current angle.
     */
    public double getCurrentAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        return orientation.firstAngle;
    }


    /**
     * Updates the position of the robot from the wheel positions.
     *
     * @return Point: Sum of movement Sideways, Sum of movement Forward; in cm.
     */
    public PointD updateDistances() {
        final double fLPosition = frontLeft.getCurrentPosition();
        final double fRPosition = frontRight.getCurrentPosition();
        final double bLPosition = backLeft.getCurrentPosition();
        final double bRPosition = backRight.getCurrentPosition();

        final double returnXValue = (fRPosition - fLPosition + bLPosition - bRPosition) / 4. * CM_PER_TICK;
        final double returnYValue = (fLPosition + fRPosition + bLPosition + bRPosition) / 4. * CM_PER_TICK;

        PreviousDistances.x = returnXValue;
        PreviousDistances.y = returnYValue;

        return new PointD(returnXValue, returnYValue);
    }


    /**
     * Gets the position of the robot from the wheel positions.
     *
     * @return Point: Sum of movement Sideways, Sum of movement Forward; in cm.
     */
    public PointD getDistances() {
        final double fLPosition = frontLeft.getCurrentPosition();
        final double fRPosition = frontRight.getCurrentPosition();
        final double bLPosition = backLeft.getCurrentPosition();
        final double bRPosition = backRight.getCurrentPosition();

        final double returnXValue = (fRPosition - fLPosition + bLPosition - bRPosition)  / 4. * CM_PER_TICK;
        final double returnYValue = (fLPosition + fRPosition + bLPosition + bRPosition)  / 4. * CM_PER_TICK;

        return new PointD(returnXValue, returnYValue);
    }


    /**
     * Drives the robot on a mechanism drive.
     *
     * @param x   vertical power, positive is left, negative is right
     * @param y   horizontal power, positive is forward, negative is backwards.
     * @param rot rotational power, positive is counter clockwise, negative is clockwise.
     */
    public void driveMecanum(double x, double y, double rot) {
        double frontRightPower = y + x + rot;
        double frontLeftPower = y - x - rot;
        double backRightPower = y - x + rot;
        double backLeftPower = y + x - rot;

        // the function setPower only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1.
        double norm = Math.max(Math.max(frontRightPower, frontLeftPower), Math.max(backRightPower, backLeftPower));
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
        driveMecanum(0, 0, 0);
    }


    /**
     * Keeps track of robot position on the field
     */
    public void trackPosition() {
        double deltaXCM = -PreviousDistances.x;
        double deltaYCM = -PreviousDistances.y;
        final PointD currentDistances = updateDistances();

        deltaXCM += currentDistances.x;
        deltaYCM += currentDistances.y;

        final double currentAngle = getCurrentAngle();
        final double angleAverage = (currentAngle + previousAngle) / 2;
        previousAngle = currentAngle;

        PositionCM.x += deltaYCM * sin(angleAverage) + deltaXCM * cos(angleAverage);
        PositionCM.y += deltaYCM * cos(angleAverage) - deltaXCM * sin(angleAverage);


        opMode.telemetry.addData("X POS:", PositionCM.x);
        opMode.telemetry.addData("Y POS:", PositionCM.y);
        opMode.telemetry.addData("ANGLE:", currentAngle * RADIANS_TO_DEGREES);
        opMode.telemetry.update();
    }


    /**
     * @return The distance the robot has moved forward, in cm, since the last time that resetDistance() was called.
     * Assumes the robot hasn't moved in any other directions.
     */
    public double getForwardDistance() {
        return (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()
                + backLeft.getCurrentPosition() + backRight.getCurrentPosition())
                / 4. * CM_PER_TICK;
    }


    /**
     * Drives the robot straight a given distance.
     *
     * @param distance How far the robot should go, measured in cm. Positive is forwards, negative is backwards.
     * @param power    How much power should be given to the motor, from 0 to 1.
     */
    public void driveStraight(double distance, double power) {
        final double ANGLE_DEVIATION_SCALAR = 0.05;

        // if we're traveling a negative distance, that means traveling backwards,
        // so the power should be inverted and so should the distance.
        if (distance < 0) {
            distance = -distance;
            power = -power;
        }

        resetDistance();
        double startAngle = getCurrentAngle();
        double forwardDistance = getForwardDistance();

        while (abs(forwardDistance) < distance) {
            forwardDistance = getForwardDistance();
            double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
            driveMecanum(0, power, angleDeviation * ANGLE_DEVIATION_SCALAR);
        }


        stop();
    }


    /**
     * @return The distance the robot has moved sideways, in cm, since the last time that resetDistance() was called.
     * Assumes the robot hasn't moved in any other directions.
     */
    public double getSidewaysDistance() {
        return (frontRight.getCurrentPosition() - frontLeft.getCurrentPosition()
                + backLeft.getCurrentPosition() - backRight.getCurrentPosition())
                / 4. * CM_PER_TICK;
    }


    /**
     * Drives the robot sideways a given distance.
     *
     * @param distance How far the robot should go, measured in cm. Positive is right, negative is left.
     * @param power    How much power should be given to the motor, from 0 to 1.
     */
    public void driveSideways(double distance, double power) {
        double ANGLE_DEVIATION_SCALAR = 0.05;

        // if we're traveling a negative distance, that means traveling left,
        // so the power should be inverted and so should the distance.
        if (distance < 0) {
            distance = -distance;
            power = -power;
        }

        resetDistance();
        double startAngle = getCurrentAngle();
        double sidewaysDistance = getSidewaysDistance();
        while (abs(sidewaysDistance) < distance) {
            sidewaysDistance = getSidewaysDistance();
            double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
            driveMecanum(power, 0, -angleDeviation * ANGLE_DEVIATION_SCALAR);
        }
        stop();
    }


    /**
     * Rotates the robot a given number of RADIANS.
     * A positive angle means clockwise rotation, a negative angle is counterclockwise. TODO: fix this
     *
     * @param angle An angle to rotate the robot to.
     */
    public void rotate(double angle) {
        final double powerScalar = 0.007;
        final double minPower = 0.2;
        final double epsilon = 0.5;

        double currentAngle = getCurrentAngle();

        // Angles must always be between -180 and 180 RADIANS.
        // The function used below adds or subtracts 360 RADIANS from the angle
        // so that it's always in the good range.
        double targetAngle = normalizeAngle(currentAngle + angle);

        double deviation = normalizeAngle(targetAngle - currentAngle);

        while (abs(deviation) > epsilon) { // once the angular error is less than 0.5 RADIANS, we have arrived
            currentAngle = getCurrentAngle();
            deviation = normalizeAngle(targetAngle - currentAngle);
            // the power is proportional to the deviation, but may not go below minPower.
            double rotatePower = deviation * deviation * powerScalar + minPower * Math.signum(deviation);
            driveMecanum(0, 0, rotatePower);
        }
        stop();
    }


    /**
     * Drives the Robot in straight line to given position on the board(x, y, angle)
     *
     * @param targetPointCm is the point to move to
     * @param targetAngle   is the angle to finish in
     */
    public void moveTo(PointD targetPointCm, double targetAngle) {
        final double powerScalar = 0.007;
        final double angleScalar = 0.01;
        final double minPower = 0.2;
        final double minAnglePower = 0.15;
        final double epsilon = 0.5;
        final double angleEpsilon = 2.0;

        while (abs(targetPointCm.x - PositionCM.x) > epsilon || abs(targetPointCm.y - PositionCM.y) > epsilon | abs(getCurrentAngle() - targetAngle) > angleEpsilon) {
            final PointD currentPosition = getDistances();
            final double currentAngle = getCurrentAngle();
            final double xDeviation = targetPointCm.x - currentPosition.x;
            final double yDeviation = targetPointCm.y - currentPosition.y;
            final double angleDeviation = normalizeAngle(targetAngle - currentAngle);

            driveMecanum(max(minPower, (xDeviation * cos(currentAngle) + yDeviation * sin(currentAngle)) * (xDeviation * cos(currentAngle) + yDeviation * sin(currentAngle)) * powerScalar),
                    max(minPower, (-xDeviation * sin(currentAngle) + yDeviation * cos(currentAngle)) * (-xDeviation * sin(currentAngle) + yDeviation * cos(currentAngle)) * powerScalar),
                    max(minAnglePower, angleDeviation * angleDeviation * angleScalar));
        }

        stop();
    }


}
