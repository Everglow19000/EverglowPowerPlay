package org.firstinspires.ftc.teamcode.systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PosePIDController;

/**
 * A class for handling the driving of the robot.
 */
public class DrivingSystem {

    /**
     * Marker marking PID activated methods.
     */
    @interface PID{}

    /**
     * Enum to indicate which robot we are currently running. The IMU initialization is
     * unique for each robot, since the Control Hub's orientation is different.
     */
    private enum Robot {
        ARMADILLO, NEW_ROBOT
    }
    private static final Robot robot = Robot.ARMADILLO;

    private static final double WHEEL_RADIUS_CM = 4.8;
    private static final double TICKS_PER_ROTATION = 515;
    private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS_CM * 2 * PI;

    private final LinearOpMode opMode;

    private final BNO055IMU imu;
    public final DcMotor frontRight;
    public final DcMotor frontLeft;
    public final DcMotor backRight;
    public final DcMotor backLeft;

    private double flPreviousTicks = 0;
    private double frPreviousTicks = 0;
    private double blPreviousTicks = 0;
    private double brPreviousTicks = 0;

    private final double ROTATION_EPSILON = toRadians(0.5);

    private Pose positionCM = new Pose(0., 0., 0.);

    /**
     * @param opMode The Current opmode the robot is running with.
     */
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

        // Some motors are wired in reverse, so we must reverse them back.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset Distance
        resetDistance();
    }

    /**
     * Creates an IMU object and calibrates it correctly to the current orientation.
     *
     * @param opMode the current opMode
     * @return an BNO055IMU object
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
        if (robot == Robot.ARMADILLO) {
            // armadillo requires extra configuration for its IMU.
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
        }
        return imu;
    }

    /**
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
     * Given any angle, normalizes it such that it is between PI and PI RADIANS,
     * increasing or decreasing by 2 * PI RADIANS to make it so.
     *
     * @param angle Random angle.
     * @return The angle normalized (-PI < angle < PI).
     */
    private static double normalizeAngle(double angle) {
        while (angle >= PI) angle -= 2.0 * PI;
        while (angle < -PI) angle += 2.0 * PI;
        return angle;
    }

    /**
     * Gets the robots current angle and returns it.
     * Angle is measured relative to the robot's starting angle, with positive angles being counterclockwise.
     *
     * @return double representing the robot's current angle.
     */
    public double getCurrentAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        return orientation.firstAngle;
    }

    /**
     * Gets the movement of the robot in the robot's axis since the last tracked position.
     *
     * @return PointD: Sum of movement Sideways, Sum of movement Forward; in cm.
     */
    public PointD getDistances() {
        final double fLChange = frontLeft.getCurrentPosition() - flPreviousTicks;
        final double fRChange = frontRight.getCurrentPosition() - frPreviousTicks;
        final double bLChange = backLeft.getCurrentPosition() - blPreviousTicks;
        final double bRChange = backRight.getCurrentPosition() - brPreviousTicks;

        PointD movementChange = new PointD();
        movementChange.x = (-fLChange + fRChange + bLChange - bRChange) / 4. * CM_PER_TICK;
        movementChange.y = (fLChange + fRChange + bLChange + bRChange) / 4. * CM_PER_TICK;

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

        return movementChange;
    }

    /**
     * Drives the robot and keeps track of it's position.
     * Gets called multiple times per second.
     */
    public void driveMecanum(Pose powers) {
        //Determine how much power each motor should have.
        double frontRightPower = powers.y + powers.x + powers.angle;
        double frontLeftPower = powers.y - powers.x - powers.angle;
        double backRightPower = powers.y - powers.x + powers.angle;
        double backLeftPower = powers.y + powers.x - powers.angle;

        // The function setPower only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1.
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
     * Drives the robot in the given orientation i the driver's axis and keeps track of it's position.
     */
    public void driveByAxis(Pose Powers) {
        final double currentAngle = getCurrentAngle();
        final double cosAngle = cos(currentAngle);
        final double sinAngle = sin(currentAngle);

        Pose mecanumPowers = new Pose(
                cosAngle * Powers.x - sinAngle * Powers.y,
                cosAngle * Powers.y + sinAngle * Powers.x,
                Powers.angle);


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
    }

    /**
     * Drives the robot straight a given distance.
     *
     * @param distance How far the robot should go, measured in cm.
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
        double forwardDistance = updateDistances().y;

        while (abs(forwardDistance) < distance) {
            double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
            driveMecanum(new Pose(0, power, angleDeviation * ANGLE_DEVIATION_SCALAR));
            forwardDistance = updateDistances().y;
        }
        stop();
    }

    /**
     * Drives the robot sideways a given distance. Positive direction is rightwards.
     *
     * @param distance How far the robot should go, measured in cm.
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
        double sidewaysDistance = updateDistances().x;
        while (abs(sidewaysDistance) < distance) {
            double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
            driveMecanum(new Pose(power, 0, -angleDeviation * ANGLE_DEVIATION_SCALAR));
            sidewaysDistance = updateDistances().x;
        }
        stop();
    }

    /**
     * Rotates the robot a given number of radians.
     * A positive angle means clockwise rotation, a negative angle is counterclockwise. TODO: fix this
     *
     * @param angle An angle to rotate the robot to.
     */
    public void rotate(double angle) {
        final double powerScalar = 0.007;
        final double minPower = 0.2;

        double currentAngle = getCurrentAngle();

        // Angles must always be between -180 and 180 RADIANS.
        // The function used below adds or subtracts 360 RADIANS from the angle
        // so that it's always in the good range.
        double targetAngle = normalizeAngle(currentAngle + angle);

        double deviation = normalizeAngle(targetAngle - currentAngle);

        while (abs(deviation) > ROTATION_EPSILON) { // once the angular error is less than ROTATION_EPSILON, we have arrived
            currentAngle = getCurrentAngle();
            deviation = normalizeAngle(targetAngle - currentAngle);
            // the power is proportional to the deviation, but may not go below minPower.
            double rotatePower = deviation * powerScalar + minPower * signum(deviation);
            driveMecanum(new Pose(0, 0, rotatePower));
        }
        stop();
    }

    /**
     * Drives the Robot in straight line to given position on the board (x, y, angle).
     *
     * @param targetLocation The location which the robot needs to be driven to.
     */
    public void moveTo(Pose targetLocation) {
        final Pose powerScalar = new Pose(0.007, 0.008, 0.6);
        final Pose minPower = new Pose(0.12, 0.15, 0.08);
        final Pose epsilon = new Pose(0.5, 1, 0.0087);

        Pose Deviation = Pose.difference(targetLocation, positionCM);
        Deviation.normalizeAngle();
        Pose actPowers = new Pose();

        while (abs(Deviation.x) > epsilon.x ||
                abs(Deviation.y) > epsilon.y || abs(Deviation.angle) > epsilon.angle) {

            if (abs(Deviation.x) > epsilon.x) {
                actPowers.x = signum(Deviation.x) * minPower.x + Deviation.x * powerScalar.x;
            }
            if (abs(Deviation.y) > epsilon.y) {
                actPowers.y = signum(Deviation.y) * minPower.y + Deviation.y * powerScalar.y;
            }
            if (abs(Deviation.angle) > epsilon.angle) {
                actPowers.angle = signum(Deviation.angle) * minPower.angle + Deviation.angle * powerScalar.angle;
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
     * drives the Robot to location on given field
     *
     * @param targetLocation the location the robots moves to.
     */
    @PID
    public void move2(Pose targetLocation) {
        final Pose Kp = new Pose(0.01, 0.01, 0.73);
        final Pose Ki = new Pose(0, 0, 0);
        final Pose Kd = new Pose(0.000003, 0.000003, 0.00001);

        final Pose epsilon = new Pose(0.5, 1, ROTATION_EPSILON);

        Pose Deviation = Pose.difference(targetLocation, positionCM);
        Deviation.normalizeAngle();
        PosePIDController actPowers = new PosePIDController(Kp, Ki, Kd);

        while (opMode.opModeIsActive() && (
                abs(Deviation.x) > epsilon.x ||
                        abs(Deviation.y) > epsilon.y ||
                        abs(Deviation.angle) > epsilon.angle)) {

            driveByAxis(actPowers.powerByDeviation(Deviation));

            Deviation = Pose.difference(targetLocation, positionCM);
            Deviation.normalizeAngle();
        }
        stop();
    }

    /**
     * drives the Robot to location
     *
     * @param Distances the relative distances to the by.
     */
    @PID
    public void moveRelativeToRobot(Pose Distances) {
        Pose targetLocation = new Pose();

        final double cosAngle = cos(positionCM.angle);
        final double sinAngle = sin(positionCM.angle);

        targetLocation.x = positionCM.x + Distances.x * cosAngle + Distances.y * sinAngle;
        targetLocation.y = positionCM.y - Distances.x * sinAngle + Distances.y * cosAngle;
        targetLocation.angle = normalizeAngle(positionCM.angle + Distances.angle);

        move2(targetLocation);
    }
}
