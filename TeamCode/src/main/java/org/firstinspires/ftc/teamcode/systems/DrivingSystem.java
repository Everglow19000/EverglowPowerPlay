package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DrivingSystem {

    private static final double WHEEL_RADIUS_CM = 4.8;
    private static final double TICKS_PER_ROTATION = 515;
    private static final double CM_PER_TICK = 1. / TICKS_PER_ROTATION * WHEEL_RADIUS_CM * 2 * Math.PI;

    /**
     * Creates an IMU object and calibrates it correctly to its actual face.
     * @param opMode the current opMode
     * @return an BNO055IMU object
     */
    private static BNO055IMU initalizeImu(LinearOpMode opMode){
        // Create IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
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

        while (!imu.isGyroCalibrated()){
            // wait for the gyroscope calibration
            opMode.sleep(10);
        }
        return imu;
    }


    private final LinearOpMode opMode;

    private final BNO055IMU imu;
    private final DcMotor frontRight;
    private final DcMotor frontLeft;
    private final DcMotor backRight;
    private final DcMotor backLeft;

    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        imu = initalizeImu(opMode);

        // Creates objects to control the motors
        frontRight = opMode.hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "front_left");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "back_left");
        backRight = opMode.hardwareMap.get(DcMotor.class, "back_right");

        // makes the motors break when their power is set to zero, so they can better stop in place.
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // some motors are wired in reverse, so we must reverse them back
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drives the robot on a mechanism drive.
     * @param x vertical power, positive is right, negative is left
     * @param y horizontal power, positive is forward, negative is backwards.
     * @param rot rotational power, positive is clockwise, negative is counterclockwise. (todo: check this is correct)
     */
    public void driveMecanum(double x, double y, double rot) {
        double frontRightPower = y - x + rot;
        double frontLeftPower  = y + x - rot;
        double backRightPower  = y + x + rot;
        double backLeftPower   = y - x - rot;

        // the function setPower only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1.
        double norm = Math.max(
                Math.max(frontRightPower, frontLeftPower),
                Math.max(backRightPower, backLeftPower)
        );
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
    public void stop(){
        driveMecanum(0, 0,0);
    }

    public double getCurrentAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    /**
     * Given any angle, normalizes it such that it is between -180 and 180 degrees,
     * increasing or decreasing by 360 to make it so.
     */
    public static double normalizeAngle(double angle){
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Rotates the robot a given number of degrees.
     * A positive angle means clockwise rotation, a negetive angle is counterclockwise. (todo: check this is correct)
     */
    public void rotate(double angle) {
        final double powerScalar = 0.007;
        final double minPower = 0.2;
        final double epsilon = 0.5;

        double currentAngle = getCurrentAngle();

        // Angles must always be between -180 and 180 degrees.
        // The function used below adds or subtracts 360 degrees from the angle
        // so that it's always in the good range.
        double targetAngle = normalizeAngle(currentAngle + angle);

        double deviation = normalizeAngle(currentAngle - targetAngle);

        while (Math.abs(deviation) > epsilon) { // once the angular error is less than 0.5 degrees, we have arrived
            currentAngle = getCurrentAngle();
            deviation = normalizeAngle(currentAngle - targetAngle);
            // the power is proportional to the deviation, but may not go below minPower.
            double rotatePower = -deviation * powerScalar - minPower * Math.signum(deviation);
            driveMecanum(0, 0, rotatePower);
        }
        driveMecanum(0, 0, 0);  // stops the robot
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
     * returns how far forward, in centimeters,
     * the robot has moved since the last time that resetDistance() was called.
     * Assumes the robot hasn't moved in any other directions.
     */
    public double getForwardDistance() {
        return (
                frontLeft.getCurrentPosition() +
                        frontRight.getCurrentPosition() +
                        backLeft.getCurrentPosition() +
                        backRight.getCurrentPosition()
        ) / 4. * CM_PER_TICK;
    }

    /**
     * Drives the robot straight a given distance.
     * @param distance How far the robot should go, measured in centimeters. Positive is forwards, negative is backwards.
     * @param power How much power should be given to the motor, from 0 to 1.
     */
    public void driveStraight(double distance, double power) {
        final double ANGLE_DEVIATION_SCALAR = 0.05;

        // if we're traveling a negative distance, that means traveling backwards,
        // so the power should be inverted and so should the distance.
        if (distance < 0){
            distance = -distance;
            power = -power;
        }

        resetDistance();
        double startAngle = getCurrentAngle();
        double forwardDistance = getForwardDistance();

        while (Math.abs(forwardDistance) < distance) {
            forwardDistance = getForwardDistance();
            double angleDeviation = AngleUnit.DEGREES.normalize(startAngle - getCurrentAngle());
            double rotatePower = angleDeviation * ANGLE_DEVIATION_SCALAR;
            driveMecanum(0, power, rotatePower);
        }
        driveMecanum(0, 0, 0);
    }

    /**
     * returns how far sideways, in centimeters,
     * the robot has moved since the last time that resetDistance() was called.
     * Assumes the robot hasn't moved in any other directions.
     */

    public double getSidewaysDistance() {
        return (
                frontRight.getCurrentPosition() +
                        - frontLeft.getCurrentPosition() +
                        - backRight.getCurrentPosition() +
                        + backLeft.getCurrentPosition()
        ) / 4. * CM_PER_TICK;
    }

    /**
     * Drives the robot straight a given distance.
     * @param distance How far the robot should go, measured in centimeters. Positive is right, negative is left.
     * @param power How much power should be given to the motor, from 0 to 1.
     */
    public void driveSideways(double distance, double power) {
        double ANGLE_DEVIATION_K = 0.05;

        // if we're traveling a negative distance, that means traveling left,
        // so the power should be inverted and so should the distance.
        if (distance < 0){
            distance = -distance;
            power = -power;
        }


        resetDistance();
        double startAngle = getCurrentAngle();
        double sidewaysDistance = getSidewaysDistance();
        while (Math.abs(sidewaysDistance) < distance) {
            sidewaysDistance = getSidewaysDistance();
            double angleDeviation = normalizeAngle(startAngle - getCurrentAngle());
            driveMecanum(power, 0, -angleDeviation * ANGLE_DEVIATION_K);
        }
        driveMecanum(0, 0, 0);
    }
}
