package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBar {
    /**
     * Enum encapsulating the most common positions for the system to reach
     */
    public enum Level {
        PICKUP(-250), DROPOFF(-500), NEUTRAL(-1000);

        Level(int position) {
            this.position = position;
        }

        private final int position;
    }

    private final DcMotor motor;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public FourBar(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotor.class, "4bar");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.7);
        goTo(Level.NEUTRAL);
    }

    /**
     * Moves the system to a position from the Level enum.
     *
     * @param level Position from the Level enum.
     */
    public void goTo(Level level) {
        motor.setTargetPosition(level.position);
    }

    /**
     * Moves the system to a given position.
     *
     * @param position Target position of the motor in ticks.
     */
    public void goTo(int position) {
        motor.setTargetPosition(position);
    }
}
