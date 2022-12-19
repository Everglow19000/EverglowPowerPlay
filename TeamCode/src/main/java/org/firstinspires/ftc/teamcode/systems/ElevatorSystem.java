package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A class for handling the elevator.
 */
public class ElevatorSystem {
    /**
     * Enum encapsulating the most common positions for the system to reach.
     */
    public enum Level {
        PICKUP(0), PRE_PICKUP(-600), LOW(-1000), MID(-1900), HIGH(-2700);

        Level(int position) {
            this.position = position;
        }

        public final int position;
    }

    private final DcMotor left;
    private final DcMotor right;

    public ElevatorSystem(OpMode opMode) {
        left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
        right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(0);
        right.setTargetPosition(0);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(1);
        right.setPower(1);
    }

    /**
     * Moves the system to a position from the Level enum.
     *
     * @param level Position from the Level enum.
     */
    public void goTo(Level level) {
        left.setTargetPosition(level.position);
        right.setTargetPosition(level.position);
    }
}
