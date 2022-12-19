package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the claw.
 */
public class ClawSystem {
    /**
     * Enum encapsulating the two positions the system should reach.
     */
    public enum Level {
        OPEN(0.67), CLOSED(0.15);

        Level(double position) {
            this.position = position;
        }

        public final double position;
    }

    private final Servo servo;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public ClawSystem(LinearOpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, "claw");
        goTo(Level.OPEN);
    }

    /**
     * Moves the system to a position from the Level enum.
     *
     * @param level Position from the Level enum.
     */
    public void goTo(Level level) {
        servo.setPosition(level.position);
    }
}
