package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class for handling the four bar linkage system.
 */
public class FourBar {
    /**
     * Enum encapsulating the most common positions for the system to reach
     */
    public enum Level {
        PICKUP(0.63), DROPOFF(0.123);

        Level(double position) {
            this.position = position;
        }

        private final double position;
    }

    private final Servo servo;

    /**
     * @param opMode The current opMode running on the robot.
     */
    public FourBar(LinearOpMode opMode) {
        servo = opMode.hardwareMap.get(Servo.class, "4bar");
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
