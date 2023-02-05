package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.Sequence;

@TeleOp(name = "TwoDriverTeleop2", group = ".Main")
public class TwoDriverTeleop2 extends LinearOpMode {
    /**
     * A number to divide the speed by when finner controls are activated
     */
    final double speedDivisor = 4.5;

    @Override
    public void runOpMode() {
        SystemCoordinator systems = new SystemCoordinator(this);
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        Sequence elevatorSequence = null;
        boolean isClawOpen = false;

        waitForStart();

        Sequence sequence = new Sequence(
                systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP));
        sequence.start();

        while (opModeIsActive()) {
            gamepadA.update();
            gamepadB.update();

            Pose powers = new Pose();

            powers.x = -gamepad1.left_stick_x * 0.75;
            powers.y = -gamepad1.left_stick_y * 0.75;
            powers.angle = -gamepad1.right_stick_x * 0.5;

            // Activate slower driving and turning, for finer adjustment
            if (gamepad1.right_trigger > 0.2) {
                powers.x = -gamepad1.left_stick_x / speedDivisor;
                powers.y = -gamepad1.left_stick_y / speedDivisor;
                powers.angle = -gamepad1.right_stick_x / speedDivisor;
            }

            // Apply calculated velocity to mecanum wheels
            systems.drivingSystem.driveByAxis(powers);


            if (gamepadB.lt()) {
                systems.gWheelSystem.toggleSpit();
            } else if (gamepadB.rt()) {
                systems.gWheelSystem.toggleCollect();
            }

            if(gamepadB.dpad_up()) {
                systems.interrupt();
                elevatorSequence = new Sequence(
                    systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
                    systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 0.5),
                    systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
                    systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF)
                );
                systems.executeSequence(elevatorSequence);
            } else if(gamepadB.dpad_left()) {
                systems.interrupt();
                elevatorSequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
                        systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 0.5),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF)
                );
                systems.executeSequence(elevatorSequence);
            } else if(gamepadB.dpad_down()) {
                systems.interrupt();
                elevatorSequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
                        systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 0.5),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF)
                );
                systems.executeSequence(elevatorSequence);
            }

            if (gamepadB.triangle()) {
                systems.interrupt();
                elevatorSequence = new Sequence(
                        systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 0.5),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP)
                );
                systems.executeSequence(elevatorSequence);
            }

            systems.tick();
        }
    }
}
