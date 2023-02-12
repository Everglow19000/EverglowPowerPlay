package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@TeleOp(name = "TwoDriverTeleopIvri", group = ".Main")
public class TwoDriverTeleopIvri extends LinearOpMode {
    /**
     * A number to divide the speed by when finner controls are activated.
     */
    final double speedDivisor = 4.5;

    @Override
    public void runOpMode() {
        SystemCoordinator systems = SystemCoordinator.init(this);
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        Sequence sequence;
        ClawSystem.Position claw = ClawSystem.Position.OPEN;

        waitForStart();

        Sequence startSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP));
        systems.executeSequence(startSequence);

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
            systems.drivingSystem.driveMecanum(powers);


            // Claw
            if (gamepadB.circle()) {
                claw = claw.flip();
                Sequence clawSequence = new Sequence(systems.clawSystem.goToSequenceItem(claw, 1));
                systems.executeSequence(clawSequence);
            }

            if (gamepadB.lt()) {
                systems.gWheelSystem.toggleSpit();
            } else if (gamepadB.rt()) {
                systems.gWheelSystem.toggleCollect();
            }

            if(gamepadB.dpad_right()) {
                systems.interrupt();
                sequence = new Sequence(
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP)
                );
                systems.executeSequence(sequence);
            }

            if(gamepadB.dpad_up()) {
                systems.interrupt();
                sequence = new Sequence(
                    systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
                    systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
                );
                systems.executeSequence(sequence);
            } else if(gamepadB.dpad_left()) {
                systems.interrupt();
                sequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
                );
                systems.executeSequence(sequence);
            } else if(gamepadB.dpad_down()) {
                systems.interrupt();
                sequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW)
                );
                systems.executeSequence(sequence);
            }

            if (gamepadB.square()){
                sequence = new Sequence(
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.LOW_DROPOFF)
                );
                systems.executeSequence(sequence);
            }

            if (gamepadB.triangle()) {
                systems.interrupt();
                claw = ClawSystem.Position.OPEN;
                sequence = new Sequence(
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                        systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
                );
                systems.executeSequence(sequence);
            }
            systems.tick();
        }
    }
}
