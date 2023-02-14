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
        SystemCoordinator systems = SystemCoordinator.init(this, false);
        EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
        EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

        Sequence sequence;
        ClawSystem.Position claw = ClawSystem.Position.OPEN;

        waitForStart();

        Sequence startSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP));
        Sequence triangleSequence;
        Sequence squareSequence;
        Sequence clawSequence;

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
                clawSequence = new Sequence(systems.clawSystem.goToSequenceItem(claw, 1));
                systems.executeSequence(clawSequence);
            }

            if (gamepadB.lt()) {
                systems.gWheelSystem.toggleSpit();
            } else if (gamepadB.rt()) {
                systems.gWheelSystem.toggleCollect();
            }

            if (gamepadB.dpad_right()) {
                systems.interrupt();
                sequence = new Sequence(
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
                        systems.sleepingSystem.goToSequenceItem(600),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP)
                );
                telemetry.addData("pos: ", systems.fourBarSystem.fourBar.getCurrentPosition());
                systems.executeSequence(sequence);
            }
            if (gamepadB.dpad_up()) {
                systems.interrupt();
                systems.fourBarSystem.fourBar.setPower((0.3));
                sequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
                );
                systems.executeSequence(sequence);
                systems.fourBarSystem.fourBar.setPower((0.7));
            } else if (gamepadB.dpad_left()) {
                systems.interrupt();
                systems.fourBarSystem.fourBar.setPower((0.3));
                sequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
                );
                systems.executeSequence(sequence);
                systems.fourBarSystem.fourBar.setPower((0.7));
            } else if (gamepadB.dpad_down()) {
                systems.interrupt();
                systems.fourBarSystem.fourBar.setPower((0.3));
                sequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW)
                );
                systems.executeSequence(sequence);
                systems.fourBarSystem.fourBar.setPower((0.7));
            }

            if (gamepadB.square()) {
                systems.interrupt();
                claw = ClawSystem.Position.OPEN;
                if (systems.fourBarSystem.fourBar.getCurrentPosition() >= -120) {

                    squareSequence = new Sequence(
                            systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                            systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP)
                    );
                    systems.executeSequence(squareSequence);

                } else {
                    squareSequence = new Sequence(
                            systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                            systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
                            systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP)
                    );
                    systems.executeSequence(squareSequence);
                }
            }

            if (gamepadB.triangle()) {
                systems.interrupt();
                claw = ClawSystem.Position.OPEN;
                triangleSequence = new Sequence(
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.LOW_DROPOFF),
                        systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP)
                );
                systems.executeSequence(triangleSequence);
            }

            if (gamepadB.cross()) {
                systems.interrupt();
                claw = ClawSystem.Position.OPEN;
                triangleSequence = new Sequence(
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF),
                        systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1),
                        systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
                        systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP)
                );
                systems.executeSequence(triangleSequence);
            }

            systems.tick();
        }
    }
}
