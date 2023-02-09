package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@Autonomous(name = "DumbConeAutonomous", group = "Cone")
public class DumbConeAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        SystemCoordinator systems = new SystemCoordinator(this);
        CameraSystem cameraSystem = new CameraSystem(this);

        double sidewaysDistance;
        int coneNumber = 1;

        Pose pickUpLocation = new Pose(), dropOffLocation = new Pose();

        Sequence startSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
        );
        Sequence dropOffSequences = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH)
        );

        Sequence[] pickUpSequences = {new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.CONE1)
        ), new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.CONE2)
        ), new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.CONE3)
        ), new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.CONE4)
        ), new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.CONE5)
        ), };

        Sequence endSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.START),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
        );


        telemetry.addData("Ready", coneNumber);
        telemetry.update();

        waitForStart();

        switch (cameraSystem.detectAprilTag()) {
            case TAG_1:
                sidewaysDistance = 65;
                break;
            case TAG_2:
            default:
                sidewaysDistance = 0;
                break;
            case TAG_3:
                sidewaysDistance = -65;
                break;
        }

        systems.executeSequence(startSequence);
        systems.drivingSystem.driveY(TILE_SIZE * 2 + 10);
        systems.waitForSequencesDone();


        for (int i = 0; i < coneNumber; i++) {
            systems.executeSequence(pickUpSequences[i]);
            systems.drivingSystem.move2(pickUpLocation);
            systems.waitForSequencesDone();

            systems.executeSequence(new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1)));
            systems.waitForSequencesDone();

            systems.executeSequence(dropOffSequences);
            systems.drivingSystem.move2(dropOffLocation);
            systems.waitForSequencesDone();

            systems.executeSequence(new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1)));
            systems.waitForSequencesDone();
        }

        systems.executeSequence(endSequence);
        systems.drivingSystem.driveToX(sidewaysDistance);
    }
}