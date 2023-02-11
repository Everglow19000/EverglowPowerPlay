package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.SplinePath;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.Trajectory;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@Autonomous(name = "MultiConeTeleop", group = "Template")
public class MultiConeTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {

        SystemCoordinator systems = new SystemCoordinator(this);
        CameraSystem cameraSystem = new CameraSystem(this);

        double sidewaysDistance;
        int coneNumber = 1;

        Pose pickUpLocation = new Pose(), dropOffLocation = new Pose();

        Sequence startSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 10),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
                systems.sleepingSystem.goToSequenceItem(200),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1),
                systems.sleepingSystem.goToSequenceItem(500),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
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


        Trajectory startTrajectory = new Trajectory(
                new SplinePath(new PointD[]{
                        new PointD(0, 0),
                        new PointD(0, 120),
                        new PointD(-27.6, 132.2),
                }), RobotParameters.MAX_V_X * 0.5);



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
        systems.drivingSystem.driveByPath(startTrajectory, 0, 1);
        systems.waitForSequencesDone();
        systems.executeSequence(
                new Sequence(
                        systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
                )
        );


//        for (int i = 0; i < coneNumber; i++) {
//            systems.executeSequence(pickUpSequences[i]);
//            systems.drivingSystem.move2(pickUpLocation);
//            systems.waitForSequencesDone();
//
//            systems.executeSequence(new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1)));
//            systems.waitForSequencesDone();
//
//            systems.executeSequence(new Sequence(systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH)));
//            systems.drivingSystem.move2(dropOffLocation);
//            systems.waitForSequencesDone();
//
//            systems.executeSequence(new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1)));
//            systems.waitForSequencesDone();
//        }
//
//        systems.executeSequence(endSequence);
//        systems.drivingSystem.driveToX(sidewaysDistance);

       



        systems.positionLogger.saveTo(PositionLogger.generateLogFileName("drive-path-sm"));

        systems.sleep(1000000000);


    }
}