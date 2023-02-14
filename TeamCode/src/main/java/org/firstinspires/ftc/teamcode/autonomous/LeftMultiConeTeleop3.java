package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.SplinePath;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.Trajectory;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@Autonomous(name = "LeftMultiConeTeleop3", group = "Template")
public class LeftMultiConeTeleop3 extends LinearOpMode {
    @Override
    public void runOpMode() {
        int left = 1;
        Pose startPosition = new Pose(left * (1 * TILE_SIZE + (25.5 + 59) / 2), -2 * TILE_SIZE - (51.5 + 22.5) / 2, 0);

        SystemCoordinator systems = SystemCoordinator.init(this);
        CameraSystem cameraSystem = new CameraSystem(this);

        double sidewaysDistance;
        int coneNumber = 3;

        Sequence startSequence1 = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 10),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
				systems.sleepingSystem.goToSequenceItem(600),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 2),
                systems.sleepingSystem.goToSequenceItem(600),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH)
        );

        Sequence startSequence2 = new Sequence(
                systems.sleepingSystem.goToSequenceItem(20000000),
                systems.sleepingSystem.goToSequenceItem(500),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
        );

        Pose pickUpLocation = new Pose(left * 2.6 * TILE_SIZE, -0.5 * TILE_SIZE, toRadians(90));
        Pose afterFirstDropOffLocation = new Pose(left * (1 * TILE_SIZE + 5), -0.42 * TILE_SIZE - 10, 0);
        Pose middleLocation = new Pose(left * 1.5 * TILE_SIZE, -0.45 * TILE_SIZE, toRadians(-45));
        Pose dropOffLocation = new Pose(left * (1.5 * TILE_SIZE - 22), -0.5 * TILE_SIZE + 12 + 10, toRadians(-45));

        Sequence endSequence = new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.BACK),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.START),
                systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
        );

        // before copy paste
        Trajectory startTrajectoryNew = new Trajectory(
                new SplinePath(new PointD[]{
                        new PointD(startPosition.x, startPosition.y),
                        new PointD(left * 1.5 * TILE_SIZE, -2 * TILE_SIZE),
                        new PointD(left * 1.5 * TILE_SIZE, -0.8 * TILE_SIZE),
                        new PointD(left * (1 * TILE_SIZE), -0.4 * TILE_SIZE)
                }), RobotParameters.MAX_V_X * 0.5);


        systems.trackingSystem.resetPosition(startPosition);

        Pose finalPose = new Pose(left * 1.5 * TILE_SIZE, -33, 0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

		/*systems.drivingSystem.driveX(20);
		systems.drivingSystem.stop();
		systems.sleep(1000);*/

        CameraSystem.AprilTagType detectedTag = cameraSystem.detectAprilTag();
        telemetry.addData("Detected Tag: ", detectedTag);
        telemetry.update();
        switch (detectedTag) {
            case TAG_1:
                finalPose.x += TILE_SIZE * 1.3;
                break;
            case TAG_2:
            default:
                break;
            case TAG_3:
                finalPose.x -= TILE_SIZE * 1.3;
                break;
        }

        systems.executeSequence(startSequence1);
        systems.executeSequence(startSequence2);
        systems.drivingSystem.driveByPath(startTrajectoryNew, 0, 1);
        systems.drivingSystem.stop();
        systems.waitForSequencesDone();

        systems.sleep(2000);
        systems.executeSequence(new Sequence(
                systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOCK_IN),
                systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN)
        ));
        systems.waitForSequencesDone();
        systems.drivingSystem.driveY(-5);
        systems.drivingSystem.stop();


        systems.executeSequence(endSequence);
        systems.drivingSystem.move3(finalPose);
        systems.drivingSystem.stop();
        systems.waitForSequencesDone();
        sleep(1000000000);
    }
}