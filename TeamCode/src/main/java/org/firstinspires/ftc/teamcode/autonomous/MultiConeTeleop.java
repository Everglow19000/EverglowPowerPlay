package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.systems.FourBarSystem.Position.AUTO_PICKUP;
import static org.firstinspires.ftc.teamcode.systems.FourBarSystem.Position.DROPOFF;
import static org.firstinspires.ftc.teamcode.utils.RobotParameters.TILE_SIZE;
import static java.lang.Math.sqrt;
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

@Autonomous(name = "MultiConeTeleop", group = "Template")
public class MultiConeTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {

		Pose startPosition = new Pose(1.5 * TILE_SIZE, -2 * TILE_SIZE - 32.5, 0);

		SystemCoordinator systems = new SystemCoordinator(this);
		CameraSystem cameraSystem = new CameraSystem(this);

		double sidewaysDistance;
		int coneNumber = 1;


		Pose endStartSequenceLocation = new Pose(1.6 * TILE_SIZE, -0.8 * TILE_SIZE, toRadians(90));
		Pose pickUpLocation = new Pose(2.5 * TILE_SIZE, -0.7 * TILE_SIZE, toRadians(90));
		Pose dropOffLocation = new Pose(1 * TILE_SIZE, -23.5, 0);

		Pose preDropoffFirstPose = new Pose(1.5 * TILE_SIZE, -0.5 * TILE_SIZE, toRadians(-45));
		Pose firstDropoffPose = Pose.sum(preDropoffFirstPose, new Pose(-26 / sqrt(2), 20 / sqrt(2), 0));


		Sequence endSequence = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.START),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
		);


		Trajectory startTrajectory = new Trajectory(
				new SplinePath(new PointD[]{
						new PointD(startPosition.x, startPosition.y),
						new PointD(startPosition.x, 120 + startPosition.y),
						new PointD(startPosition.x - 27.6, startPosition.y + 132.2)
				}), RobotParameters.MAX_V_X * 0.5);


		systems.trackingSystem.resetPosition(startPosition);

		telemetry.addData("Ready", coneNumber);
		telemetry.update();

		waitForStart();

		Pose finalPose = new Pose(1.5 * TILE_SIZE, -23.5, 0);

		CameraSystem.AprilTagType detectedTag = cameraSystem.detectAprilTag();
		telemetry.addData("Detected Tag: ", detectedTag);
		telemetry.update();
		switch (detectedTag) {
			case TAG_1:
				finalPose.x += TILE_SIZE;
				break;
			case TAG_2:
			default:
				break;
			case TAG_3:
				finalPose.x -= TILE_SIZE;
				break;
		}


        systems.executeSequence(startSequence);
		systems.drivingSystem.driveByPath(startTrajectory, 0, 1);
		systems.drivingSystem.stop();
		systems.waitForSequencesDone();

		systems.executeSequence(new Sequence(
						systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
				));
		systems.sleep(100);
//        systems.drivingSystem.move3(endStartSequenceLocation);

		for (int i = 0; i < coneNumber; i++) {
			systems.executeSequence(new Sequence(
					systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
					systems.fourBarSystem.goToSequenceItem(AUTO_PICKUP)
			));
			systems.drivingSystem.move3(pickUpLocation);
			systems.waitForSequencesDone();
			systems.executeSequence(new Sequence(
					systems.elevatorSystem.goToSequenceItem(ElevatorSystem.conePickupLevels[i]),
					systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED),
					systems.sleepingSystem.goToSequenceItem(250)
			));
			systems.waitForSequencesDone();

			systems.executeSequence(new Sequence(
					systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
					systems.fourBarSystem.goToSequenceItem(DROPOFF)
			));

			systems.sleep(500);
			systems.drivingSystem.move3(dropOffLocation);
			systems.waitForSequencesDone();

			systems.executeSequence(new Sequence(
					systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
			));
		}

		systems.executeSequence(endSequence);
		systems.drivingSystem.move3(finalPose);


//		systems.positionLogger.saveTo(PositionLogger.generateLogFileName("drive-path-sm"));

		systems.sleep(10000000000L);


	}
}