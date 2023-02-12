package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.systems.FourBarSystem.Position.AUTO_PICKUP;
import static org.firstinspires.ftc.teamcode.systems.FourBarSystem.Position.DROPOFF;
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

@Autonomous(name = "MultiConeTeleop2", group = "Template")
public class MultiConeTeleop2 extends LinearOpMode {
	@Override
	public void runOpMode() {

		Pose startPosition = new Pose(1.7 * TILE_SIZE, -2 * TILE_SIZE - 32.5, 0);

		SystemCoordinator systems = SystemCoordinator.init(this);
		CameraSystem cameraSystem = new CameraSystem(this);

		double sidewaysDistance;
		int coneNumber = 3;

		Sequence startSequence = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PRE_PICKUP),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 10),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
//				systems.sleepingSystem.goToSequenceItem(500),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 2),
				systems.sleepingSystem.goToSequenceItem(200),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
		);

		Pose pickUpLocation = new Pose(2.72 * TILE_SIZE, -0.64 * TILE_SIZE + 4, toRadians(90));
		Pose afterFirstDropOffLocation = new Pose(1.5 * TILE_SIZE - 30.6, -2 * TILE_SIZE - 32.5 + 130.2 - 10, 0);
		Pose middleLocation = new Pose(1.5 * TILE_SIZE, -0.5 * TILE_SIZE, toRadians(-45));
		Pose dropOffLocation = new Pose(1.5 * TILE_SIZE - 12, -0.5 * TILE_SIZE + 12, toRadians(-45));

		Sequence endSequence = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.MID),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.START),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
		);


		Trajectory startTrajectoryNew = new Trajectory(
				new SplinePath(new PointD[]{
						new PointD(startPosition.x, startPosition.y),
						new PointD(1.5 * TILE_SIZE, -2 * TILE_SIZE),
						new PointD(1.5 * TILE_SIZE, - 0.45 * TILE_SIZE),
						new PointD(1.5 * TILE_SIZE - 30.6, -2 * TILE_SIZE - 32.5 + 120.2)
				}), RobotParameters.MAX_V_X * 0.5);

		systems.trackingSystem.resetPosition(startPosition);

		Pose finalPose = new Pose(1.5 * TILE_SIZE, -33, 0);

		telemetry.addLine("Ready");
		telemetry.update();

		waitForStart();

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
		systems.drivingSystem.driveByPath(startTrajectoryNew, 0, 1);
		systems.drivingSystem.stop();
		systems.waitForSequencesDone();

		systems.sleep(200);
		systems.executeSequence(new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOCK_IN),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN)
		));
		systems.drivingSystem.move3(afterFirstDropOffLocation);

		systems.waitForSequencesDone();
		for (int i = 0; i < coneNumber; i++) {
			systems.executeSequence(new Sequence(
					systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
					systems.fourBarSystem.goToSequenceItem(AUTO_PICKUP)
			));
			systems.drivingSystem.move3(pickUpLocation);
			systems.drivingSystem.stop();
			systems.sleep(1000);
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
			systems.sleep(200);

			systems.drivingSystem.move3(middleLocation);
			systems.drivingSystem.move3(dropOffLocation);
			systems.drivingSystem.stop();
			systems.waitForSequencesDone();

			systems.sleep(200);
			systems.executeSequence(new Sequence(
					systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOCK_IN),
					systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 10)
			));
			systems.waitForSequencesDone();
			systems.drivingSystem.move3(middleLocation);
		}

		systems.executeSequence(endSequence);
		systems.drivingSystem.move3(finalPose);
		systems.sleep(10000000000L);
	}
}