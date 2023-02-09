package org.firstinspires.ftc.teamcode.autonomous;

import android.os.Build;
import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

/**
 * A class that contains all of the autonomous routes for the robot.
 */
public class AutonomousRoutes implements Runnable {
	private final LinearOpMode opMode;
	private final SystemCoordinator systems;
	private final CameraSystem cameraSystem;
	private boolean isTimerActive = false;
	boolean isRightAutonomous;

	public AutonomousRoutes(LinearOpMode opMode, boolean isRightAutonomous) {
		this.opMode = opMode;
		cameraSystem = new CameraSystem(opMode);
		systems = new SystemCoordinator(opMode);
		this.isRightAutonomous = isRightAutonomous;
	}

	/**
	 * A test method that drives the robot forwards or sideways, depending on the value the AprilTag.
	 */
	@RequiresApi(api = Build.VERSION_CODES.O)
	public void run() {
		isTimerActive = true;
		ElapsedTime elapsedTime = new ElapsedTime();
		while (opMode.opModeIsActive() && elapsedTime.seconds() < 22) {

		}
		isTimerActive = false;
	}

	public void putConesAndBack() {
		Thread timerThread = new Thread();
		timerThread.start();

		isTimerActive = true;
		final double lenSquare = 64;

		final int right;
		if (isRightAutonomous) {
			right = 1;
		} else {
			right = -1;
		}

		final Pose startPose = systems.trackingSystem.getPosition();
		final Pose pickCone = new Pose(right * lenSquare / 2, 2 * lenSquare, 0);//PI/2
		final Pose putCone = new Pose(-right * lenSquare / 4, 2.5 * lenSquare, 0);//PI/2

		//starting Sequence
		Sequence preparePickUp = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.START));
		Sequence toCone = new Sequence(
				/*systems.drivingSystem.driveSidewaysSequenceItem(pickCone.x, pickCone.angle),
				systems.drivingSystem.driveStraightSequenceItem(pickCone.y, pickCone.angle)*/);
		Sequence toPole = new Sequence(
				/*systems.drivingSystem.driveSidewaysSequenceItem(putCone.x, putCone.angle),
				systems.drivingSystem.driveStraightSequenceItem(putCone.y, putCone.angle)*/);
		Sequence sequencePickUp = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW));
		Sequence prepareDropOff = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF));


		Sequence dropOff = new Sequence(systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1));

		Sequence prepareForAnotherRep = new Sequence(systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP));

		while (opMode.opModeIsActive() && isTimerActive) {

			systems.executeSequence(toCone);
			while (!toCone.isSequenceDone() && opMode.opModeIsActive()) { // may to upsideDown Sequence
				systems.tick();
				systems.executeSequence(preparePickUp);
				while (!preparePickUp.isSequenceDone() && opMode.opModeIsActive()) {
					systems.tick();
				}
			}

			systems.executeSequence(sequencePickUp);
			while (!sequencePickUp.isSequenceDone() && opMode.opModeIsActive()) {
				systems.tick();
			}

			systems.executeSequence(prepareDropOff);
			while (!prepareDropOff.isSequenceDone() && opMode.opModeIsActive()) {
				systems.tick();
				systems.executeSequence(toPole);
				while (!toPole.isSequenceDone() && opMode.opModeIsActive()) {
					systems.tick();
				}
			}

			systems.executeSequence(prepareForAnotherRep);
			while (!prepareForAnotherRep.isSequenceDone() && opMode.opModeIsActive()) {
				systems.tick();
			}
		}

		Sequence waitForPark = new Sequence(
				/*systems.drivingSystem.driveSidewaysSequenceItem(startPose.x, startPose.angle),
				systems.drivingSystem.driveStraightSequenceItem(startPose.y, startPose.angle)*/);

		systems.executeSequence(waitForPark);
		while (!waitForPark.isSequenceDone() && opMode.opModeIsActive()) {
			systems.tick();
		}
	}

	public void park(CameraSystem.AprilTagType tagType) {
		double sidewaysDistance;
		switch (tagType) {
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

		Sequence sequence = new Sequence(
				/*systems.drivingSystem.driveSidewaysSequenceItem(sidewaysDistance, 0),
				systems.drivingSystem.driveStraightSequenceItem(90, 0)*/);
		systems.executeSequence(sequence);
		while (opMode.opModeIsActive() && !sequence.isSequenceDone()) {
			systems.tick();
		}
	}
}