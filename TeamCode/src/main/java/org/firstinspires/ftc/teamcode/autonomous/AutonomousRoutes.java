package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.time.LocalTime;
import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.Sequence;

/**
 * A class that contains all of the autonomous routes for the robot.
 */
public class AutonomousRoutes implements Runnable {
	private final LinearOpMode opMode;
	private SystemCoordinator systems;
	private CameraSystem cameraSystem;
	private boolean isTimeractive = false;
	private boolean befourAction = true;

	public AutonomousRoutes(LinearOpMode opMode) {
		this.opMode = opMode;
		cameraSystem = new CameraSystem(opMode);
		systems = new SystemCoordinator(opMode);
	}

	/**
	 * A test method that drives the robot forwards or sideways, depending on the value the AprilTag.
	 */

	@RequiresApi(api = Build.VERSION_CODES.O)
	public void run(){

		int startSec = LocalTime.now().getSecond();
		int nowSec = LocalTime.now().getSecond();
		double deltaTime = nowSec - startSec;
		double deltaTimeRelativeToAction = deltaTime;
		while (opMode.opModeIsActive() && deltaTimeRelativeToAction<20){
			nowSec = LocalTime.now().getSecond();
			deltaTime = nowSec - startSec;
			if (befourAction){
				deltaTimeRelativeToAction = deltaTime+5;
			}
			else{
				deltaTimeRelativeToAction = deltaTime;
			}
		}
		isTimeractive = false;
		return;
	}

	public void putConesAndBack(boolean isRightAutonomous){
		Thread timerThread = new Thread();
		timerThread.run();

		isTimeractive = true;
		final double lenSquere = 64;

		final int right;
		if (isRightAutonomous){
			right = 1;
		}else {
			right =-1;
		}

		final Pose startPose = systems.trackingSystem.getPosition();
		final Pose pickCone = new Pose(right*lenSquere/2, 2*lenSquere, 0);//PI/2
		final Pose putCone = new Pose(-right*lenSquere/4, 2.5*lenSquere, 0);//

		Sequence sequencePickUp = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
				systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 1),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH));

		Sequence sequenceDropOff = new Sequence(
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.DROPOFF),
				systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.OPEN, 1));

		Sequence sequenceBackToStart = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.ClawPosition.CLOSED, 1),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.FourBarPosition.PICKUP),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW));


		while (opMode.opModeIsActive() && isTimeractive) {
			befourAction = true;

			if (isTimeractive){
				befourAction = false;
				systems.drivingSystem.driveSidewaysSequenceItem(pickCone.x, pickCone.angle);
				systems.drivingSystem.driveStraightSequenceItem(pickCone.y, pickCone.angle);
				//systems.gWheelSystem.toggleCollect();
				//opMode.sleep(250);
				//systems.gWheelSystem.toggleCollect();
				//sequencePickUp.start();
				//opMode.sleep(250);
				systems.drivingSystem.driveSidewaysSequenceItem(putCone.x, putCone.angle);
				systems.drivingSystem.driveStraightSequenceItem(putCone.y, putCone.angle);
				//sequenceDropOff.start();
				opMode.sleep(750);
				//sequenceBackToStart.start();
			}else {
				break;
			}
		}

		systems.drivingSystem.driveSidewaysSequenceItem(startPose.x, startPose.angle);
		systems.drivingSystem.driveStraightSequenceItem(startPose.y, startPose.angle);
	}
}