package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Sequence;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		CameraSystem cameraSystem = new CameraSystem(this);
		waitForStart();
		CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
		double sidewaysDistance;
		switch (tagType){
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
				systemCoordinator.drivingSystem.driveSidewaysSequenceItem(sidewaysDistance, 0),
				systemCoordinator.drivingSystem.driveStraightSequenceItem(90, 0)
		);
		systemCoordinator.executeSequence(sequence);
		while (opModeIsActive() && !sequence.isSequenceDone()){
			systemCoordinator.tick();
		}
	}
}