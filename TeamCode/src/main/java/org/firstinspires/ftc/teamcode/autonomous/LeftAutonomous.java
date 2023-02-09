package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@Autonomous(name = "LeftAutonomous")
public class LeftAutonomous extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		//CameraSystem cameraSystem = new CameraSystem(this);
		AutonomousRoutes autonomousRoutes = new AutonomousRoutes(this, false);
		waitForStart();
//        CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
//        double sidewaysDistance;
//        switch (tagType){
//            case TAG_1:
//                sidewaysDistance = 65;
//                break;
//            case TAG_2:
//            default:
//                sidewaysDistance = 0;
//                break;
//            case TAG_3:
//                sidewaysDistance = -65;
//                break;
//        }
		autonomousRoutes.putConesAndBack();
		Sequence sequence = new Sequence(
				systemCoordinator.drivingSystem.driveSidewaysSequenceItem(0, 0), //sidewaysDistance
				systemCoordinator.drivingSystem.driveStraightSequenceItem(90, 0)
		);
		systemCoordinator.executeSequence(sequence);
		while (opModeIsActive() && !sequence.isSequenceDone()) {
			systemCoordinator.tick();
		}
	}
}
