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

@Autonomous(name = "ConeAutonomous2")
public class ConeAutonomous extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systems = new SystemCoordinator(this);
		CameraSystem cameraSystem = new CameraSystem(this);
		//AutonomousRoutes autonomousRoutes = new AutonomousRoutes(this);

		double sidewaysDistance;
		int coneNumber = 1, pickUpTicks = 700, pickUpTicksDifference = 150;

		Pose pickUpLocation = new Pose(), dropOffLocation = new Pose();

		Sequence startSequence = new Sequence(
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.LOW),
				systems.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF),
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
		);
		Sequence pickUpSequence = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH)
		);
		Sequence dropOffSequence = new Sequence(
				systems.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1),
				systems.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH)
		);

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

		// TODO: elevator sequence

		systems.drivingSystem.driveStraightSequenceItem(TILE_SIZE * 2 + 10, 0);
		for (int i = 0; i < coneNumber; i++) {
			systems.drivingSystem.move2(pickUpLocation);
			// systems.executeSequence(pickUpSequence);
			systems.drivingSystem.move2(dropOffLocation);
			// Wait for the previous sequence to finish
			// systems.executeSequence(dropOffSequence);
		}

		systems.drivingSystem.driveSidewaysSequenceItem(sidewaysDistance, 0);
	}
}