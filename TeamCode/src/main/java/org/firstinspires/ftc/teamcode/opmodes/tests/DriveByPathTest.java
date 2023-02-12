package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.SplinePath;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.Trajectory;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.StateMachine.Sequence;

@TeleOp(name = "DriveByPathTest", group = "Test")
public class DriveByPathTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		PointD[] pts = {
				new PointD(0, 0),
				new PointD(0, 120),
//				new PointD(-27.6, 135.2),
		};

		SplinePath spline = new SplinePath(pts);
		Trajectory traj = new Trajectory(spline, RobotParameters.MAX_V_X * 0.5);

		SystemCoordinator systemCoordinator = SystemCoordinator.init(this);
		waitForStart();

		systemCoordinator.executeSequence(new Sequence(
				systemCoordinator.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PRE_PICKUP),
				systemCoordinator.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 10),
				systemCoordinator.fourBarSystem.goToSequenceItem(FourBarSystem.Position.PICKUP),
				systemCoordinator.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.PICKUP),
				systemCoordinator.sleepingSystem.goToSequenceItem(200),
				systemCoordinator.clawSystem.goToSequenceItem(ClawSystem.Position.CLOSED, 1),
				systemCoordinator.sleepingSystem.goToSequenceItem(500),
				systemCoordinator.elevatorSystem.goToSequenceItem(ElevatorSystem.Level.HIGH),
				systemCoordinator.fourBarSystem.goToSequenceItem(FourBarSystem.Position.DROPOFF)
				));
		systemCoordinator.drivingSystem.driveByPath(traj, 0, 1);
//		systemCoordinator.waitForSequencesDone();
//		systemCoordinator.executeSequence(new Sequence(
//				systemCoordinator.clawSystem.goToSequenceItem(ClawSystem.Position.OPEN, 1)
//		));

//		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("drive-path-sm"));
		systemCoordinator.sleep(1000000000);
	}
}
