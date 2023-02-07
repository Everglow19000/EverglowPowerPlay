package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.SplinePath;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.Trajectory;

@TeleOp(name = "DriveByPathTest", group = ".Main")
public class DriveByPathTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		PointD[] pts = {
				new PointD(0, 0),
				new PointD(0, 100),
				new PointD(100, 100),
		};

		SplinePath spline = new SplinePath(pts);
		Trajectory traj = new Trajectory(spline, RobotParameters.MAX_V_X * 0.5);

		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		waitForStart();

		systemCoordinator.drivingSystem.driveByPath(traj, 0, 0);
		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("drive-path-sm"));
	}
}
