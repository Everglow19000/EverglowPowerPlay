package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.Point2D;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

@TeleOp(name = "TestDriveByPath", group = ".Main")
public class TestDriveByPath extends LinearOpMode {
	@Override
	public void runOpMode() {

		Point2D[] pts = {
				new Point2D(0,0),
				new Point2D(0,100),
				new Point2D(100,100),
		};

		SplinePath spline = new SplinePath(pts);
		Trajectory traj = new Trajectory(spline, RobotParameters.MAX_V_X * 0.5);

		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		waitForStart();
		systemCoordinator.drivingSystem.driveByPath(traj, 0, 0);
		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("drive-path-sm"));
	}
}
