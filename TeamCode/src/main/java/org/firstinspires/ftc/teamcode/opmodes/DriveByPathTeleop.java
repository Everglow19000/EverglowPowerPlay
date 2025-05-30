package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.SplinePath;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

@TeleOp(name = "DriveByPathTeleop", group = ".Main")
public class DriveByPathTeleop extends LinearOpMode {
	@Override
	public void runOpMode() {

		PointD[] pts = {
				new PointD(0,0),
				new PointD(0,100),
				new PointD(100,100),
		};

		SplinePath spline = new SplinePath(pts);
		Trajectory traj = new Trajectory(spline, 0 , toRadians(45));
		DrivingSystem drivingSystem = new DrivingSystem(this);

		waitForStart();

		drivingSystem.driveByPath(traj);
	}
}
