package org.firstinspires.ftc.teamcode.opmodes;

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

		PointD p1 = new PointD(0,0);
		PointD p2 = new PointD(0,150);
		PointD p3 = new PointD(-150,150);
		PointD p4 = new PointD(-150,300);
		PointD[] pts = {p1,p2,p3, p4};

		SplinePath spline = new SplinePath(pts);
		//Trajectory traj = new Trajectory(spline);
		DrivingSystem drivingSystem = new DrivingSystem(this);

		waitForStart();

		//drivingSystem.driveByPath(traj);

	}
}
