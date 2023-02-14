package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "TestMoveTo", group = "Template")
public class TestMoveTo extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = SystemCoordinator.init(this);
		waitForStart();
		systemCoordinator.drivingSystem.move3(new Pose(0, 100, Math.toRadians(90)));
		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("posLog"));
	}
}
