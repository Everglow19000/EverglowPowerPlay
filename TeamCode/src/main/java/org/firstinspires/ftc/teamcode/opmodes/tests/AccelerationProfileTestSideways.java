package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.DriveByPath.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.RobotParameters;

@TeleOp(name = "AccelerationProfileTestSideways", group = "Test")
public class AccelerationProfileTestSideways extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		double d = 30;
		AccelerationProfile accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_X, RobotParameters.MAX_V_X * 0.9, d);
		waitForStart();
		systemCoordinator.drivingSystem.driveSidewaysByProfile(accelerationProfile);
		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("acceleration-profile-sm"));
	}
}
