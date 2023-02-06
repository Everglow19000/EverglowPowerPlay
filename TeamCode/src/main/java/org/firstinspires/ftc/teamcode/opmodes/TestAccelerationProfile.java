package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.PositionLogger;
import org.firstinspires.ftc.teamcode.systems.SystemCoordinator;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;

@TeleOp(name = "SimpleAccelerationProfile", group = ".Main")
public class TestAccelerationProfile extends LinearOpMode {
	@Override
	public void runOpMode() {
		SystemCoordinator systemCoordinator = new SystemCoordinator(this);
		double d = 150;
		AccelerationProfile accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_X, RobotParameters.MAX_V_X * 0.9, d);
		waitForStart();
		systemCoordinator.drivingSystem.driveForwardByProfile(accelerationProfile);
		systemCoordinator.positionLogger.saveTo(PositionLogger.generateLogFileName("acceleration-profile-sm"));
	}
}
