package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "TemplateTeleop2", group = ".Main")
public class TemplateTeleop2 extends LinearOpMode {
	@Override
	public void runOpMode() {
		waitForStart();
		double d = 100;
		AccelerationProfile accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_Y, RobotParameters.MAX_V_Y, d);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		drivingSystem.driveForwardByProfile(accelerationProfile);
		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("driveByProfile"));
	}
}
