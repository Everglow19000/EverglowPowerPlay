package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "SimpleAccelerationProfile", group = ".Main")
public class SimpleAccelerationProfile extends LinearOpMode {
	@Override
	public void runOpMode() {
		double d = 150;
		AccelerationProfile accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_X, RobotParameters.MAX_V_X * 0.9, d);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		waitForStart();
		drivingSystem.driveForwardByProfile(accelerationProfile);
		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("updated_profile"));
	}
}