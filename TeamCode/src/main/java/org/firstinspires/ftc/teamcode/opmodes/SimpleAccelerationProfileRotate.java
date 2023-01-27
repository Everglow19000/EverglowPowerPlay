package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "SimpleAccelerationProfileRotate", group = ".Main")
public class SimpleAccelerationProfileRotate extends LinearOpMode {
	@Override
	public void runOpMode() {
		double d = Math.toRadians(180);
		AccelerationProfile accelerationProfile = new AccelerationProfile(RobotParameters.MAX_A_ROT, RobotParameters.MAX_V_ROT * 0.9, d);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		waitForStart();
		drivingSystem.rotateByAccelerationProfile(accelerationProfile);
		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("updated_profile"));
	}
}