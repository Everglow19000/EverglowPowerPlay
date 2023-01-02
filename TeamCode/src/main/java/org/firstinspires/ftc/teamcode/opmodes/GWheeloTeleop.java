package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "GWheeloTeleop", group = ".Main")
public class GWheeloTeleop extends LinearOpMode {


	@Override
	public void runOpMode() {
		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		GWheelSystem gWheelSystem = new GWheelSystem(this);
		waitForStart();

		while (opModeIsActive()) {
			TelemetryPacket packet = new TelemetryPacket();

			gamepadA.update();
			Pose actPowers = new Pose();
			actPowers.x = -gamepad1.left_stick_x * 0.75;
			actPowers.y = -gamepad1.left_stick_y * 0.75;
			actPowers.angle = -gamepad1.right_stick_x * 0.5;

			drivingSystem.driveMecanum(actPowers);
			if(gamepadA.circle()){
				gWheelSystem.toggleCollect();
			}
			if(gamepadA.square()){
				gWheelSystem.toggleSpit();
			}
		}
	}
}
