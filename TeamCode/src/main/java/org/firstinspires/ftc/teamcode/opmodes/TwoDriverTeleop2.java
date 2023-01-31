package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "TwoDriverTeleop2", group = ".Main")
public class TwoDriverTeleop2 extends LinearOpMode {

	DrivingSystem drivingSystem;
	ClawSystem claw;
	ElevatorSystem elevator;
	FourBarSystem fourBar;
	GWheelSystem gWheel;


	@Override
	public void runOpMode() {

		drivingSystem = new DrivingSystem(this);
		claw = new ClawSystem(this);
		elevator = new ElevatorSystem(this);
		fourBar = new FourBarSystem(this);
		gWheel = new GWheelSystem(this);

		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);
		Pose actPowers = new Pose(0, 0, 0);

		final double speedDivisor = 4.5; // the amount to divide the speed when finner controls are activated
		double position1 = 0.01;
		double position2 = 0.3;




		waitForStart();
		claw.goTo(ClawSystem.ClawState.OPEN);
		fourBar.goTo(FourBarSystem.FourBarState.PICKUP);

		while (opModeIsActive()) {
			gamepadA.update();
			gamepadB.update();

			// Calculate desired robot velocity
			actPowers.x = -gamepad1.left_stick_x * 0.75;
			actPowers.y = -gamepad1.left_stick_y * 0.75;
			actPowers.angle = -gamepad1.right_stick_x * 0.5;

			// Activate slower driving and turning, for finer adjustment
			if (gamepad1.right_trigger > 0.2) {
				actPowers.x = -gamepad1.left_stick_x / speedDivisor;
				actPowers.y = -gamepad1.left_stick_y / speedDivisor;
				actPowers.angle = -gamepad1.right_stick_x / speedDivisor;
			}

			// Apply calculated velocity to mecanum wheels
			drivingSystem.driveMecanum(actPowers);

//			if (gamepadB.dpad_up()){
//				elevator.goTo(ElevatorSystem.Level.HIGH);
//			}
//			if (gamepadB.dpad_left()){
//				elevator.goTo(ElevatorSystem.Level.MID);
//			}
//			if (gamepadB.dpad_down()){
//				elevator.goTo(ElevatorSystem.Level.LOW);
//			}
//			if (gamepadB.dpad_right()){
//				elevator.goTo(ElevatorSystem.Level.PICKUP);
//			}
			elevator.setPower(gamepad2.left_stick_y * 0.5);
			//fourBar.setPower(gamepad2.right_stick_y * 0.1)

			if(gamepadB.lt()){
				gWheel.toggleSpit();
			}
			if(gamepadB.rt()){
				gWheel.toggleCollect();
			}

			double bla = gamepad2.right_stick_y * 0.1;
			position1 += bla;
			position2 += bla;

			//fourBar.goToD(position1, bla);

			//FourBarSystem.FourBarState state = new FourBarState(position1, position2);


			if (gamepadB.lb()){
				fourBar.goTo(FourBarSystem.FourBarState.DROPOFF);
			}

			if (gamepadB.rb()){
				fourBar.goTo(FourBarSystem.FourBarState.PICKUP);
			}

			if (gamepadB.cross()){
				claw.goTo(ClawSystem.ClawState.CLOSED);
			}
			if (gamepadB.circle()){
				claw.goTo(ClawSystem.ClawState.OPEN);
			}


			telemetry.update();
		}


	}



}
