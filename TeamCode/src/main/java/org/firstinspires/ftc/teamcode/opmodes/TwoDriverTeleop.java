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

@TeleOp(name = "TwoDriverTeleop", group = ".Main")
public class TwoDriverTeleop extends LinearOpMode implements Runnable {

	DrivingSystem drivingSystem;
	ClawSystem claw;
	ElevatorSystem elevator;
	FourBarSystem fourBar;
	GWheelSystem gWheel;


	public void pickUp(ElevatorSystem.Level level) {
		elevator.goTo(ElevatorSystem.Level.PICKUP);
		claw.goTo(ClawSystem.ClawState.CLOSED);

		elevator.goTo(level);
		fourBar.goTo(FourBarSystem.FourBarState.DROPOFF);
	}


	public void drop() {
		claw.goTo(ClawSystem.ClawState.OPEN);
		elevator.goTo(ElevatorSystem.Level.PRE_PICKUP);
		fourBar.goTo(FourBarSystem.FourBarState.PICKUP);
	}


	public void run() {
		EverglowGamepad gamepadB = new EverglowGamepad(gamepad2);

		ClawSystem.ClawState clawPosition = ClawSystem.ClawState.OPEN;
		FourBarSystem.FourBarState fourBarPosition = FourBarSystem.FourBarState.DROPOFF;

		// reset claw position
		claw.goTo(clawPosition);
//		fourBar.goTo(0.3);

		while (opModeIsActive()) {
			gamepadB.update();

			if(gamepadB.triangle()) {
				pickUp(ElevatorSystem.Level.HIGH);
			}

			if(gamepadB.circle()) {
				pickUp(ElevatorSystem.Level.MID);
			}

			if(gamepadB.x()) {
				pickUp(ElevatorSystem.Level.LOW);
			}


			if(gamepadB.dpad_up() || gamepadB.dpad_down() || gamepadB.dpad_right() || gamepadB.dpad_left()) {
				drop();
			}


			if(gamepadB.lt()){
				gWheel.toggleSpit();
			}
			if(gamepadB.rt()){
				gWheel.toggleCollect();
			}

			telemetry.addData("0:",  0);
		}
	}

	@Override
	public void runOpMode() {

		drivingSystem = new DrivingSystem(this);
		claw = new ClawSystem(this);
		elevator = new ElevatorSystem(this);
		fourBar = new FourBarSystem(this);
		gWheel = new GWheelSystem(this);

		Thread thread1 = new Thread(this);

		EverglowGamepad gamepadA = new EverglowGamepad(gamepad1);
		Pose actPowers = new Pose(0, 0, 0);

		final double speedDivisor = 4.5; // the amount to divide the speed when finner controls are activated



		waitForStart();

		thread1.start();

		while (opModeIsActive()) {
			gamepadA.update();

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

			// Telemetry
			//drivingSystem.printPosition();
			telemetry.addData("1:",  1);
			telemetry.update();
		}


	}



}
