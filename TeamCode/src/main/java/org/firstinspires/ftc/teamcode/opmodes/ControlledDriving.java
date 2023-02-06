package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.PointD;
import org.firstinspires.ftc.teamcode.utils.Pose;

@TeleOp(name = "ControlledDriving")
public class ControlledDriving extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		EverglowGamepad gamepad = new EverglowGamepad(gamepad1);
		DrivingSystem drivingSystem = new DrivingSystem(this);
		//Claw claw = new Claw(this);
		//FourBar fourBar = new FourBar(this);
		//GWheel gWheel = new GWheel(this);

		drivingSystem.maxDrivePower = 0.2;
		drivingSystem.resetStartLocation(new PointD(71. / 2, 71. / 2));

		final int driveTypeCount = 3;
		int driveType = 0;

		//1drivingSystem.resetStartLocation( new PointD(1.5 * drivingSystem.SQUARE_SIZE_CM, -2.5 * drivingSystem.SQUARE_SIZE_CM));

		waitForStart();
		Pose actPowers = new Pose(0, 0, 0);
		while (opModeIsActive()) {
			gamepad.update();


			if (gamepad.square()) {
				driveType = (driveType + 1) % driveTypeCount;
			}

			actPowers.x = -gamepad1.left_stick_x;
			actPowers.y = -gamepad1.left_stick_y;
			actPowers.angle = -gamepad1.right_stick_x;


			switch (driveType) {
				case 0:
				case 2:
					drivingSystem.controlledDriveByAxis2(actPowers);
					break;
				case 1:
					drivingSystem.controlledDriveByAxis3(actPowers);
					break;
			}
            /*
            if (gamepad.rt()) {
                claw.close();
            }
            if (gamepad.lt()) {
                claw.open();
            }

            if (gamepad.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamepad.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }
            if (gamepad.cross()) {
                fourBar.goTo(FourBar.Level.NEUTRAL);
            }

//            if(gamepad.rb()){
//                gWheel.toggleCollect();
//            }
//            if(gamepad.lb()){
//                gWheel.toggleSpit();
//            }

             */
			drivingSystem.printPosition();
			telemetry.update();
		}
	}
}
