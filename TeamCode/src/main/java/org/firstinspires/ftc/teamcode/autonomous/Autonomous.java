package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutonomousRoutes autonomous = new AutonomousRoutes(this);
		telemetry.addLine("Ready");
		telemetry.update();
		waitForStart();
		if (opModeIsActive()) {
			autonomous.run(1);
			//autonomous.runOld(1);
			//autonomous.runOld2();
		}
	}
}