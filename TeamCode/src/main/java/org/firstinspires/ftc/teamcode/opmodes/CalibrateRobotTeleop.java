package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "CalibrateRobotTeleop", group = "Test")
public class CalibrateRobotTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        telemetry.addLine("Calibrating robot. Put robot in location where it can move forward and press cross. ");
		telemetry.update();
		while (!gamepad1.cross){}
		ElapsedTime elapsedTime = new ElapsedTime();
		PositionLogger positionLogger = new PositionLogger(drivingSystem, this);
		drivingSystem.driveMecanum(new Pose(0, 1, 0));
		while (opModeIsActive() && elapsedTime.milliseconds() < 1000){
			drivingSystem.driveMecanum(new Pose(0, 1, 0));
			positionLogger.update();
		}
		drivingSystem.driveMecanum(new Pose(0, 0, 0));
		positionLogger.saveTo(PositionLogger.generateLogFileName("moveForwardLog"));
		positionLogger.clear();

        telemetry.addLine("Calibrating robot. Put robot in location where it can move sideways and press cross. ");
		telemetry.update();
		while (!gamepad1.cross){}
		elapsedTime.reset();
		drivingSystem.driveMecanum(new Pose(1, 0, 0));
		while (opModeIsActive() && elapsedTime.milliseconds() < 1000){
			drivingSystem.driveMecanum(new Pose(1, 0, 0));
			positionLogger.update();
		}
		drivingSystem.driveMecanum(new Pose(0, 0, 0));
		positionLogger.saveTo(PositionLogger.generateLogFileName("moveSidewaysLog"));
		positionLogger.clear();


		telemetry.addLine("Calibrating robot. Put robot in location where it can rotate and press cross. ");
		telemetry.update();
		while (!gamepad1.cross){}
		elapsedTime.reset();
		drivingSystem.driveMecanum(new Pose(0, 0, 1));
		while (opModeIsActive() && elapsedTime.milliseconds() < 1000){
			drivingSystem.driveMecanum(new Pose(0, 0, 1));
			positionLogger.update();
		}
		drivingSystem.driveMecanum(new Pose(0, 0, 0));
		positionLogger.saveTo(PositionLogger.generateLogFileName("moveRotateLog"));


	}
}
