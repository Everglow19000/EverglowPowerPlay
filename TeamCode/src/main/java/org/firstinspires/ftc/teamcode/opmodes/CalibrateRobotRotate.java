package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "CalibrateRobotRotate", group = "Calibrate")
public class CalibrateRobotRotate extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive() && elapsedTime.milliseconds() < 2000){
            drivingSystem.driveMecanum(new Pose(0, 0, 1));
            drivingSystem.positionLogger.update();
        }



		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("moveRotateLog"));
	}
}
