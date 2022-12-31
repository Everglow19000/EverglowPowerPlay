package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "CalibrateRobotSideways", group = "Calibrate")
public class CalibrateRobotSideways extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }
		drivingSystem.driveSideways(180, 1);
		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("moveSidewaysLog"));
	}
}
