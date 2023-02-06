package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.utils.Pose;
import org.firstinspires.ftc.teamcode.utils.PositionLogger;

@TeleOp(name = "CalibrateRobotForward", group = "Calibrate")
public class CalibrateRobotForwards extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }
		drivingSystem.driveY(20);
		drivingSystem.positionLogger.saveTo(PositionLogger.generateLogFileName("moveForwardLog"));
	}
}
