package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

public class Autonomous {
    private final OpMode opMode;
    private final DrivingSystem drivingSystem;
    private final CameraSystem cameraSystem;
    private final ClawSystem clawSystem;

    public Autonomous(LinearOpMode opMode) {
        this.opMode = opMode;
        drivingSystem = new DrivingSystem(opMode);
        cameraSystem = new CameraSystem(opMode);
        clawSystem = new ClawSystem(opMode);
    }

    public void run(int mirror){
        clawSystem.setPosition(ClawSystem.ServoPosition.CLOSED);
        CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
        opMode.telemetry.addData("AprilTag: ", tagType.toString());
        opMode.telemetry.update();
        switch (tagType){
            case TAG_1:
                drivingSystem.driveStraight(10, 0.5);
                drivingSystem.driveSideways(60 * mirror, 0.5);
                drivingSystem.driveStraight(80, 0.5);
                break;
            case TAG_2:
                drivingSystem.driveStraight(90, 0.5);
                break;
            case TAG_3:
            default:
                drivingSystem.driveStraight(10, 0.5);
                drivingSystem.driveSideways(-60 * mirror, 0.5);
                drivingSystem.driveStraight(80, 0.5);
                break;
        }
    }

}
