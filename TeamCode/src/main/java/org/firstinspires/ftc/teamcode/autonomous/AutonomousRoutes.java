package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.CameraSystem;
import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

public class AutonomousRoutes {
    private final LinearOpMode opMode;
    private final DrivingSystem drivingSystem;
    private final CameraSystem cameraSystem;
    private final ClawSystem clawSystem;

    public AutonomousRoutes(LinearOpMode opMode) {
        this.opMode = opMode;
        drivingSystem = new DrivingSystem(opMode);
        cameraSystem = new CameraSystem(opMode);
        clawSystem = new ClawSystem(opMode);
    }

    public void run(){
        clawSystem.setPosition(ClawSystem.ServoPosition.CLOSED);
        CameraSystem.AprilTagType tagType = cameraSystem.detectAprilTag();
        opMode.telemetry.addData("tag", tagType.toString());
        opMode.telemetry.update();
        switch (tagType){
            case TAG_1:
                drivingSystem.driveSideways(65, 0.5);
                drivingSystem.driveStraight(90, 0.5);
                break;
            case TAG_2:
            default:
                drivingSystem.driveStraight(90, 0.5);
                break;
            case TAG_3:
                drivingSystem.driveSideways(-65, 0.5);
                drivingSystem.driveStraight(90, 0.5);
                break;
        }
    }

    public void testDriveSideways(){
        drivingSystem.driveSideways(60, 0.5);
    }

}
