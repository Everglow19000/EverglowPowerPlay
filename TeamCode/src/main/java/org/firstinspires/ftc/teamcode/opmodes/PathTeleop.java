package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Claw;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.FourBar;
import org.firstinspires.ftc.teamcode.utils.EverglowGamepad;
import org.firstinspires.ftc.teamcode.utils.Path;

@TeleOp(name = "PathTeleop")
public class PathTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        EverglowGamepad gamePad = new EverglowGamepad(gamepad1);

        DrivingSystem drivingSystem = new DrivingSystem(this);
        Claw claw = new Claw(this);
        FourBar fourBar = new FourBar(this);

        waitForStart();

        while (opModeIsActive()) {
            gamePad.update();

            drivingSystem.driveByPath(new Path()); //TODO: fill in the path

/*            if (gamePad.rt()) {
                claw.close();
            }
            if (gamePad.lt()) {
                claw.open();
            }

            if (gamePad.triangle()) {
                fourBar.goTo(FourBar.Level.PICKUP);
            }
            if (gamePad.circle()) {
                fourBar.goTo(FourBar.Level.DROPOFF);
            }
            if (gamePad.cross()) {
                fourBar.goTo(FourBar.Level.NEUTRAL);
            }*/

            drivingSystem.printPosition();
            telemetry.update();
        }
    }
}
