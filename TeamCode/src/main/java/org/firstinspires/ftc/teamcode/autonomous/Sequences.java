package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.ClawSystem;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.FourBarSystem;

public class Sequences {
    private final LinearOpMode opMode;
    private final FourBarSystem fourBarSystem;
    private final ElevatorSystem elevatorSystem;
    private final ClawSystem clawSystem;

    public Sequences(LinearOpMode opMode, FourBarSystem fourBarSystem, ElevatorSystem elevatorSystem, ClawSystem clawSystem) {
        this.opMode = opMode;
        this.fourBarSystem = fourBarSystem;
        this.elevatorSystem = elevatorSystem;
        this.clawSystem = clawSystem;
    }

    public void pickUp(ElevatorSystem.Level level) {
        elevatorSystem.goTo(ElevatorSystem.Level.PICKUP);
        clawSystem.goTo(ClawSystem.ClawState.CLOSED);

        elevatorSystem.goTo(level);
        fourBarSystem.goTo(FourBarSystem.FourBarState.DROPOFF);
    }

    public void drop() {
        clawSystem.goTo(ClawSystem.ClawState.OPEN);
        elevatorSystem.goTo(ElevatorSystem.Level.PRE_PICKUP);
        fourBarSystem.goTo(FourBarSystem.FourBarState.PICKUP);
    }



}
