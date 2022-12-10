package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.systems.DrivingSystem;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Logs the position of the robot, and saves it to a csv file.
 */
public class PositionLogger {
    private final DrivingSystem drivingSystem;
    private final List<RobotState> robotStates;
    static class RobotState{
        public RobotState(long time, Pose pose) {
            this.time = time;
            this.pose = pose;
        }

        long time;
        Pose pose;
    }

    public PositionLogger(DrivingSystem drivingSystem) {
        this.drivingSystem = drivingSystem;
        robotStates = new ArrayList<>();
    }

    public void update(){
//        long timeSeconds = (drivingSystem.getLastCycleTime() - startTime) / 1000000000.;

        RobotState robotState = new RobotState(drivingSystem.getLastCycleTime(), drivingSystem.getPosition());
        robotStates.add(robotState);
    }

    public void save(){
        if (robotStates.isEmpty()){
            return;
        }
        File positionLogsDir = new File(AppUtil.ROOT_FOLDER, "everglow_position_logs");
        String filename = String.format(Locale.US, "positionLog-%s.csv", AndroidUtils.timestampString());
        File currentLog = new File(positionLogsDir, filename);
        double startTimeMicros = robotStates.get(0).time;
        try(PrintStream stream = new PrintStream(new FileOutputStream(currentLog), true,"utf-8")) {
            stream.println("time[sec], x[cm], y[cm], rot[degrees]");
            for (RobotState robotState : robotStates) {
                String lineString = String.format("%s, %s, %s, %s",
                        (robotState.time - startTimeMicros) / 1000000000.,
                        robotState.pose.x,
                        robotState.pose.y,
                        Math.toDegrees(robotState.pose.angle)
                );
                stream.println(lineString);
            }
        }catch (IOException e){
            throw new RuntimeException(e);
        }

    }
}
