package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

	private final LinearOpMode opMode;

	private static class RobotState {
		final long time;
		final Pose pose;

		public RobotState(long time, Pose pose) {
			this.time = time;
			this.pose = pose;
		}
	}

	public PositionLogger(DrivingSystem drivingSystem, LinearOpMode opMode) {
		this.drivingSystem = drivingSystem;
		this.opMode = opMode;
		robotStates = new ArrayList<>();
	}

	public void update() {
//        long timeSeconds = (drivingSystem.getLastCycleTime() - startTime) / 1000000000.;
		RobotState robotState = new RobotState(drivingSystem.getLastCycleTime(), drivingSystem.getPosition());
		robotStates.add(robotState);
	}


	public void saveTo(File fileToCreate)  {
		if (robotStates.isEmpty()) {
			return;
		}
		double startTimeNanos = robotStates.get(0).time;

		PrintStream stream = null;

		try {
			// Create the directories containing the file if they don't already exist.
			File parentFile = fileToCreate.getParentFile();
			if (parentFile == null) {
				throw new IOException("saveTo() given invalid file. ");
			}
			//noinspection ResultOfMethodCallIgnored
			parentFile.mkdirs();
			// create the file
			if (!fileToCreate.createNewFile()) {
				throw new IOException("Could not create file");
			}
			// put the needed data in the file
			stream = new PrintStream(new FileOutputStream(fileToCreate), true, "utf-8");
			stream.println("time[sec], x[cm], y[cm], rot[degrees]");
			if (stream.checkError()) {
				throw new IOException("Failed to write to fileToCreate.");
			}
			int i = 0;
			for (RobotState robotState : robotStates) {
				String lineString = String.format("%s, %s, %s, %s",
						(robotState.time - startTimeNanos) / 1e9,
						robotState.pose.x,
						robotState.pose.y,
						Math.toDegrees(robotState.pose.angle)
				);
				stream.println(lineString);
				if (stream.checkError()) {
					throw new IOException("Failed to write to fileToCreate.");
				}
			}
		} catch (IOException e){
			throw new RuntimeException(e);
		}
		finally {
			if (stream != null) {
				stream.close();
			}
		}

	}

	public void clear() {
		robotStates.clear();
	}

	@NonNull
	public static File generateLogFileName(String baseName) {
		File positionLogsDir = new File(AppUtil.FIRST_FOLDER, "Everglow_position_logs");
		String filename = String.format(Locale.US, "%s-%s.csv", AndroidUtils.timestampString(), baseName);
		return /* Current Log: */ new File(positionLogsDir, filename);
	}
}
