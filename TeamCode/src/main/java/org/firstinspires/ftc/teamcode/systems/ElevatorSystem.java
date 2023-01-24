package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotParameters;
import org.firstinspires.ftc.teamcode.utils.AccelerationProfile;
import org.firstinspires.ftc.teamcode.utils.Pose;

/**
 * A class for handling the elevator system.
 */
public class ElevatorSystem {

	AccelerationProfile accelerationProfile;
	/**
	 * Enum encapsulating all the positions the system should reach.
	 */
	public enum Level {
		PICKUP(0), LOW(-1833), MID(-2914), HIGH(-2914);

		public final int state;

		Level(int state) {
			this.state = state;
		}
	}

	private final DcMotor left;
	private final DcMotor right;

	public ElevatorSystem(OpMode opMode) {
		left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
		right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

		left.setDirection(DcMotor.Direction.REVERSE);
		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		left.setTargetPosition(0);
		right.setTargetPosition(0);

		left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		left.setPower(0.7);
		right.setPower(0.7);
	}

	/**
	 * Moves the elevator to the specified state.
	 *
	 * @param level The level to move the elevator to.
	 */
	public void goTo(Level level) {
		left.setTargetPosition(level.state);
		right.setTargetPosition(level.state);
	}

	public void goToAccelerationProfile(Level level){
		accelerationProfile = new AccelerationProfile(1,1,level.state);
		ElapsedTime elapsedTime = new ElapsedTime();

		double power;
		while(elapsedTime.seconds() < accelerationProfile.finalTime()){
			power = accelerationProfile.velocity(elapsedTime.seconds()) / RobotParameters.MAX_V_Y;
			left.setPower(power);
			right.setPower(power);
		}
	}
}
