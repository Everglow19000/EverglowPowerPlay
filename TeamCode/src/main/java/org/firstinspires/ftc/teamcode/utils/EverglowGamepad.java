package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EverglowGamepad {
	private final Gamepad gamepad;
	private final ElapsedTime crossHoldTimer;

	private boolean previousA;
	private boolean previousB;
	private boolean previousX;
	private boolean previousY;
	private boolean previousRight;
	private boolean previousLeft;
	private boolean previousUp;
	private boolean previousDown;
	private boolean previousRb;
	private boolean previousRt;
	private boolean previousLb;
	private boolean previousLt;
	private boolean previousShare;

	private boolean currentA;
	private boolean currentB;
	private boolean currentX;
	private boolean currentY;
	private boolean currentRight;
	private boolean currentLeft;
	private boolean currentUp;
	private boolean currentDown;
	private boolean currentRb;
	private boolean currentRt;
	private boolean currentLb;
	private boolean currentLt;
	private boolean currentShare;

	public EverglowGamepad(Gamepad gamepad) {
		this.gamepad = gamepad;
		this.crossHoldTimer = new ElapsedTime();
	}

	/**
	 * cross
	 */
	public boolean a() {
		return currentA && !previousA;
	}

	/**
	 * circle
	 */
	public boolean b() {
		return currentB && !previousB;
	}

	/**
	 * square
	 */
	public boolean x() {
		return currentX && !previousX;
	}

	/**
	 * triangle
	 */
	public boolean y() {
		return currentY && !previousY;
	}

	// aliases with a ps4 controller
	public boolean cross() {
		return a();
	}

	public boolean circle() {
		return b();
	}

	public boolean square() {
		return x();
	}

	public boolean triangle() {
		return y();
	}

	public boolean dpad_left() {
		return currentLeft && !previousLeft;
	}

	public boolean dpad_right() {
		return currentRight && !previousRight;
	}

	public boolean dpad_up() {
		return currentUp && !previousUp;
	}

	public boolean dpad_down() {
		return currentDown && !previousDown;
	}

	public boolean rt() {
		return currentRt && !previousRt;
	}

	public boolean lt() {
		return currentLt && !previousLt;
	}

	public boolean rb() {
		return currentRb && !previousRb;
	}

	public boolean lb() {
		return currentLb && !previousLb;
	}

	public boolean crossHold() {
		return crossHoldTimer.seconds() > 4;
	}

	public boolean share() {
		return currentShare && !previousShare;
	}

	public void update() {
		previousA = currentA;
		previousB = currentB;
		previousX = currentX;
		previousY = currentY;
		previousRight = currentRight;
		previousLeft = currentLeft;
		previousUp = currentUp;
		previousDown = currentDown;

		previousLb = currentLb;
		previousLt = currentLt;
		previousRb = currentRb;
		previousRt = currentRt;

		previousShare = currentShare;

		currentA = gamepad.a;
		currentB = gamepad.b;
		currentX = gamepad.x;
		currentY = gamepad.y;
		currentRight = gamepad.dpad_right;
		currentLeft = gamepad.dpad_left;
		currentUp = gamepad.dpad_up;
		currentDown = gamepad.dpad_down;
		currentLb = gamepad.left_bumper;
		currentLt = gamepad.left_trigger > 0.1;
		currentRb = gamepad.right_bumper;
		currentRt = gamepad.right_trigger > 0.1;
		currentShare = gamepad.share;

		if (!gamepad.a) {
			crossHoldTimer.reset();
		}
	}
}
