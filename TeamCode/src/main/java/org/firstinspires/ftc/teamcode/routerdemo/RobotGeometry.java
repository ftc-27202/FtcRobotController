package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.Contract;

public class RobotGeometry
{
	public static final int SLIDE_COLLAPSED_MIN = 0;        // Fully collapsed
	public static final int SLIDE_DRIVE_CONFIG = 0;         //
	public static final int SLIDE_SAFE_PASS_THROUGH = 0;    //
	public static final int SLIDE_BASKET_LOW = 2500;        // Maximum allowable extension (check value)
	public static final int SLIDE_BASKET_HIGH = 2500;       // Maximum allowable extension (check value)
	public static final int SLIDE_SPECIMEN_LOW = 2500;      // Maximum allowable extension (check value)
	public static final int SLIDE_SPECIMEN_HIGH = 2500;     // Maximum allowable extension (check value)
	public static final int SLIDE_ASCENT_LOW_HOVER = 2000;  //
	public static final int SLIDE_ASCENT_LOW_HANG = 2000;   //
	public static final int SLIDE_ASCENT_HIGH_HOVER = 2000; //
	public static final int SLIDE_ASCENT_HIGH_HANG = 2000;  //
	public static final int SLIDE_EXTENDED_MAX = 2500;      // Maximum allowable extension (check value)
	public static final int TILT_VERTICAL_DEG = 0;          // Slides are vertical.
	public static final int TILT_FLOOR_MAX_DEG = 70;        // Tilting toward front of robot.
	public static final int ARM_PIVOT_MIN_DEG = -120;       // Claw hovering over floor.
	public static final int ARM_PIVOT_COLLAPSED_DEG = 0;    // Fully collapsed inside with slides.
	public static final int ARM_PIVOT_MAX_DEG = 180;        // Fully extended inline with slides.
	public static final int WRIST_PIVOT_MIN_DEG = -90;      //
	public static final int WRIST_PIVOT_MAX_DEG = 90;       //
	public static final int WRIST_TWIST_MIN_DEG = -90;      //
	public static final int WRIST_TWIST_MAX_DEG = 90;       //

	@NonNull
	@Contract("_ -> new")
	public static DriveMotors.PowerLevels calculateDrivePower(@NonNull Gamepad gamepad)
	{
		final double speed = gamepad.right_trigger > 0 ? 0.6 : 1.0;
		final double turnSpeed = gamepad.right_trigger > 0 ? 0.2 : 1.0;

		// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
		final double axial = -gamepad.left_stick_y;  // Note: pushing stick forward gives negative value
		final double lateral = gamepad.left_stick_x;
		final double yaw = gamepad.right_stick_x * turnSpeed;

		// Combine the joystick requests for each axis-motion to determine each wheel's power.
		// Set up a variable for each drive wheel to save the power level for telemetry.
		double frontLeftPower = axial + lateral + yaw;
		double frontRightPower = axial - lateral - yaw;
		double backLeftPower = axial - lateral + yaw;
		double backRightPower = axial + lateral - yaw;

		// Normalize the values so no wheel power exceeds 100%
		// This ensures that the robot maintains the desired motion.
		double max;
		max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
		max = Math.max(max, Math.abs(backLeftPower));
		max = Math.max(max, Math.abs(backRightPower));

		if (max > 1.0)
		{
			frontLeftPower /= max;
			frontRightPower /= max;
			backLeftPower /= max;
			backRightPower /= max;
		}

		return new DriveMotors.PowerLevels(
				frontLeftPower * speed,
				backLeftPower * speed,
				frontRightPower * speed,
				backRightPower * speed);
	}

	@NonNull
	@Contract("_ -> new")
	public static TiltMotors.Pose toPose(@NonNull TiltRouter.Preset preset)
	{
		switch (preset)
		{
			case COMPACT:
			case PICK_HOVER:
			case PICK:
				return new TiltMotors.Pose( //
						SLIDE_COLLAPSED_MIN, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case DRIVE_CONFIG:
				return new TiltMotors.Pose( //
						SLIDE_DRIVE_CONFIG, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case BASKET_LOW:
				return new TiltMotors.Pose( //
						SLIDE_BASKET_LOW, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case BASKET_HIGH:
				return new TiltMotors.Pose( //
						SLIDE_BASKET_HIGH, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case SPECIMEN_LOW:
				return new TiltMotors.Pose( //
						SLIDE_SPECIMEN_LOW, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case SPECIMEN_HIGH:
				return new TiltMotors.Pose( //
						SLIDE_SPECIMEN_HIGH, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case ASCENT_LOW_HOVER:
				return new TiltMotors.Pose( //
						SLIDE_ASCENT_LOW_HOVER, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case ASCENT_LOW_HANG:
				return new TiltMotors.Pose( //
						SLIDE_ASCENT_LOW_HANG, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case ASCENT_HIGH_HOVER:
				return new TiltMotors.Pose( //
						SLIDE_ASCENT_HIGH_HOVER, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case ASCENT_HIGH_HANG:
				return new TiltMotors.Pose( //
						SLIDE_ASCENT_HIGH_HANG, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition

			case SAFE_PASS_THROUGH:
			default:
				return new TiltMotors.Pose( //
						SLIDE_SAFE_PASS_THROUGH, //
						0, // slidePositionLeft
						0, // slidePositionRight
						0.0, // armPivotPositionLeft
						0.0, // armPivotPositionRight
						0.0, // wristPivotPosition
						0.0); // wristTwistPosition
		}
	}
}
