package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.Contract;

public class RobotGeometry
{
	public static final int SLIDE_COLLAPSED = 0;            // Fully collapsed
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

	public static final int TILT_COMPACT = 100;             //
	public static final int TILT_BASKET = 0;                //
	public static final int TILT_VERTICAL = 300;            //
	public static final int TILT_FLOOR_HOVER = 500;         //
	public static final int TILT_FLOOR_PICK = 600;          // Tilting toward front of robot.

	public static final double ARM_PIVOT_MIN = 0.0;         // Claw hovering over floor.
	public static final double ARM_PIVOT_COMPACT = 0.5;     //
	public static final double ARM_PIVOT_HOVER = 0.4;       //
	public static final double ARM_PIVOT_MAX = 1.0;         // Fully extended inline with slides.

	public static final double WRIST_PIVOT_MIN = 0.0;       //
	public static final double WRIST_PIVOT_STRAIGHT = 0.5;  //
	public static final double WRIST_PIVOT_MAX = 1.0;       //

	public static final double CLAW_TWIST_MIN = 0.0;        // 90 deg CCW
	public static final double CLAW_TWIST_MID = 0.5;        //  0 deg
	public static final double CLAW_TWIST_MAX = 1.0;        // 90 deg CW

	public static final double CLAW_OPEN = 1.0;             //
	public static final double CLAW_CLOSED = 0.0;           //

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
				frontRightPower * speed,
				backLeftPower * speed,
				backRightPower * speed);
	}

	@NonNull
	@Contract("_ -> new")
	public static TiltMotors.Pose toPose(@NonNull TiltRouter.NamedPose namedPose)
	{
		switch (namedPose)
		{
			case COMPACT:
				return new TiltMotors.Pose(
						TILT_COMPACT,
						SLIDE_COLLAPSED, SLIDE_COLLAPSED,
						ARM_PIVOT_COMPACT, ARM_PIVOT_COMPACT,
						WRIST_PIVOT_STRAIGHT);

			case INTAKE_HOVER:
			case INTAKE_FLOOR:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_COLLAPSED, SLIDE_COLLAPSED,
						ARM_PIVOT_HOVER, ARM_PIVOT_HOVER,
						WRIST_PIVOT_STRAIGHT);

			case TRANSPORT:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_COLLAPSED, SLIDE_COLLAPSED,
						0.0, 0.0,
						0.0);

			case BASKET_LOW:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_BASKET_LOW, SLIDE_BASKET_LOW,
						0.0, 0.0,
						0.0);

			case BASKET_HIGH:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_BASKET_HIGH, SLIDE_BASKET_HIGH,
						0.0, 0.0,
						0.0);

			case SPECIMEN_LOW:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_SPECIMEN_LOW, SLIDE_SPECIMEN_LOW,
						0.0, 0.0,
						0.0);

			case SPECIMEN_HIGH:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_SPECIMEN_HIGH, SLIDE_SPECIMEN_HIGH,
						0.0, 0.0,
						0.0);

			case ASCENT_LOW_HOVER:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_ASCENT_LOW_HOVER, SLIDE_ASCENT_LOW_HOVER,
						0.0, 0.0,
						0.0);

			case ASCENT_LOW_HANG:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_ASCENT_LOW_HANG, SLIDE_ASCENT_LOW_HANG,
						0.0, 0.0,
						0.0);

			case ASCENT_HIGH_HOVER:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_ASCENT_HIGH_HOVER, SLIDE_ASCENT_HIGH_HOVER,
						0.0, 0.0,
						0.0);

			case ASCENT_HIGH_HANG:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_COLLAPSED, SLIDE_COLLAPSED,
						0.1, 0.0,
						0.0);

			case SAFE_PASS_THROUGH:
			default:
				return new TiltMotors.Pose(
						TILT_FLOOR_HOVER,
						SLIDE_SAFE_PASS_THROUGH, SLIDE_SAFE_PASS_THROUGH,
						0.0, 0.0,
						0.0);
		}
	}
}
