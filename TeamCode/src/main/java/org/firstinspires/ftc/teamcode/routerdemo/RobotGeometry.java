package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

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

	public static double convertToEncoderPosition(@NonNull ClawRouter.Waypoint waypoint)
	{
		switch (waypoint)
		{
			case OPEN:
				return 1.0;
			case GRASP:
				return 0.5;
			case CLOSED:
			default:
				return 0.0;
		}
	}

	public static TiltRouter.Pose convertToPose(@NonNull TiltRouter.Waypoint waypoint)
	{
		switch (waypoint)
		{
			case COMPACT:
			case PICK_HOVER:
			case PICK:
				return new TiltRouter.Pose( //
						SLIDE_COLLAPSED_MIN, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case DRIVE_CONFIG:
				return new TiltRouter.Pose( //
						SLIDE_DRIVE_CONFIG, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case BASKET_LOW:
				return new TiltRouter.Pose( //
						SLIDE_BASKET_LOW, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case BASKET_HIGH:
				return new TiltRouter.Pose( //
						SLIDE_BASKET_HIGH, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case SPECIMEN_LOW:
				return new TiltRouter.Pose( //
						SLIDE_SPECIMEN_LOW, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case SPECIMEN_HIGH:
				return new TiltRouter.Pose( //
						SLIDE_SPECIMEN_HIGH, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_LOW_HOVER:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_LOW_HOVER, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_LOW_HANG:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_LOW_HANG, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_HIGH_HOVER:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_HIGH_HOVER, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_HIGH_HANG:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_HIGH_HANG, //
						0, //
						0.0, //
						0.0, //
						0.0); //

			case SAFE_PASS_THROUGH:
			default:
				return new TiltRouter.Pose( //
						SLIDE_SAFE_PASS_THROUGH, //
						0, //
						0.0, //
						0.0, //
						0.0); //
		}
	}

	public static RobotMotors.TiltEncoderPositions convertToEncoderPositions(TiltRouter.Pose pose)
	{
		final double armPivotPosition = pose.armPivotAngleDeg / 180.0;

		return new RobotMotors.TiltEncoderPositions(
				(int) (pose.tiltAngleDeg * 123.0),
				pose.slidePosition, // Left slide.
				pose.slidePosition, // Right slide.
				armPivotPosition,
				armPivotPosition,
				pose.wristPivotAngleDeg * 123.0,
				pose.wristTwistAngleDeg * 123.0);
	}

	// This function is the same conversion as the one above, but in the opposite direction.
	public static TiltRouter.Pose convertToPose(RobotMotors.TiltEncoderPositions encoderPositions)
	{
		final int slidePositionAvg = (encoderPositions.slidePositionLeft + encoderPositions.slidePositionRight) / 2;
		final double armPivotPositionAvg = (encoderPositions.armPivotPositionLeft + encoderPositions.armPivotPositionRight) / 2.0;

		return new TiltRouter.Pose(
				encoderPositions.tiltPosition / 123.0,
				slidePositionAvg,
				armPivotPositionAvg,
				encoderPositions.wristPivotPosition / 123.0,
				encoderPositions.wristTwistPosition / 123.0);
	}

	public static RobotMotors.TiltEncoderPositions convertToEncoderPositions(TiltRouter.Waypoint waypoint)
	{
		final TiltRouter.Pose pose = convertToPose(waypoint);
		return convertToEncoderPositions(pose);
	}

	private static boolean valueWithin(double v0, double v1, double delta)
	{
		return Math.abs(v0 - v1) <= delta;
	}

	private static boolean valueWithin(int v0, double v1, double delta)
	{
		return Math.abs(v0 - v1) <= delta;
	}

	public static boolean atWaypoint(TiltRouter.Waypoint waypoint, TiltRouter.Pose currentPose)
	{
		final TiltRouter.Pose waypointPose = convertToPose(waypoint);

		if (!valueWithin(waypointPose.tiltAngleDeg, currentPose.tiltAngleDeg, 5.0))
			return false;

		if (!valueWithin(waypointPose.slidePosition, currentPose.slidePosition, 10))
			return false;

		if (!valueWithin(waypointPose.armPivotAngleDeg, currentPose.armPivotAngleDeg, 5.0))
			return false;

		if (!valueWithin(waypointPose.wristPivotAngleDeg, currentPose.wristPivotAngleDeg, 5.0))
			return false;

		if (!valueWithin(waypointPose.wristTwistAngleDeg, currentPose.wristTwistAngleDeg, 5.0))
			return false;

		return true;
	}
}