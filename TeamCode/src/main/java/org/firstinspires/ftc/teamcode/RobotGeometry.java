package org.firstinspires.ftc.teamcode;

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

	public static TiltRouter.Pose evaluate(@NonNull TiltRouter.Waypoint waypoint)
	{
		switch (waypoint)
		{
			case COMPACT:
			case PICK_HOVER:
			case PICK:
				return new TiltRouter.Pose( //
						SLIDE_COLLAPSED_MIN, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case DRIVE_CONFIG:
				return new TiltRouter.Pose( //
						SLIDE_DRIVE_CONFIG, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case BASKET_LOW:
				return new TiltRouter.Pose( //
						SLIDE_BASKET_LOW, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case BASKET_HIGH:
				return new TiltRouter.Pose( //
						SLIDE_BASKET_HIGH, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case SPECIMEN_LOW:
				return new TiltRouter.Pose( //
						SLIDE_SPECIMEN_LOW, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case SPECIMEN_HIGH:
				return new TiltRouter.Pose( //
						SLIDE_SPECIMEN_HIGH, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_LOW_HOVER:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_LOW_HOVER, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_LOW_HANG:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_LOW_HANG, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_HIGH_HOVER:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_HIGH_HOVER, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case ASCENT_HIGH_HANG:
				return new TiltRouter.Pose( //
						SLIDE_ASCENT_HIGH_HANG, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //

			case SAFE_PASS_THROUGH:
			default:
				return new TiltRouter.Pose( //
						SLIDE_SAFE_PASS_THROUGH, //
						0.0, //
						0.0, //
						0.0, //
						0.0); //
		}
	}
}