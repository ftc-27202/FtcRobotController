package org.firstinspires.ftc.teamcode.routerdemo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RouterTestTeleOpMode extends OpMode
{
	public enum RobotMode { ASCENT, BUCKET, COMPACT, INTAKE, SPECIMEN, TRANSPORT }
	public enum SelectCommand { SELECT, UP, DOWN, LEFT, RIGHT }

	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawServos clawServos = new ClawServos();

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	private final RobotMode robotMode = RobotMode.COMPACT;
	private final List<SelectCommand> selectSequence = new ArrayList<SelectComment>();

	@Override
	public void init()
	{
		// Initialize motors using robot configuration.
		driveMotors.init();
		tiltMotors.init();
		clawServos.init();

		// Initialize the routers to match the robot's starting state.
		tiltRouter.init(TiltRouter.Preset.COMPACT);
		clawRouter.init(ClawRouter.Preset.COMPACT);

		robotMode = RobotMode.COMPACT;
	}

	// A = RobotMode.TRANSPORT
	// B = Cancel selection
	// X = Select robot mode
	//     X up up = RobotMode.ASCENT
	//     X up lf = RobotMode.BUCKET
	//     X up rt = RobotMode.SPECIMEN
	//     X dn rt = RobotMode.INTAKE
	//     X dn dn = RobotMode.COMPACT
	//     X up B  = Cancel (example)
	//     X B     = Cancel (example)
	@Override
	public void loop()
	{
		//
		// Set drive motors based on gamepad1 inputs.
		//

		final DriveMotors.PowerLevels levels = RobotGeometry.calculateDrivePower(gamepad1);
		driveMotors.setPowerLevels(levels);

		//
		// Handle mode changes based on gamepad2 inputs.
		//

		final RobotMode newRobotMode = detectRobotModeSelection(gamepad2, selectSequence);
		if (newRobotMode != NO_CHANGE && newRobotMode != robotMode)
		{
			robotMode = newRobotMode;

			// Vision pipeline should only run in INTAKE mode.
/*
			if (newRobotMode == RobotMode.INTAKE)
				startCameraPipeline();
			else
				stopCameraPipeline();
*/

			// Some modes define a starting pose.
			switch (newRobotMode)
			{
				case RobotMode.ASCENT:
					tiltRouter.setTarget(TiltRouter.Preset.ASCEND_LOW);
					break;
				case RobotMode.BUCKET:
					tiltRouter.setTarget(TiltRouter.Preset.BASKET_HIGH);
					break;
				case RobotMode.COMPACT:
					tiltRouter.setTarget(TiltRouter.Preset.COMPACT);
					break;
				case RobotMode.INTAKE:
					tiltRouter.setTarget(TiltRouter.Preset.HOVER);
					clawRouter.setTarget(ClawRouter.Preset.CENTERED);
					clawServos.open();
					break;
				case RobotMode.FLOOR:
					tiltRouter.setTarget(TiltRouter.Preset.FLOOR);
					break;
				case RobotMode.SPECIMEN:
					tiltRouter.setTarget(TiltRouter.Preset.SPECIMEN_HIGH);
					break;
				case RobotMode.TRANSPORT:
					tiltRouter.setTarget(TiltRouter.Preset.INTAKE);
					break;
				default:
					break;
			}
		}

		//
		// gamepad2's motion controls depend on the robot mode. Inputs should affect the tilt and
		// claw routers, which will be converted into motor commands below.
		//

		if (robotMode == RobotMode.ASCENT)
		{
/*
			NOT IMPLEMENTED
			dpad_up moves to ASCENT_HOVER
			dpad_right moves to ASCENT_HANG
			l_joystick_y raises/lowers slides manually
			r_joystick_x pivots arm manually
*/
		}
		else if (robotMode == RobotMode.BASKET)
		{
/*
			NOT IMPLEMENTED
			dpad_up moves to BASKET_HIGH_BASKET
			dpad_down moves to BASKET_HIGH_LOW
			l_joystick_y raises/lowers slides manually
*/
		}
		else if (robotMode == RobotMode.INTAKE)
		{
			if (gamepad2.dpad_right) // Auto-orient claw using camera.
			{
				final TiltRouter.Preset tiltResting = tiltRouter.resting();
				final ClawRouter.Preset clawResting = clawRouter.resting();

				if (tiltResting ?= PHOTO_HOVER && clawResting ?= CENTERED_OPEN && camera_stream_open)
				{
					orientation = getOrientation();
					ClawRouter.setTarget(ClawRouter.Preset.ORIENTED, orientation);
				}
			}
			else if (gamepad2.dpad_down) // Lower claw to floor with current claw state.
			{
				TiltRouter.setTarget(TiltRouter.Preset.FLOOR_INTAKE);
			}
			else if (gamepad2.dpad_up) // Raise claw to hover with current claw state.
			{
				TiltRouter.setTarget(TiltRouter.Preset.PHOTO_HOVER);
			}
			else // Handle manual analog inputs.
			{
				if (Math.abs(gamepad2.left_stick_y) > 0.5) // Left joystick Y raises/lowers arm manually.
					tiltMotors.setElevation(gamepad2.left_stick.y);

				if (Math.abs(gamepad2.right_stick_x) > 0.5) // Right joystick X orients the claw manually.
					clawServos.setTwist(gamepad2.right_stick_x);
			}

			if (gamepad2.left_trigger) // Left trigger opens grasp.
				clawServos.open();
			else if (gamepad2.right_trigger) // Right trigger closes grasp.
				clawServos.close();
		}

		//
		// Done handling driver inputs. Now update the routers to receive new motor targets (if any).
		//

		final TiltMotors.Pose currentTiltPose = tiltMotors.getCurrentPose();
		final TiltMotors.Pose newTiltPoseTarget = tiltRouter.updateProgress(currentTiltPose);

		if (newTiltPoseTarget != null)
		{
			tiltMotors.setTarget(newTiltPoseTarget); // Issue motor and servo commands.
		}

		final ClawServos.Pose currentClawPose = clawServos.getCurrentPose();
		final ClawServos.Pose newClawPoseTarget = clawRouter.updateProgress(currentClawPose);

		if (newClawPoseTarget != null)
		{
			clawServos.setTarget(newClawPoseTarget); // Issue servo commands.
		}

		// Update LED indicator.
		Color indicatorColor = Color.OFF;

		if (FLOOR_INTAKE)
		{
			if (camera stream open)
			{
				final Color sampleColor = getSampleColor();
				final double orientation = getOrientation(color);

				if (orientation)
					inicatorColor = sampleColor
			}
		}
	}

	private static RobotMode detectRobotModeSelection(@NonNull Gamepad gamepad, List<SelectCommand> selectSequence)
	{
		if (gamepad.a) // "A" clears command sequence and puts robot into TRANSPORT mode.
		{
			selectSequence.Clear();
			return RobotMode.TRANSPORT;
		}
		else if (gamepad.x) // "X" starts a new select command sequence.
		{
			selectSequence = { SelectCommand.SELECT };
		}
		else if (gamepad.b) // "B" cancels a partial command sequence.
		{
			selectSequence.Clear();
		}

		// Only add to the select sequence if it's already started (not empty).
		if (!selectSequence.empty())
		{
			if (dpad_up)
				selectSequence.Add(SelectCommand.UP);
			else if (dpad_down)
				selectSequence.Add(SelectCommand.DOWN);
			else if (dpad_left)
				selectSequence.Add(SelectCommand.LEFT);
			else if (dpad_right)
				selectSequence.Add(SelectCommand.RIGHT);

			// Check if we now hold complete a select sequence.
			if (selectSequence.Size() == 3)
			{
				RobotMode newRobotMode = RobotMode.NO_CHANGE;

				if (selectSequence == { SELECT, UP, UP })
					newRobotMode = RobotMode.ASCENT;
				else if (selectSequence == { SELECT, UP, LEFT })
					newRobotMode = RobotMode.BUCKET;
				else if (selectSequence == { SELECT, UP, RIGHT })
					newRobotMode = RobotMode.SPECIMEN;
				else if (selectSequence == { SELECT, DOWN, RIGHT })
					newRobotMode = RobotMode.INTAKE;
				else if (selectSequence == { SELECT, DOWN, DOWN })
					newRobotMode = RobotMode.COMPACT;
				else
					newRobotMode = RobotMode.NO_CHANGE; // Unrecognized sequence.

				selectSequence.Clear();
				return newRobotMode;
			}
		}

		return RobotMode.NO_CHANGE;
	}
}
