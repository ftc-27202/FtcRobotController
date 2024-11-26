package org.firstinspires.ftc.teamcode.routerdemo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RouterTestTeleOpMode extends OpMode
{
	public enum RobotMode { ASCENT, BUCKET, COMPACT, FLOOR_PICK, SELECT, SPECIMEN, TRANSPORT }
	public enum SelectCommand { SELECT, UP, DOWN, LEFT, RIGHT }

	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawServos clawServos = new ClawServos();

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	private final RobotMode robotMode = new RobotMode();

	@Override
	public void init()
	{
		// Initialize motors using robot configuration.
		driveMotors.init();
		tiltMotors.init();
		clawServos.init();

		// Initialize the routers to match the robot's starting state.
		tiltRouter.init(TiltRouter.Preset.COMPACT);
		clawRouter.init(ClawRouter.Preset.OPEN);
	}

	// A = RobotMode.TRANSPORT
	// B = Cancel selection
	// X = Select robot mode
	//     X up up = RobotMode.ASCENT
	//     X up lf = RobotMode.BUCKET
	//     X up rt = RobotMode.SPECIMEN
	//     X dn rt = RobotMode.FLOOR_PICK
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

			// Vision pipeline should only run in PICK mode.
			if (newRobotMode == RobotMode.PICK)
				startCameraPipeline();
			else
				stopCameraPipeline();

			// Go to the default pose for the new mode.
			switch (newRobotMode)
			{
				case RobotMode.ASCENT:
					tiltRouter.setTarget(TiltRouter.Preset.ASCEND_LOW);
					break;
				case RobotMode.BUCKET:
					tiltRouter.setTarget(TiltRouter.Preset.BASKET_HIGH);
					break;
				case RobotMode.SPECIMEN:
					tiltRouter.setTarget(TiltRouter.Preset.SPECIMEN_HIGH);
					break;
				case RobotMode.PICK:
					tiltRouter.setTarget(TiltRouter.Preset.PICK_HOVER);
					clawRouter.setTarget(ClawRouter.Preset.CENTERED_OPEN);
					break;
				case RobotMode.COMPACT:
					tiltRouter.setTarget(TiltRouter.Preset.COMPACT);
					break;
				default:
					break;
			}
		}

		//
		// gamepad2's controls depend on robot mode. Inputs affect the tilt and claw routers, which
		// will be converted into motor commands below.
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
		else if (robotMode == RobotMode.PICK)
		{
			if (gamepad2.dpad_left)
			{
				// Driver requested centered (not auto-orientated) claw.
				ClawRouter.setTarget(ClawRouter.Preset.CENTERED_OPEN);
			}
			else if (gamepad2.dpad_right)
			{
				// Driver requested auto claw orientation. Only proceed if robot is ready.
				final TiltRouter.Preset tiltResting = tiltRouter.resting();
				final ClawRouter.Preset clawResting = clawRouter.resting();

				if (tiltResting ?= PHOTO_HOVER && clawResting ?= CENTERED_OPEN && camera_stream_open)
				{
					orientation = getOrientation();
					ClawRouter.setTarget(ClawRouter.Preset.ORIENTED_OPEN, orientation);
				}
			}
			else if (gamepad2.dpad_down)
			{
				// Driver requested to drop claw to floor.
				TiltRouter.setTarget(TiltRouter.Preset.FLOOR_PICK);
			}
			else if (gamepad2.dpad_up)
			{
				// Driver requested to raise claw to hover.
				TiltRouter.setTarget(TiltRouter.Preset.PHOTO_HOVER);
			}
			else
			{
				// If no dpad inputs are active then handle any manual stick inputs.
				if (Math.abs(gamepad2.l_joystick_y) > 0.5)
					; // l_joystick_y raises/lowers arm manually
				if (Math.abs(gamepad2.r_joystick_x) > 0.5)
					; // r_joystick_x twists orientation manually

				// right trigger closes grasp

				// left trigger opens grasp
			}
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

		if (FLOOR_PICK)
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

	private RobotMode detectRobotModeSelection(@NonNull Gamepad gamepad, List<SelectCommand> selectSequence)
	{
		if (gamepad.a)
		{
			// "A" button clears any partial command sequence and immediately puts robot into TRANSPORT mode.
			selectSequence.Clear();
			return RobotMode.TRANSPORT;
		}
		else if (gamepad.x)
		{
			// "X" button starts a new select command sequence but doesn't not affect the current robot mode.
			selectSequence = { SelectCommand.SELECT };
		}
		else if (gamepad.b)
		{
			// "B" button cancels a partial command sequence.
			selectSequence.Clear();
		}

		// Only add to the select sequence if it's started (not empty).
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
					newRobotMode = RobotMode.FLOOR_PICK;
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
