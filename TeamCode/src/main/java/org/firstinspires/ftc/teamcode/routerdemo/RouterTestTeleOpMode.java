package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RouterTestTeleOpMode extends OpMode
{
	public enum RobotMode
	{NO_CHANGE, ASCENT, BASKET, COMPACT, INTAKE, SPECIMEN, TRANSPORT}

	public enum SelectCommand
	{SELECT, UP, DOWN, LEFT, RIGHT}

	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawServos clawServos = new ClawServos();

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	private final List<SelectCommand> selectSequence = new ArrayList<SelectCommand>();
	RobotMode robotMode = RobotMode.COMPACT;

	@Override
	public void init()
	{
		// Initialize motors using robot configuration.
		driveMotors.init();
		tiltMotors.init();
		clawServos.init();

		// Initialize the routers to match the robot's starting state.
		tiltRouter.init(TiltRouter.Preset.COMPACT);
		clawRouter.init(ClawRouter.Preset.CENTERED);

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
		if (newRobotMode != RobotMode.NO_CHANGE && newRobotMode != robotMode)
		{
			robotMode = newRobotMode;

/*
			// Vision pipeline should only run in INTAKE mode.
			if (newRobotMode == RobotMode.INTAKE)
				startCameraPipeline();
			else
				stopCameraPipeline();
*/

			// Some modes define a starting pose.
			switch (newRobotMode)
			{
				case ASCENT:
					tiltRouter.setTarget(TiltRouter.Preset.ASCENT_LOW_HOVER);
					break;
				case BASKET:
					tiltRouter.setTarget(TiltRouter.Preset.BASKET_HIGH);
					break;
				case COMPACT:
					tiltRouter.setTarget(TiltRouter.Preset.COMPACT);
					break;
				case INTAKE:
					tiltRouter.setTarget(TiltRouter.Preset.INTAKE_HOVER);
					clawRouter.setTarget(ClawRouter.Preset.CENTERED);
					clawServos.open();
					break;
				case SPECIMEN:
					tiltRouter.setTarget(TiltRouter.Preset.SPECIMEN_HIGH);
					break;
				case TRANSPORT:
					tiltRouter.setTarget(TiltRouter.Preset.INTAKE_HOVER);
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

				if (tiltResting == TiltRouter.Preset.INTAKE_HOVER &&
						clawResting == ClawRouter.Preset.CENTERED)
				// && camera_stream_open)
				{
					//orientation = getOrientation();
					//ClawRouter.setTarget(ClawRouter.Preset.ORIENTED, orientation);
				}
			}
			else if (gamepad2.dpad_down) // Lower claw to floor with current claw state.
			{
				tiltRouter.setTarget(TiltRouter.Preset.INTAKE_FLOOR);
			}
			else if (gamepad2.dpad_up) // Raise claw to hover with current claw state.
			{
				tiltRouter.setTarget(TiltRouter.Preset.INTAKE_HOVER);
			}
			else // Handle manual analog inputs.
			{
/*
				if (Math.abs(gamepad2.left_stick_y) > 0.5) // Left joystick Y raises/lowers arm manually.
					tiltMotors.setElevation(gamepad2.left_stick_y);

				if (Math.abs(gamepad2.right_stick_x) > 0.5) // Right joystick X orients the claw manually.
					clawServos.setTwist(gamepad2.right_stick_x);
 */
			}

			if (gamepad2.left_trigger > 0.5) // Left trigger opens grasp.
				; // clawServos.open();
			else if (gamepad2.right_trigger > 0.5) // Right trigger closes grasp.
				; // clawServos.close();
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

/*
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
*/
	}

	static private RobotMode detectRobotModeSelection(@NonNull Gamepad gamepad, List<SelectCommand> selectSequence)
	{
		final List<SelectCommand> ascentSequence = Arrays.asList(
				SelectCommand.SELECT, SelectCommand.UP, SelectCommand.UP);

		final List<SelectCommand> basketSequence = Arrays.asList(
				SelectCommand.SELECT, SelectCommand.UP, SelectCommand.LEFT);

		final List<SelectCommand> specimenSequence = Arrays.asList(
				SelectCommand.SELECT, SelectCommand.UP, SelectCommand.RIGHT);

		final List<SelectCommand> intakeSequence = Arrays.asList(
				SelectCommand.SELECT, SelectCommand.DOWN, SelectCommand.RIGHT);

		final List<SelectCommand> compactSequence = Arrays.asList(
				SelectCommand.SELECT, SelectCommand.DOWN, SelectCommand.RIGHT);

		if (gamepad.a) // "A" clears command sequence and puts robot into TRANSPORT mode.
		{
			selectSequence.clear();
			return RobotMode.TRANSPORT;
		}
		else if (gamepad.x) // "X" starts a new select command sequence.
		{
			selectSequence.clear();
			selectSequence.add(SelectCommand.SELECT);
		}
		else if (gamepad.b) // "B" cancels a partial command sequence.
		{
			selectSequence.clear();
		}

		// Only add to the select sequence if it's already started (not empty).
		if (!selectSequence.isEmpty())
		{
			if (gamepad.dpad_up)
				selectSequence.add(SelectCommand.UP);
			else if (gamepad.dpad_down)
				selectSequence.add(SelectCommand.DOWN);
			else if (gamepad.dpad_left)
				selectSequence.add(SelectCommand.LEFT);
			else if (gamepad.dpad_right)
				selectSequence.add(SelectCommand.RIGHT);

			// Check if we now hold complete a select sequence.
			if (selectSequence.size() == 3)
			{
				RobotMode newRobotMode;

				if (selectSequence.equals(ascentSequence))
					newRobotMode = RobotMode.ASCENT;
				else if (selectSequence.equals(basketSequence))
					newRobotMode = RobotMode.BASKET;
				else if (selectSequence.equals(specimenSequence))
					newRobotMode = RobotMode.SPECIMEN;
				else if (selectSequence.equals(intakeSequence))
					newRobotMode = RobotMode.INTAKE;
				else if (selectSequence.equals(compactSequence))
					newRobotMode = RobotMode.COMPACT;
				else
					newRobotMode = RobotMode.NO_CHANGE; // Unrecognized sequence.

				selectSequence.clear();
				return newRobotMode;
			}
		}

		return RobotMode.NO_CHANGE;
	}
}
