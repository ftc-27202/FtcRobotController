package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RouterTestTeleOpMode extends OpMode
{
	// The robot is always in one mode, based its current objective.
	private enum RobotMode
	{ASCENT, BASKET, COMPACT, INTAKE, SPECIMEN, TRANSPORT}

	// These commands (taken from gamepad inputs) are strung together to request a new robot mode.
	private enum RobotModeSelectCommand
	{SELECT, UP, DOWN, LEFT, RIGHT}

	// Variables for interacting with motors and servos.
	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawServos clawServos = new ClawServos();

	// Routers for tracking and planning motor/servo targets.
	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	// Initialize robot mode to reflect its state when starting a match.
	RobotMode robotMode = RobotMode.COMPACT;

	// Initialize mode command sequence to be empty.
	private final List<RobotModeSelectCommand> commandSequence = new ArrayList<RobotModeSelectCommand>();

	@Override
	public void init()
	{
		// Initialize motors and servos.
		driveMotors.init();
		tiltMotors.init();
		clawServos.init();

		// Initialize routers to reflect the robot's starting state.
		tiltRouter.init(TiltRouter.NamedPose.COMPACT);
		clawRouter.init(ClawRouter.NamedPose.CENTERED);
	}

	@Override
	public void loop()
	{
		//
		// Calculate and set drive motors based on gamepad1 inputs.
		//

		final DriveMotors.PowerLevels driveLevels = RobotGeometry.calculateDrivePower(gamepad1);
		driveMotors.setPowerLevels(driveLevels);

		//
		// Detect robot mode requests based on gamepad2 inputs.
		//

		final RobotMode newRobotMode = checkForCompletedCommandSequence(gamepad2, commandSequence, robotMode);
		if (newRobotMode != robotMode)
		{
			// Robot mode change has been requested.
			robotMode = newRobotMode;

			// Check if the new mode requires a new pose.
			switch (newRobotMode)
			{
				case ASCENT:
					tiltRouter.setTarget(TiltRouter.NamedPose.ASCENT_LOW_HOVER);
					break;
				case BASKET:
					tiltRouter.setTarget(TiltRouter.NamedPose.BASKET_HIGH);
					break;
				case COMPACT:
					tiltRouter.setTarget(TiltRouter.NamedPose.COMPACT);
					break;
				case INTAKE:
					tiltRouter.setTarget(TiltRouter.NamedPose.INTAKE_HOVER);
					clawRouter.setRestingPose(ClawRouter.NamedPose.CENTERED);
					clawServos.open();
					break;
				case SPECIMEN:
					tiltRouter.setTarget(TiltRouter.NamedPose.SPECIMEN_HIGH);
					break;
				case TRANSPORT:
					tiltRouter.setTarget(TiltRouter.NamedPose.INTAKE_HOVER);
					break;
				default:
					break;
			}

			// Optional: Start or stop vision pipeline depending on if robotMode == INTAKE.
		}

		//
		// Update router targets based on gamepad2 inputs (if any). The inputs are interpretted
		// differently depending on which mode the robot is in.
		//

		if (robotMode == RobotMode.ASCENT)
		{
			; // NOT IMPLEMENTED YET
		}
		else if (robotMode == RobotMode.BASKET)
		{
			; // NOT IMPLEMENTED YET
		}
		else if (robotMode == RobotMode.INTAKE)
		{
			if (gamepad2.dpad_right) // Auto-orient claw using camera.
			{
				final TiltRouter.NamedPose tiltResting = tiltRouter.resting();
				final ClawRouter.NamedPose clawResting = clawRouter.resting();

				if (tiltResting == TiltRouter.NamedPose.INTAKE_HOVER &&
						clawResting == ClawRouter.NamedPose.CENTERED /*&& camera_stream_open*/)
				{
					; // orientation = getOrientation(); ClawRouter.setTarget(ClawRouter.NamedPose.ORIENTED, orientation);
				}
			}
			else if (gamepad2.dpad_down) // Lower claw to floor with current claw state.
			{
				tiltRouter.setTarget(TiltRouter.NamedPose.INTAKE_FLOOR);
			}
			else if (gamepad2.dpad_up) // Raise claw to hover with current claw state.
			{
				tiltRouter.setTarget(TiltRouter.NamedPose.INTAKE_HOVER);
			}
			else // Handle manual joystick inputs.
			{
/*
				if (Math.abs(gamepad2.left_stick_y) > gamepad2.joystickDeadzone) // Left joystick Y raises/lowers arm manually.
					tiltMotors.setElevation(gamepad2.left_stick_y);

				if (Math.abs(gamepad2.right_stick_x) > gamepad2.joystickDeadzoen) // Right joystick X orients the claw manually.
					clawServos.setTwist(gamepad2.right_stick_x);
 */
			}

			if (gamepad2.left_trigger > 0.1) // Left trigger opens grasp.
				clawServos.open();
			else if (gamepad2.right_trigger > 0.1) // Right trigger closes grasp.
				clawServos.close();
		}

		//
		// Done handling driver inputs. Finally, inform the routers of actual motor positions to see if changes are needed.
		//

		final TiltMotors.Pose currentTiltPose = tiltMotors.getCurrentPose();
		final TiltMotors.Pose newTiltPoseTarget = tiltRouter.updateProgress(currentTiltPose);

		if (newTiltPoseTarget != null)
		{
			tiltMotors.setTarget(newTiltPoseTarget); // Issue tilt motor and servo commands.
		}

		final ClawServos.Pose currentClawPose = clawServos.getCurrentPose();
		final ClawServos.Pose newClawPoseTarget = clawRouter.updateProgress(currentClawPose);

		if (newClawPoseTarget != null)
		{
			clawServos.setTarget(newClawPoseTarget); // Issue claw servo commands.
		}

		// Optional: Update LED indicator.
	}

	/*
	 * Call this function in each main loop iteration to check if the user has completed a mode
	 * select sequence with the dpad. If so, return the new mode. Otherwise return the old robot
	 * mode to indicate that nothing has changed.
	 *
	 * Gamepad button mapping:
	 *   A = RobotMode.TRANSPORT
	 *   B = Cancel selection
	 *   X = Select robot mode
	 *       X up up = RobotMode.ASCENT
	 *       X up lf = RobotMode.BUCKET
	 *       X up rt = RobotMode.SPECIMEN
	 *       X dn rt = RobotMode.INTAKE
	 *       X dn dn = RobotMode.COMPACT
	 *       X up B  = Cancel (example)
	 *       X B     = Cancel (example)
	 */
	static private RobotMode checkForCompletedCommandSequence(@NonNull Gamepad gamepad,
		List<RobotModeSelectCommand> selectSequence, RobotMode oldRobotMode)
	{
		if (gamepad.a) // "A" clears command sequence and puts robot into TRANSPORT mode.
		{
			selectSequence.clear();
			return RobotMode.TRANSPORT;
		}
		else if (gamepad.x) // "X" starts a new select command sequence.
		{
			selectSequence.clear();
			selectSequence.add(RobotModeSelectCommand.SELECT);
		}
		else if (gamepad.b) // "B" cancels the current command sequence (if any).
		{
			selectSequence.clear();
		}

		// Only add to the select sequence if it has been started (not empty).
		if (!selectSequence.isEmpty())
		{
			if (gamepad.dpad_up)
				selectSequence.add(RobotModeSelectCommand.UP);
			else if (gamepad.dpad_down)
				selectSequence.add(RobotModeSelectCommand.DOWN);
			else if (gamepad.dpad_left)
				selectSequence.add(RobotModeSelectCommand.LEFT);
			else if (gamepad.dpad_right)
				selectSequence.add(RobotModeSelectCommand.RIGHT);

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
					newRobotMode = oldRobotMode; // Unrecognized sequence.

				selectSequence.clear();
				return newRobotMode;
			}
		}

		return oldRobotMode;
	}

	//
	// Valid command sequences to compare against.
	//

	private static final List<RobotModeSelectCommand> ascentSequence = Arrays.asList(
			RobotModeSelectCommand.SELECT, RobotModeSelectCommand.UP, RobotModeSelectCommand.UP);

	private static final List<RobotModeSelectCommand> basketSequence = Arrays.asList(
			RobotModeSelectCommand.SELECT, RobotModeSelectCommand.UP, RobotModeSelectCommand.LEFT);

	private static final List<RobotModeSelectCommand> specimenSequence = Arrays.asList(
			RobotModeSelectCommand.SELECT, RobotModeSelectCommand.UP, RobotModeSelectCommand.RIGHT);

	private static final List<RobotModeSelectCommand> intakeSequence = Arrays.asList(
			RobotModeSelectCommand.SELECT, RobotModeSelectCommand.DOWN, RobotModeSelectCommand.RIGHT);

	private static final List<RobotModeSelectCommand> compactSequence = Arrays.asList(
			RobotModeSelectCommand.SELECT, RobotModeSelectCommand.DOWN, RobotModeSelectCommand.RIGHT);
}
