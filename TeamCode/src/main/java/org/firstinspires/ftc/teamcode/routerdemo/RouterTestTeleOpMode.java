package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RouterTestTeleOpMode extends OpMode
{
	// The robot is always in one other these modes based its current objective.
	private enum RobotMode
	{ASCENT, BASKET, COMPACT, INTAKE, SPECIMEN, TRANSPORT}

	// These commands (issued by dpad) are strung together to request a new robot mode.
	private enum RobotModeSelectCommand
	{NONE, SELECT, UP, DOWN, LEFT, RIGHT}

	// Variables for interacting with motors and servos.
	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawMotors clawMotors = new ClawMotors();

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
		clawMotors.init();

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

		final RobotMode newRobotMode = updateModeSelectSequence(gamepad2, robotMode);
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
					clawMotors.open();
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
					clawMotors.setTwist(gamepad2.right_stick_x);
*/
			}

			if (gamepad2.left_trigger > 0.1) // Left trigger opens grasp.
				clawMotors.open();
			else if (gamepad2.right_trigger > 0.1) // Right trigger closes grasp.
				clawMotors.close();
		}

		//
		// Done handling driver inputs. Finally, inform the routers of the current motor positions to
		// see if it triggers updated targets.
		//

		final TiltMotors.Pose currentTiltPose = tiltMotors.getCurrentPose();
		final TiltMotors.Pose newTiltPoseTarget = tiltRouter.updateProgress(currentTiltPose);

		if (newTiltPoseTarget != null)
		{
			tiltMotors.setTarget(newTiltPoseTarget); // Issue tilt motor commands.
		}

		final ClawMotors.Pose currentClawPose = clawMotors.getCurrentPose();
		final ClawMotors.Pose newClawPoseTarget = clawRouter.updateProgress(currentClawPose);

		if (newClawPoseTarget != null)
		{
			clawMotors.setTarget(newClawPoseTarget); // Issue claw motor commands.
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
	 *       [ SELECT, UP,   NONE, UP      ] = RobotMode.ASCENT
	 *       [ SELECT, UP,   NONE, LEFT    ] = RobotMode.BUCKET
	 *       [ SELECT, UP,   NONE, RIGHT   ] = RobotMode.SPECIMEN
	 *       [ SELECT, DOWN, NONE, RIGHT   ] = RobotMode.INTAKE
	 *       [ SELECT, DOWN, NONE, DOWN    ] = RobotMode.COMPACT
	 *       [ SELECT, UP,   NONE, CANCEL  ] = Cancel (example)
	 *       [ SELECT, CANCEL              ] = Cancel (example)
	 */
	static private RobotMode updateModeSelectSequence(
		@NonNull Gamepad gamepad,
		List<RobotModeSelectCommand> selectSequence,
		RobotMode oldRobotMode)
	{
		if (gamepad.x && not already selecting)
		{
			// SELECT
			selectSequence.clear();
			selectSequence.add(RobotModeSelectCommand.SELECT);
		}
		else if (gamepad.a)
		{
			// Slam into TRANSPORT mode.
			selectSequence.clear();
			return RobotMode.TRANSPORT;
		}
		else if (gamepad.b)
		{
			// CANCEL
			selectSequence.clear();
			return oldRobotMode;
		}

		final ModeCommand dpadCommand =
			gamepad.dpad_up ? RobotModeSelectCommand.UP :
			gamepad.dpad_down ? RobotModeSelectCommand.DOWN :
			gamepad.dpad_left ? RobotModeSelectCommand.LEFT :
			gamepad.dpad_right ? RobotModeSelectCommand.RIGHT :
				RobotModeSelectCommand.NONE;

		if (selectSequence.length() > 1 && selectSequence.last() != dpadCommand)
			selectSequence.add(dpadCommand);

		// Only add to the select sequence if it has been started (not empty).
		if (!selectSequence.isEmpty())
		{
			// Check if we now hold complete a select sequence.
			if (selectSequence.size() == 4)
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
