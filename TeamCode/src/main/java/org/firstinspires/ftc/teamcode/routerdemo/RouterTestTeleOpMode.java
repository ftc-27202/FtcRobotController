package org.firstinspires.ftc.teamcode.routerdemo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RouterTestTeleOpMode extends OpMode
{
	private final RobotMotors motors = new RobotMotors();

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	@Override
	public void init()
	{
		motors.init();

		// Initialize the routers to reflect the robot's starting state.
		tiltRouter.init(TiltRouter.Waypoint.COMPACT);
		clawRouter.init(ClawRouter.Waypoint.OPEN);
	}

	@Override
	public void loop()
	{
		//
		// Handle driver inputs. Don't issue motor commands -- just set targets.
		//

		if (gamepad2.dpad_down)
		{
			tiltRouter.setTarget(TiltRouter.Waypoint.DRIVE_CONFIG);
		}
		else if (gamepad2.dpad_up)
		{
			tiltRouter.setTarget(TiltRouter.Waypoint.COMPACT);
		}

		if (gamepad2.dpad_left)
		{
			clawRouter.setTarget(ClawRouter.Waypoint.GRASP);
		}
		else if (gamepad2.dpad_right)
		{
			// Retreat after dropping into basket, pretending that it needs to be closed to fit between towers.
			clawRouter.setTarget(ClawRouter.Waypoint.CLOSED);
			tiltRouter.setTarget(TiltRouter.Waypoint.DRIVE_CONFIG);
		}

		//
		// Update the routers based on current state and capture updated targets.
		//

		final double currentClawPosition = motors.getClawEncoderPosition();
		final ClawRouter.Waypoint newClawWaypoint = clawRouter.updateProgress(currentClawPosition);

		if (newClawWaypoint != null)
		{
			final double newEncoderTarget = RobotGeometry.convertToEncoderPosition(newClawWaypoint);
			motors.setClawTarget(newEncoderTarget);
		}

		// Update the tilt tracking based on the current encoder values. A new waypoint will be
		// returned (one time) if the motors require new instructions.
		final RobotMotors.TiltEncoderPositions currentTiltPositions = motors.getTiltEncoderPositions();
		final TiltRouter.Waypoint newTiltWaypoint = tiltRouter.updateProgress(currentTiltPositions);

		if (newTiltWaypoint != null)
		{
			final RobotMotors.TiltEncoderPositions newEncoderTargets = RobotGeometry.convertToEncoderPositions(newTiltWaypoint);
			motors.setTiltEncoderTargets(newEncoderTargets);
		}
	}
}