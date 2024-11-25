package org.firstinspires.ftc.teamcode.routerdemo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RouterTestTeleOpMode extends OpMode
{
	private final DriveMotors driveMotors = new DriveMotors();
	private final TiltMotors tiltMotors = new TiltMotors();
	private final ClawServo clawServo = new ClawServo();

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	@Override
	public void init()
	{
		// Initialize motors using robot configuration.
		driveMotors.init();
		tiltMotors.init();
		clawServo.init();

		// Initialize the routers to match the robot's starting state.
		tiltRouter.init(TiltRouter.Preset.COMPACT);
		clawRouter.init(ClawRouter.Preset.OPEN);
	}

	@Override
	public void loop()
	{
		final DriveMotors.PowerLevels drivePowerLevels = RobotGeometry.calculateDrivePower(gamepad1);

		driveMotors.setPowerLevels(drivePowerLevels);

		//
		// Set tilt and claw targets based on inputs. Allow the routers to convert targets into motor commands later.
		//

		if (gamepad2.dpad_down)
		{
			tiltRouter.setTarget(TiltRouter.Preset.DRIVE_CONFIG);
		}
		else if (gamepad2.dpad_up)
		{
			tiltRouter.setTarget(TiltRouter.Preset.COMPACT);
		}

		if (gamepad2.dpad_left)
		{
			clawRouter.setTarget(ClawRouter.Preset.GRASP);
		}
		else if (gamepad2.dpad_right)
		{
			// Retreat after dropping into basket, pretending that it needs to be closed to fit between towers.
			clawRouter.setTarget(ClawRouter.Preset.CLOSED);
			tiltRouter.setTarget(TiltRouter.Preset.DRIVE_CONFIG);
		}

		//
		// Update the routers based on current encoder states and capture any target updates.
		//

		final ClawServo.Pose currentClawPose = clawServo.getCurrentPose();
		final ClawServo.Pose newClawPoseTarget = clawRouter.updateProgress(currentClawPose);

		if (newClawPoseTarget != null)
		{
			clawServo.setTarget(newClawPoseTarget);
		}

		// Update the tilt router based on the current encoder values. A new waypoint will be
		// returned (once) if the motors require new instructions.
		final TiltMotors.Pose currentTiltPose = tiltMotors.getCurrentPose();
		final TiltMotors.Pose newTiltPoseTarget = tiltRouter.updateProgress(currentTiltPose);

		if (newTiltPoseTarget != null)
		{
			tiltMotors.setTarget(newTiltPoseTarget);
		}
	}
}
