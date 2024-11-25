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
		//
		// Set drive motors based on gamepad1 inputs.
		//

		final DriveMotors.PowerLevels levels = RobotGeometry.calculateDrivePower(gamepad1);
		driveMotors.setPowerLevels(levels);

		//
		// Set router targets based on gamepad2 inputs. Will be converted into motor commands later.
		//

		if (gamepad2.dpad_up)
		{
			tiltRouter.setTarget(TiltRouter.Preset.COMPACT);
		}
		else if (gamepad2.dpad_down)
		{
			tiltRouter.setTarget(TiltRouter.Preset.DRIVE_CONFIG);

			final double clawAngle = autoOrient.hasValue() ? autoOrient.getValue() : 0.0;
				
			tiltRouter.setTarget(TiltRouter.Preset.PICK);
			clawRouter.setTarget(ClawRouter.Preset.OPEN);
		}
		else if (gamepad2.dpad_left)
		{
			if (targeting PHOTO_CENTERED)
			{
				backing out of PHOTO_CENTERED				
				// stop photo stream
			}


			// Retreat after dropping into basket, pretending that it needs to be closed to fit between towers.
			clawRouter.setTarget(ClawRouter.Preset.CLOSED);
			tiltRouter.setTarget(TiltRouter.Preset.DRIVE_CONFIG);
		}
		else if (gamepad2.dpad_right)
		{
			// right dpad only advances to next target if at rest
			if (restingTilt != null && restingClaw != null)
			{
				switch (restingTilt)
				{
					case PHOTO_DRIVE_CONFIG: // -> advances to CENTERED_PHOTO_HOVER
						TiltRouter.setTarget(TiltRouter.Preset.PHOTO_HOVER_CENTERED);
						// start photo stream
						break;

					case CENTERED_PHOTO_HOVER: // -> advances to ORIENTED_FLOOR_PICK
						if (photo stream is open)
						{
							double orientation = stream.getOrientation();

							if (successful)
							{
								// will route through ORIENTED_PHOTO_HOVER first
								TiltRouter.setTarget(TiltRouter.Preset.ORIENTED_FLOOR_PICK, orientation);
							}

							close photo stream;
						}
						break;

					case ORIENTED_FLOOR_PICK: // -> advances to DRIVE_CONFIG

						TiltRouter.setTarget(TiltRouter.Preset.DRIVE_CONFIG);
						break;
				}
			}
		}

		//
		// Update the tilt based on current values and handle new targets.
		//

		final TiltMotors.Pose currentTiltPose = tiltMotors.getCurrentPose();
		final TiltMotors.Pose newTiltPoseTarget = tiltRouter.updateProgress(currentTiltPose);

		if (newTiltPoseTarget != null)
		{
			tiltMotors.setTarget(newTiltPoseTarget);
		}

		//
		// Check for automatic claw actions.
		//

		final ClawServo.Pose currentClawPose = clawServo.getCurrentPose();
		final ClawServo.Pose newClawPoseTarget = clawRouter.updateProgress(currentClawPose);

		if (newClawPoseTarget != null)
		{
			clawServo.setTarget(newClawPoseTarget);
		}

		if (restingTilt ?= TiltRouter.Preset.ORIENTED_FLOOR_PICK && restingClaw ?= ClawRouter.Preset.OPEN)
		{
			// Tilt and claw are ready to pick up a sample.
			clawRouter.setTarget(ClawRouter.Preset.GRASP);
		}
	}
}

maybe wristtwist and claw should be together ...
