package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RouterTestTeleOpMode extends OpMode
{
	protected DcMotor leftFrontDrive = null;
	protected DcMotor leftBackDrive = null;
	protected DcMotor rightFrontDrive = null;
	protected DcMotor rightBackDrive = null;

	private final TiltRouter tiltRouter = new TiltRouter();
	private final ClawRouter clawRouter = new ClawRouter();

	@Override
	public void init()
	{
		// Initialize the routers to reflect the robot's starting state.
		tiltRouter.init(TiltRouter.Waypoint.COMPACT);
		clawRouter.init(ClawRouter.Waypoint.OPEN);

		leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
		leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
		rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
	}

	@Override
	public void loop()
	{
		////////////////////////////////////////////////////////////
		// Handle gamepad inputs
		////////////////////////////////////////////////////////////

		// Don't issue motor commands. Instead just set tilt and claw router targets here.
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

		////////////////////////////////////////////////////////////
		// do more stuff
		////////////////////////////////////////////////////////////

		// Update the claw tracking based on the current servo values.
		final ClawRouter.Waypoint updatedClaw = clawRouter.updateProgress(/*getServoState()*/);
		if (updatedClaw != null)
		{
			//clawServo.setTarget(updatedClaw.value);
		}

		// Update the tilt tracking based on the current encoder values. The routers might hold new
		// waypoints after updating. updateProgress() only returns a waypoint once.
		final TiltRouter.Waypoint updatedTilt = tiltRouter.updateProgress(/*getCurrentPose()*/);
		if (updatedTilt != null)
		{
			final TiltRouter.Pose newPose = RobotGeometry.evaluate(updatedTilt);
			//final XXX motorInputs = RobotGeometry.evaluateMotorInputs(newPose);
			// should this be converted to motor inputs before .setTarget()?

/*
			tiltMotor.setTarget(newPose.slidePivotAngle);
			slideMotorLeft.setTarget(newPose.slideExtendPosition);
			slideMotorRight.setTarget(newPose.slideExtendPosition);
			armPivotServo.setTarget(newPose.armPivotAngle);
			wristPivotServo.setTarget(newPose.wristPivotAngle);
			wristTwistServo.setTarget(newPose.wristTwistAngle;
 */
		}
	}
}

/*
public class BaseAutoOpMode extends LinearOpMode {
    private TiltRouter tilt = new TiltRouter();
    private ClawRouter claw = new ClawRouter();

    // ... same as above
}
*/