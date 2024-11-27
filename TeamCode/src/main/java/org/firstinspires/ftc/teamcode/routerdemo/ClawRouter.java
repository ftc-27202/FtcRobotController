package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

public class ClawRouter
{
	public enum Preset
	{
		CENTERED, // x
		ORIENTED, // x
		MANUAL    // x
	}

	private Preset target;

	public Preset getTarget()
	{
		return target;
	}

	public void setTarget(@NonNull Preset target)
	{
		this.target = target;
	}

	public void init(@NonNull Preset target)
	{
		this.target = target;
	}

	public ClawRouter.Preset resting()
	{
		return target;
	}

	// Update the Action motion progress against the measured encoder values. If the next waypoint
	// has been reached then instruct the motors to advance to next waypoint.
	public ClawServos.Pose updateProgress(ClawServos.Pose currentPose)
	{
		return currentPose; // incomplete
	}
}
