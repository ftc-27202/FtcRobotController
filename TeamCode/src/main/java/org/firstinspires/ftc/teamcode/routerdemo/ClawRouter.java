package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

public class ClawRouter
{
	public enum Preset
	{
		CENTERED_CLOSED, // x
		CENTERED_OPEN,   // x
		CENTERED_GRASP,  // x
		ORIENTED_CLOSED, // x
		ORIENTED_OPEN,   // x
		ORIENTED_GRASP   // x
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

	// Update the Action motion progress against the measured encoder values. If the next waypoint
	// has been reached then instruct the motors to advance to next waypoint.
	public ClawServos.Pose updateProgress(ClawServos.Pose currentPose)
	{
		return currentPose; // incomplete
	}

	// there needs to be something different than Preset to report "holding sample"
	public boolean holdsSample()
	{
		return false;
	}
}
