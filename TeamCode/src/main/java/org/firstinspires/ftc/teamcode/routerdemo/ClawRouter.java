package org.firstinspires.ftc.teamcode.routerdemo;

public class ClawRouter
{
	public enum Preset
	{
		OPEN, CLOSED, GRASP
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
	public Preset updateProgress(@NonNull double clawEncoderPosition)
	{
		return target; // incomplete
	}

	// there needs to be something different than Preset to report "holding sample"
	public boolean holdsSample()
	{
		return false;
	}
}
