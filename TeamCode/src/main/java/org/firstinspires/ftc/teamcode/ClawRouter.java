package org.firstinspires.ftc.teamcode;

public class ClawRouter
{
	public enum Waypoint
	{
		OPEN, CLOSED, GRASP
	}

	private Waypoint target;

	public Waypoint getTarget()
	{
		return target;
	}

	public void setTarget(Waypoint target)
	{
		this.target = target;
	}

	public void init(Waypoint target)
	{
		this.target = target;
	}

	// Update the Action motion progress against the measured encoder values. If the next waypoint
	// has been reached then instruct the motors to advance to next waypoint.
	public Waypoint updateProgress(/*ServoState servoState*/)
	{
		return target; // incomplete
	}

	// there needs to be something different than Waypoint to report "holding sample"
	public boolean holdsSample()
	{
		return false;
	}
}
