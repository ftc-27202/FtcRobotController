package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;

//
// The claw router is responsible for tracking orienting and closing the claw.
//
public class ClawRouter
{
	public enum Preset
	{
		CENTERED, // Claw is centered at 0 degrees.
		ORIENTED, // Claw has been automatically oriented using the vision pipeline.
		MANUAL    // Claw has been manually oriented using the gamepad controls.
	}

	private Preset restingPreset;

	public Preset resting()
	{
		return restingPreset;
	}

	public void setRestingPreset(@NonNull Preset restingPreset)
	{
		this.restingPreset = restingPreset;
	}

	public void init(@NonNull Preset target)
	{
		this.restingPreset = target;
	}

	// Update the Action motion progress against the measured encoder values. If the next waypoint
	// has been reached then instruct the motors to advance to next waypoint.
	public ClawServos.Pose updateProgress(ClawServos.Pose currentPose)
	{
		return currentPose; // incomplete
	}
}
