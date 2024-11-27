package org.firstinspires.ftc.teamcode.routerdemo;

import java.util.ArrayList;
import java.util.List;

//
// The tilt router is responsible for safely moving the claw between preset waypoint poses.
//
public class TiltRouter
{
	public enum Preset
	{
		COMPACT,           // Fits inside an 18" cube.
		DRIVE_CONFIG,      // Close to sample picking config but stable for driving.
		SAFE_PASS_THROUGH, // Intermediate waypoint that won't collide with slides.
		PICK_HOVER,        // Hover over sample for a photo op.
		PICK,              // Claw lowered to pick up a sample off the ground.
		BASKET_LOW,        // Claw positioned over low basket.
		BASKET_HIGH,       // Claw positioned over high basket.
		SPECIMEN_LOW,      // Claw positioned over low specimen bar.
		SPECIMEN_HIGH,     // Claw positioned over high specimen bar.
		ASCENT_LOW_HOVER,  // Hook positioned over low ascent bar.
		ASCENT_LOW_HANG,   // Hook hanging on low specimen bar.
		ASCENT_HIGH_HOVER, // Hook positioned over high ascent bar.
		ASCENT_HIGH_HANG   // Hook hanging on high specimen bar.
	}

	private Preset restingPreset;
	private List<Preset> routePresets = new ArrayList<>();

	private long lastTargetChangeTimeMillis;

	public void init(Preset initialPreset)
	{
		restingPreset = initialPreset;
	}

	public void setTarget(Preset target)
	{
		if (routePresets.isEmpty()) // Robot tilt mechanism is currently at rest.
		{
			// Set new routePresets so the next call to updateProgress() will command the motors to start.
			routePresets = findRoute(restingPreset, target);
		}
		else // Robot tilt mechanism is moving.
		{
			if (target == restingPreset)
				return; // Nothing to do: Action was already moving toward this restingPreset.

			// Switch to the new restingPreset by replacing the existing routePresets, but allow the current leg
			// to finish before starting toward the new restingPreset. Routes ensure safe motion between known
			// routePresets but changing the path between arbitrary routePresets could cause a collision.
			final Preset nextPreset = routePresets.get(0);
			routePresets.clear(); // Abandon old unreached routePresets.
			routePresets = findRoute(nextPreset, target); // Re-route to new restingPreset, keeping next preset.
		}

		restingPreset = target;
		lastTargetChangeTimeMillis = System.currentTimeMillis();
	}

	// Update the tile route progress using the measured encoder values. If the current waypoint
	// has been reached then instruct the motors to advance to next one.
	public TiltMotors.Pose updateProgress(TiltMotors.Pose currentPose)
	{
		if (routePresets.isEmpty())
			return null; // Nothing to do: The action has reached its restingPreset.

		final Preset nextPreset = routePresets.get(0);
		final TiltMotors.Pose nextPose = RobotGeometry.toPose(nextPreset);

		if (TiltMotors.areClose(currentPose, nextPose))
		{
			// Reached the next waypoint. Pop it off and determine what's next.
			routePresets.remove(0);

			if (routePresets.isEmpty()) // Tilt has reached its destination.
			{
				if (restingPreset == Preset.COMPACT)
					; // Turn off motor power at a resting position?

				// Should we kill power? (and where do we start it up again?)
			}
			else
			{
				return RobotGeometry.toPose(routePresets.get(0));
			}
		}
		else // Not there yet.
		{
			if (System.currentTimeMillis() - lastTargetChangeTimeMillis > 2000)
				; // kill power to motors?
		}

		return null;
	}

	private isFloorZone(Preset preset)
	{
		return preset == (DRIVE_CONFIG || preset == PICK_HOVER || preset == PICK_FLOOR);
	}

	// Build a list of routePresets that will safely transition the robot from startPreset to
	// restingPreset. The resulting list includes startPreset, restingPreset, and any intermediate
	// poses required for safe travel.
	public List<Preset> findRoute(Preset startPreset, Preset endPreset)
	{
		List<Preset> routePresets = new ArrayList<Preset>();

		routePresets.add(startPreset);

		// The tilt arm can safely move between poses in the "near floor" zone, and also between poses
		// in the "vertical reach" zone (e.g., baskets and ascent), but moving between zones requires
		// it to pass between the linear slides. For these cases add an intermediate SAFE_PASS_THROUGH
		// pose that aligns the tilt arm into a safe orientation.
		final bool startIsInFloorZone = inFloorZone(startPreset);
		final bool endIsInFloorZone = inFloorZone(endPreset);

		if (startIsInFloorZone != endIsInFloorZone)
			routePresets.add(Preset.SAFE_PASS_THROUGH);

		routePresets.add(endPreset);

		return routePresets;
	}
}
