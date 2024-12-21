package org.firstinspires.ftc.teamcode.routerdemo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/*
 * The tilt router is responsible for safely moving the claw between named poses.
 */
public class TiltRouter
{
	public enum NamedPose
	{
		ASCENT_HIGH_HOVER, // Hook positioned over high ascent bar.
		ASCENT_HIGH_HANG,  // Hook hanging on high specimen bar.
		ASCENT_LOW_HOVER,  // Hook positioned over low ascent bar.
		ASCENT_LOW_HANG,   // Hook hanging on low specimen bar.
		BASKET_LOW,        // Claw positioned over low basket.
		BASKET_HIGH,       // Claw positioned over high basket.
		COMPACT,           // Fits inside an 18" cube.
		INTAKE_FLOOR,      // Claw lowered to pick up a sample off the ground.
		INTAKE_HOVER,      // Hover over sample for a photo op.
		SAFE_PASS_THROUGH, // Intermediate waypoint that won't collide with slides.
		SPECIMEN_LOW,      // Claw positioned over low specimen bar.
		SPECIMEN_HIGH,     // Claw positioned over high specimen bar.
		TRANSPORT          // Close to sample picking config but stable for driving.
	}

	// Pose that the robot is either currently resting at, or will rest at after completing the current route.
	private NamedPose restingNamedPose;

	// Sequence of poses that the robot is currently traversing on its way to restingNamedPose. If the tilt mechanism
	// is at rest then this list will be empty. Otherwise the last item in namedPoseRoute will match restingNamedPose.
	private List<NamedPose> namedPoseRoute = new ArrayList<>();

	private long lastTargetChangeTimeMillis;

	public void init(NamedPose initialNamedPose)
	{
		restingNamedPose = initialNamedPose;
	}

	public TiltRouter.NamedPose resting()
	{
		return namedPoseRoute.isEmpty() ? restingNamedPose : null;
	}

	public void setTarget(NamedPose target)
	{
		if (namedPoseRoute.isEmpty()) // Robot tilt mechanism is currently at rest.
		{
			// Set new namedPoseRoute so the next call to updateProgress() will command the motors to start.
			namedPoseRoute = findRoute(restingNamedPose, target);
		}
		else // Robot tilt mechanism is moving.
		{
			if (target == restingNamedPose)
				return; // Nothing to do: Action was already moving toward this restingNamedPose.

			// Switch to the new restingNamedPose by replacing the existing namedPoseRoute, but allow the current leg
			// to finish before starting toward the new restingNamedPose. Routes ensure safe motion between known
			// namedPoseRoute but changing the path between arbitrary namedPoseRoute could cause a collision.
			final NamedPose nextNamedPose = namedPoseRoute.get(0);
			namedPoseRoute.clear(); // Abandon old unreached namedPoseRoute.
			namedPoseRoute = findRoute(nextNamedPose, target); // Re-route to new restingNamedPose, keeping next pose.
		}

		restingNamedPose = target;
		lastTargetChangeTimeMillis = System.currentTimeMillis();
	}

	// Update the tile route progress using the measured encoder values. If the current waypoint
	// has been reached then instruct the motors to advance to next one.
	public TiltMotors.Pose updateProgress(TiltMotors.Pose currentPose, Telemetry telemetry)
	{
		if (namedPoseRoute.isEmpty())
			return null; // Nothing to do: The action has reached its restingNamedPose.

		final NamedPose nextNamedPose = namedPoseRoute.get(0);
		final TiltMotors.Pose nextPose = RobotGeometry.toPose(nextNamedPose);

		telemetry.addData("tilt", "%s %d %d %s",
				nextNamedPose.toString(),
				currentPose.tiltPosition,
				nextPose.tiltPosition,
				TiltMotors.areClose(currentPose, nextPose) ? "CLOSE" : "NOT CLOSE");

		if (TiltMotors.areClose(currentPose, nextPose))
		{
			// Reached the next waypoint. Pop it off and determine what's next.
			namedPoseRoute.remove(0);

			if (namedPoseRoute.isEmpty()) // Tilt has reached its destination.
			{
				if (restingNamedPose == NamedPose.COMPACT)
					; // Turn off motor power at a resting position?

				// Should we kill power? (and where do we start it up again?)
			}
			else
			{
				return RobotGeometry.toPose(namedPoseRoute.get(0));
			}
		}
		else // Not there yet.
		{
			if (System.currentTimeMillis() - lastTargetChangeTimeMillis > 2000)
				; // kill power to motors?
		}

		return null;
	}

	private boolean inFloorZone(NamedPose namedPose)
	{
		return namedPose == NamedPose.TRANSPORT ||
			namedPose == NamedPose.INTAKE_HOVER ||
			namedPose == NamedPose.INTAKE_FLOOR;
	}

	// Build a list of poses that will safely transition the robot from startPose to endPose. The resulting
	// list will includes startPose, endPose, and any intermediate poses that are required for safe travel.
	public List<NamedPose> findRoute(NamedPose startPose, NamedPose endPose)
	{
		List<NamedPose> namedPoseRoute = new ArrayList<NamedPose>();

		namedPoseRoute.add(startPose);

		// The tilt arm can safely move between poses in the "near floor" zone, and also between poses
		// in the "vertical reach" zone (e.g., baskets and ascent), but moving between zones requires
		// it to pass between the linear slides. For these cases add an intermediate SAFE_PASS_THROUGH
		// pose that aligns the tilt arm into a safe orientation.
		final boolean startIsInFloorZone = inFloorZone(startPose);
		final boolean endIsInFloorZone = inFloorZone(endPose);

		if (startIsInFloorZone != endIsInFloorZone)
			namedPoseRoute.add(NamedPose.SAFE_PASS_THROUGH);

		namedPoseRoute.add(endPose);

		return namedPoseRoute;
	}
}
