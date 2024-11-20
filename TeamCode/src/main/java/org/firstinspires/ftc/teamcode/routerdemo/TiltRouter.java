package org.firstinspires.ftc.teamcode.routerdemo;

import java.util.ArrayList;
import java.util.List;

//
// The tilt router is responsible for safely moving the claw between preset waypoint poses.
//
public class TiltRouter
{
	public enum Waypoint
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

	public static class Pose
	{
		double tiltAngleDeg;       // Tilt angles are relative to world "up" when viewed from robot's left side.
		int slidePosition;         // Linear slide extension value used by left and right slides.
		double armPivotAngleDeg;   // Arm pivot angles are relative to slide direction when viewed from robot's left side.
		double wristPivotAngleDeg; // Angle relative to arm when viewed from robot's left side.
		double wristTwistAngleDeg; // Angle relative to center when looking straight at it.

		public Pose(
				double tiltAngleDeg,
				int slidePosition,
				double armPivotAngleDeg,
				double wristPivotAngleDeg,
				double wristTwistAngleDeg)
		{
			this.tiltAngleDeg = tiltAngleDeg;
			this.slidePosition = slidePosition;
			this.armPivotAngleDeg = armPivotAngleDeg;
			this.wristPivotAngleDeg = wristPivotAngleDeg;
			this.wristTwistAngleDeg = wristTwistAngleDeg;
		}
	}

	private Waypoint target;
	private List<Waypoint> waypoints = new ArrayList<>();

	public void init(Waypoint state)
	{
		target = state;
	}

	public void setTarget(Waypoint newTarget)
	{
		if (waypoints.isEmpty()) // Robot is currently at rest.
		{
			// Update waypoints so the next call to updateProgress() will command the motors to start.
			waypoints = findRoute(target, newTarget);
			target = newTarget;
		}
		else // Robot is currently moving toward a target.
		{
			Waypoint finalWaypoint = waypoints.get(waypoints.size() - 1);

			if (newTarget == finalWaypoint)
				return; // Nothing to do: Action was already moving toward this target.

			// Switch to the new target by replacing the existing waypoints, but finish the current
			// waypoint before starting toward the new target. Routes only ensure safe movement between
			// known waypoints so changing the path between waypoints could cause a collision.
			final Waypoint nextWaypoint = waypoints.get(0);
			waypoints.clear(); // Abandon old unreached waypoints.
			waypoints = findRoute(nextWaypoint, newTarget); // Re-route to new target, but finish current waypoint first.
			target = newTarget;
		}
	}

	// Update the tile router progress using the measured encoder values. If the current waypoint
	// has been reached then instruct the motors to advance to next one.
	public Waypoint updateProgress(RobotMotors.TiltEncoderPositions currentEncoderPositions)
	{
		if (waypoints.isEmpty())
			return null; // Nothing to do: The action has reached its target.

		final Pose currentPose = RobotGeometry.convertToPose(currentEncoderPositions);
		final Waypoint nextWaypoint = waypoints.get(0);

		if (RobotGeometry.atWaypoint(nextWaypoint, currentPose))
		{
			// Reached the next waypoint. Pop it off and determine what's next.
			waypoints.remove(0);

			if (waypoints.isEmpty())
			{
				; // Action has reached its destination. Turn off motors at a resting position?
			}
			else
			{
				return waypoints.get(0);
			}
		}
/*
		else // Not there yet.
		{
			if (lastTargetCommandTime - currentTime() > 1500) {}
				kill power?
		}
*/

		return null;
	}

	// Build a list of waypoints that will safely transform the robot from startWaypoint to endWaypoint. The
	// resulting list includes startWaypoint, endWaypoint, and any intermediate poses required for safe travel.
	public List<Waypoint> findRoute(Waypoint startWaypoint, Waypoint endWaypoint)
	{
		ArrayList<Waypoint> route = new ArrayList<Waypoint>();
		route.add(startWaypoint);

		switch (startWaypoint)
		{
			case BASKET_LOW:
			case BASKET_HIGH:
				route.add(Waypoint.SAFE_PASS_THROUGH);
				route.add(endWaypoint);
				break;

			case SPECIMEN_HIGH:
				// not implemented
				break;

			case SPECIMEN_LOW:
				// not implemented
				break;

			case DRIVE_CONFIG:
				route.add(endWaypoint); // DRIVE_CONFIG can go directly to any other state
				break;

			case PICK_HOVER:
				break;

			case PICK:
				if (endWaypoint == Waypoint.DRIVE_CONFIG)
				{
					route.add(endWaypoint);
				}
				else
				{
					route.add(Waypoint.DRIVE_CONFIG);
					route.add(endWaypoint);
				}
				break;
		}

		return route;
	}
}
