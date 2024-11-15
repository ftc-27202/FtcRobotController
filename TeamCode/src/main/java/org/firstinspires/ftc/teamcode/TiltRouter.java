package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/*
 * The tilt router is responsible for safely moving the claw between preset poses, or waypoints.
 * The tilt mechanism consists of several components, including:
 *
 *   Tilt motor           tiltMotor         "TILT_MOTOR"
 *   Left linear slide    slideMotorLeft    "MOTOR_LEFT"
 *   Right linear slide   slideMotorRight   "MOTOR_RIGHT"
 *   Arm pivot servo      armPivotServo     "ARM_PIVOT_SERVO"
 *   Wrist pivot servo    wristPivotServo   "WRIST_PIVOT_SERVO"
 *   Wrist twist servo    wristTwistServo   "WRIST_TWISE_SERVO"
 */
public class TiltRouter
{
	public enum Waypoint
	{
		COMPACT,           // Fits inside an 18" cube.
		DRIVE_CONFIG,      // Close to sample picking config but stable for driving.
		SAFE_PASS_THROUGH, // Intermediate waypoint that won't collide with slides.
		PICK_HOVER,        // Hover over sample for a photo opportunity.
		PICK,              // x
		BASKET_LOW,        // x
		BASKET_HIGH,       // x
		SPECIMEN_LOW,      // x
		SPECIMEN_HIGH,     // x
		ASCENT_LOW_HOVER,  // x
		ASCENT_LOW_HANG,   // x
		ASCENT_HIGH_HOVER, // x
		ASCENT_HIGH_HANG   // x
	}

	public static class Pose
	{
		double slidePosition;   // Linear slide extension value used by left and right slides.
		double tiltAngle;       // Tilt angles are relative to world "up" when viewed from robot's left side.
		double armPivotAngle;   // Arm pivot angles are relative to slide direction when viewed from robot's left side.
		double wristPivotAngle; // Angle relative to arm when viewed from robot's left side.
		double wristTwistAngle; // Angle relative to center when looking straight at it.

		Pose(double slidePosition, double tiltAngle, double armPivotAngle, double wristPivotAngle, double wristTwistAngle)
		{
			this.slidePosition = slidePosition;
			this.tiltAngle = tiltAngle;
			this.armPivotAngle = armPivotAngle;
			this.wristPivotAngle = wristPivotAngle;
			this.wristTwistAngle = wristTwistAngle;
		}
	}

	private Waypoint target;
	private List<Waypoint> waypoints = new ArrayList<>();

	public void init(Waypoint state)
	{
		target = state;
	}

	public Waypoint getTarget()
	{
		return target;
	}

	public void setTarget(Waypoint newTarget)
	{
		if (waypoints.isEmpty())
		{
			// Robot is currently at rest. Add new waypoints so the next call to updateProgress() will
			// command the motors to start.
			final Waypoint currentTarget = target;
			waypoints = findRoute(currentTarget, newTarget);
		}
		else
		{
			// Robot is currently moving toward a target.
			Waypoint finalWaypoint = waypoints.get(waypoints.size() - 1);
			if (newTarget == finalWaypoint)
				return; // Nothing to do: Action was already moving toward this target.

			// Switch to the new target by replacing the existing waypoints, but finish the current
			// waypoint before starting toward the new target. Routes only ensure safe movement between
			// known waypoints so changing the path between waypoints could cause a collision.
			final Waypoint nextWaypoint = waypoints.get(0);
			waypoints.clear(); // Abandon old unreached waypoints.
			waypoints = findRoute(nextWaypoint, newTarget); // Re-route to new target, but finish current waypoint first.
		}
	}

	// Update the Action motion progress against the measured encoder values. If the next waypoint
	// has been reached then instruct the motors to advance to next waypoint.
	public Waypoint updateProgress(/*Pose measuredPose*/)
	{
		if (waypoints.isEmpty()) return null; // Nothing to do: The action has reached its target.

		final Waypoint nextWaypoint = waypoints.get(0);
		if (true) // AreClose(measuredPose, nextWaypoint))
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