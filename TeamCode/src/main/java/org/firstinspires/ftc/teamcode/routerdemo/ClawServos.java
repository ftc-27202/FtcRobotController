package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.routerdemo.RobotGeometry.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.routerdemo.RobotGeometry.CLAW_OPEN;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos
{
	public static class Pose
	{
		double twistPosition;

		public Pose(double twistPosition)
		{
			this.twistPosition = twistPosition;
		}
	}

	private Servo twistServo = null;
	private Servo graspServo = null;

	public void init()
	{
		twistServo = hardwareMap.get(Servo.class, "CLAW_TWIST_SERVO");
		graspServo = hardwareMap.get(Servo.class, "CLAW_GRASP_SERVO");
	}

	public void setTarget(@NonNull Pose target)
	{
		twistServo.setPosition(target.twistPosition);
	}

	public Pose getCurrentPose()
	{
		return new Pose(twistServo.getPosition());
	}

	public void close()
	{
		graspServo.setPosition(CLAW_CLOSED);
	}

	public void open()
	{
		graspServo.setPosition(CLAW_OPEN);
	}
}
