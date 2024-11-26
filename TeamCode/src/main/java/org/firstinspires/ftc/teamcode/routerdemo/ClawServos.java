package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos
{
	public static class Pose
	{
		double twistPosition;
		double graspPosition;

		public Pose(double twistPosition, double graspPosition)
		{
			this.twistPosition = twistPosition;
			this.graspPosition = graspPosition;
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
		graspServo.setPosition(target.graspPosition);
	}

	public Pose getCurrentPose()
	{
		return new Pose(twistServo.getPostion(), graspServo.getPosition());
	}
}
