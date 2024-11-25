package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawServo
{
	public static class Pose
	{
		double graspPosition;

		public Pose(double graspPosition)
		{
			this.graspPosition = graspPosition;
		}
	}

	private Servo graspServo = null;

	public void init()
	{
		graspServo = hardwareMap.get(Servo.class, "CLAW_GRASP_SERVO");
	}

	public void setTarget(@NonNull Pose target)
	{
		graspServo.setPosition(target.graspPosition);
	}

	public Pose getCurrentPose()
	{
		return new Pose(graspServo.getPosition());
	}
}
