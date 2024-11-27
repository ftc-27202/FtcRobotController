package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos
{
	public static class Pose
	{
		double twistPosition;

		public Pose(double twistPosition, double graspPosition)
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
		return new Pose(twistServo.getPostion());
	}

   public void close()
   {
      graspServo.setPosition(SERVO_GRASP_CLOSED);
   }

   public void open()
   {
      graspServo.setPosition(SERVO_GRASP_OPEN);
   }
}
