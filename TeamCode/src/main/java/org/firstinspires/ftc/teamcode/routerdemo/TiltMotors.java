package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TiltMotors
{
	// Structure to hold all tilt-related motor positions in one place.
	public static class Pose
	{
		int tiltPosition;
		int slidePositionLeft;
		int slidePositionRight;
		double armPivotPositionLeft;
		double armPivotPositionRight;
		double wristPivotPosition;

		public Pose(
				int tiltPosition,
				int slidePositionLeft,
				int slidePositionRight,
				double armPivotPositionLeft,
				double armPivotPositionRight,
				double wristPivotPosition)
		{
			this.tiltPosition = tiltPosition;
			this.slidePositionLeft = slidePositionLeft;
			this.slidePositionRight = slidePositionRight;
			this.armPivotPositionLeft = armPivotPositionLeft;
			this.armPivotPositionRight = armPivotPositionRight;
			this.wristPivotPosition = wristPivotPosition;
		}
	}

	private DcMotor tiltMotor = null;
	private DcMotor slideMotorLeft = null;
	private DcMotor slideMotorRight = null;

	private Servo armPivotServoLeft = null;
	private Servo armPivotServoRight = null;
	private Servo wristPivotServo = null;

	public void init(HardwareMap hardwareMap)
	{
		tiltMotor = hardwareMap.get(DcMotor.class, "slideTilt");
/*
		slideMotorLeft = hardwareMap.get(DcMotor.class, "SLIDE_MOTOR_L");
		slideMotorRight = hardwareMap.get(DcMotor.class, "SLIDE_MOTOR_R");

		armPivotServoLeft = hardwareMap.get(Servo.class, "ARM_PIVOT_SERVO_L");
		armPivotServoRight = hardwareMap.get(Servo.class, "ARM_PIVOT_SERVO_R");
		wristPivotServo = hardwareMap.get(Servo.class, "WRIST_PIVOT_SERVO");

		slideMotorLeft.setDirection(DcMotor.Direction.FORWARD);
		slideMotorRight.setDirection(DcMotor.Direction.REVERSE);
*/
		tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		tiltMotor.setTargetPosition(0);
		tiltMotor.setPower(0.5);
		tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
/*
		slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorLeft.setTargetPosition(0);
		slideMotorLeft.setPower(0.5);
		slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slideMotorRight.setTargetPosition(0);
		slideMotorRight.setPower(0.5);
		slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void setTarget(@NonNull Pose target, Telemetry telemetry)
	{
		telemetry.addData("tilt motor command", "%d", target.tiltPosition);
		tiltMotor.setTargetPosition(target.tiltPosition);
		//slideMotorLeft.setTargetPosition(target.slidePositionLeft);
		//slideMotorRight.setTargetPosition(target.slidePositionRight);

		//armPivotServoLeft.setPosition(target.armPivotPositionLeft);
		//armPivotServoRight.setPosition(target.armPivotPositionRight);
		//wristPivotServo.setPosition(target.wristPivotPosition);
	}

	public Pose getCurrentPose()
	{
		return new Pose(
				tiltMotor.getCurrentPosition(),
				0, // slideMotorLeft.getCurrentPosition(),
				0, // slideMotorRight.getCurrentPosition(),
				0.0, // armPivotServoLeft.getPosition(),
				0.0, // armPivotServoRight.getPosition(),
				0.0); // wristPivotServo.getPosition());
	}

	private static boolean areClose(int lhs, int rhs, int maxDelta)
	{
		return Math.abs(lhs - rhs) <= maxDelta;
	}

	private static boolean areClose(double lhs, double rhs, double maxDelta)
	{
		return Math.abs(lhs - rhs) <= maxDelta;
	}

	public static boolean areClose(@NonNull Pose lhs, @NonNull Pose rhs)
	{
		final int maxTiltDelta = 50;
		final int maxSlideDelta = 50;
		final double maxArmPivotDelta = 0.1;
		final double maxWristPivotDelta = 0.1;

		return areClose(lhs.tiltPosition, rhs.tiltPosition, maxTiltDelta); // &&
/*
				areClose(lhs.slidePositionLeft, rhs.slidePositionLeft, maxSlideDelta) &&
				areClose(lhs.slidePositionRight, rhs.slidePositionRight, maxSlideDelta) &&
				areClose(lhs.armPivotPositionLeft, rhs.armPivotPositionLeft, maxArmPivotDelta) &&
				areClose(lhs.armPivotPositionRight, rhs.armPivotPositionRight, maxArmPivotDelta) &&
				areClose(lhs.wristPivotPosition, rhs.wristPivotPosition, maxWristPivotDelta);
*/
	}
}
