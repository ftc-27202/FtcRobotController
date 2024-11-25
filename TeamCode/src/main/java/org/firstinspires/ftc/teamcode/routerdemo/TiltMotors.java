package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
		double wristTwistPosition;

		public Pose(
				@NonNull int tiltPosition,
				@NonNull int slidePositionLeft,
				@NonNull int slidePositionRight,
				@NonNull double armPivotPositionLeft,
				@NonNull double armPivotPositionRight,
				@NonNull double wristPivotPosition,
				@NonNull double wristTwistPosition)
		{
			this.tiltPosition = tiltPosition;
			this.slidePositionLeft = slidePositionLeft;
			this.slidePositionRight = slidePositionRight;
			this.armPivotPositionLeft = armPivotPositionLeft;
			this.armPivotPositionRight = armPivotPositionRight;
			this.wristPivotPosition = wristPivotPosition;
			this.wristTwistPosition = wristTwistPosition;
		}

		boolean isClose(int lhs, int rhs, int maxDelta)
		{
			return Math.abs(lhs - rhs) <= maxDelta;
		}

		boolean isClose(double lhs, double rhs, double maxDelta)
		{
			return Math.abs(lhs - rhs) <= maxDelta;
		}

		boolean isCloseTo(Pose rhs)
		{
			static final int maxTiltDelta = 50;
			static final int maxSlideDelta = 50;
			static final double maxArmPivotDelta = 0.1;
			static final double maxWristPivotDelta = 0.1;
			static final double maxWristTwistDelta = 0.1;

			return isClose(tiltPosition, rhs.tiltPosition, maxTiltDelta) &&
				isClose(slidePositionLeft, rhs.slidePositionLeft, maxSlideDelta) &&
				isClose(slidePositionRight, rhs.slidePositionRight, maxSlideDelta) &&
				isClose(armPivotPositionLeft, rhs.armPivotPositionLeft, maxArmPivotDelta) &&
				isClose(armPivotPositionRight, rhs.armPivotPositionRight, maxArmPivotDelta) &&
				isClose(wristPivotPosition, rhs.wristPivotPosition, maxWristPivotDelta) &&
				isClose(wristTwistPosition, rhs.wristTwistPosition, maxWristTwistDelta);
		}
	}

	private DcMotor tiltMotor = null;
	private DcMotor slideMotorLeft = null;
	private DcMotor slideMotorRight = null;

	private Servo armPivotServoLeft = null;
	private Servo armPivotServoRight = null;
	private Servo wristPivotServo = null;
	private Servo wristTwistServo = null;

	public void init()
	{
		driveMotorFrontLeft = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_FL");
		driveMotorBackLeft = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_BL");
		driveMotorFrontRight = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_FR");
		driveMotorBackRight = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_BR");

		tiltMotor = hardwareMap.get(DcMotor.class, "TILT_MOTOR");
		slideMotorLeft = hardwareMap.get(DcMotor.class, "SLIDE_MOTOR_L");
		slideMotorRight = hardwareMap.get(DcMotor.class, "SLIDE_MOTOR_R");

		armPivotServoLeft = hardwareMap.get(Servo.class, "ARM_PIVOT_SERVO_L");
		armPivotServoRight = hardwareMap.get(Servo.class, "ARM_PIVOT_SERVO_R");
		wristPivotServo = hardwareMap.get(Servo.class, "WRIST_PIVOT_SERVO");
		wristTwistServo = hardwareMap.get(Servo.class, "WRIST_TWIST_SERVO");
	}

	public void setTarget(@NonNull Pose target)
	{
		tiltMotor.setTargetPosition(target.tiltPosition);
		slideMotorLeft.setTargetPosition(target.slidePositionLeft);
		slideMotorRight.setTargetPosition(target.slidePositionRight);

		armPivotServoLeft.setPosition(target.armPivotPositionLeft);
		armPivotServoRight.setPosition(target.armPivotPositionRight);
		wristPivotServo.setPosition(target.wristPivotPosition);
		wristTwistServo.setPosition(target.wristTwistPosition);
	}

	public Pose getCurrentPose()
	{
		return new Pose(
				tiltMotor.getCurrentPosition(),
				slideMotorLeft.getCurrentPosition(),
				slideMotorRight.getCurrentPosition(),
				armPivotServoLeft.getPosition(),
				armPivotServoRight.getPosition(),
				wristPivotServo.getPosition(),
				wristTwistServo.getPosition()
		);
	}
}
