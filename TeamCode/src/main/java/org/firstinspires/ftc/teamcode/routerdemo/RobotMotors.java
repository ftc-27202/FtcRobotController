package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMotors
{
	// Structure to hold drive motor power levels.
	public static class DriveMotorPowerLevels
	{
		double driveMotorFrontLeft;
		double driveMotorBackLeft;
		double driveMotorFrontRight;
		double driveMotorBackRight;

		public DriveMotorPowerLevels(
				double driveMotorFrontLeft,
				double driveMotorBackLeft,
				double driveMotorFrontRight,
				double driveMotorBackRight)
		{
			this.driveMotorFrontLeft = driveMotorFrontLeft;
			this.driveMotorBackLeft = driveMotorBackLeft;
			this.driveMotorFrontRight = driveMotorFrontRight;
			this.driveMotorBackRight = driveMotorBackRight;
		}
	}

	// Structure to hold all of the tilt-related motor positions.
	public static class TiltEncoderPositions
	{
		int tiltPosition;
		int slidePositionLeft;
		int slidePositionRight;
		double armPivotPositionLeft;
		double armPivotPositionRight;
		double wristPivotPosition;
		double wristTwistPosition;

		public TiltEncoderPositions(
				int tiltPosition,
				int slidePositionLeft,
				int slidePositionRight,
				double armPivotPositionLeft,
				double armPivotPositionRight,
				double wristPivotPosition,
				double wristTwistPosition)
		{
			this.tiltPosition = tiltPosition;
			this.slidePositionLeft = slidePositionLeft;
			this.slidePositionRight = slidePositionRight;
			this.armPivotPositionLeft = armPivotPositionLeft;
			this.armPivotPositionRight = armPivotPositionRight;
			this.wristPivotPosition = wristPivotPosition;
			this.wristTwistPosition = wristTwistPosition;
		}
	}

	// Declare member variables for all motors and servos.
	public DcMotor driveMotorFrontLeft = null;
	public DcMotor driveMotorBackLeft = null;
	public DcMotor driveMotorFrontRight = null;
	public DcMotor driveMotorBackRight = null;

	private DcMotor tiltMotor = null;
	private DcMotor slideMotorLeft = null;
	private DcMotor slideMotorRight = null;

	private Servo armPivotServoLeft = null;
	private Servo armPivotServoRight = null;
	private Servo wristPivotServo = null;
	private Servo wristTwistServo = null;
	private Servo clawGraspServo = null;

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
		clawGraspServo = hardwareMap.get(Servo.class, "CLAW_GRASP_SERVO");
	}

	public void setDriveMotorPowerLevels(@NonNull DriveMotorPowerLevels levels)
	{
		driveMotorFrontLeft.setPower(levels.driveMotorFrontLeft);
		driveMotorFrontRight.setPower(levels.driveMotorFrontRight);
		driveMotorBackLeft.setPower(levels.driveMotorBackLeft);
		driveMotorBackRight.setPower(levels.driveMotorBackRight);
	}

	public void setClawTarget(double position)
	{
		clawGraspServo.setPosition(position);
	}

	public double getClawEncoderPosition()
	{
		return clawGraspServo.getPosition();
	}

	public void setTiltEncoderTargets(TiltEncoderPositions positions)
	{
		tiltMotor.setTargetPosition(positions.tiltPosition);
		slideMotorLeft.setTargetPosition(positions.slidePositionLeft);
		slideMotorRight.setTargetPosition(positions.slidePositionRight);

		armPivotServoLeft.setPosition(positions.armPivotPositionLeft);
		armPivotServoRight.setPosition(positions.armPivotPositionRight);
		wristPivotServo.setPosition(positions.wristPivotPosition);
		wristTwistServo.setPosition(positions.wristTwistPosition);
	}

	public TiltEncoderPositions getTiltEncoderPositions()
	{
		return new TiltEncoderPositions(
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