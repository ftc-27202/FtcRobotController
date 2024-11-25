package org.firstinspires.ftc.teamcode.routerdemo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveMotors
{
	// Structure to hold motor power levels for all 4 drive motors.
	public static class PowerLevels
	{
		double frontLeft;
		double frontRight;
		double backLeft;
		double backRight;

		public PowerLevels(
				@NonNull double frontLeft,
				@NonNull double frontRight,
				@NonNull double backLeft,
				@NonNull double backRight)
		{
			this.frontLeft = frontLeft;
			this.frontRight = frontRight;
			this.backLeft = backLeft;
			this.backRight = backRight;
		}
	}

	// Declare member variables for all motors and servos.
	public DcMotor frontLeftMotor = null;
	public DcMotor frontRightMotor = null;
	public DcMotor backLeftMotor = null;
	public DcMotor backRightMotor = null;

	public void init()
	{
		frontLeftMotor = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_FL");
		frontRightMotor = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_FR");
		backLeftMotor = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_BL");
		backRightMotor = hardwareMap.get(DcMotor.class, "DRIVE_MOTOR_BR");
	}

	public void setPowerLevels(@NonNull PowerLevels levels)
	{
		frontLeftMotor.setPower(levels.frontLeftMotor);
		frontRightMotor.setPower(levels.frontRightMotor);
		backLeftMotor.setPower(levels.backLeftMotor);
		backRightMotor.setPower(levels.backRightMotor);
	}
}
