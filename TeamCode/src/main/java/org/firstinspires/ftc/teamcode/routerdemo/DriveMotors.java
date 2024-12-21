package org.firstinspires.ftc.teamcode.routerdemo;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
				double frontLeft,
				double frontRight,
				double backLeft,
				double backRight)
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

	public void init(HardwareMap hardwareMap)
	{
		frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
		frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
		backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
		backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");
	}

	public void setPowerLevels(@NonNull PowerLevels levels)
	{
		frontLeftMotor.setPower(levels.frontLeft);
		frontRightMotor.setPower(levels.frontRight);
		backLeftMotor.setPower(levels.backLeft);
		backRightMotor.setPower(levels.backRight);
	}
}
