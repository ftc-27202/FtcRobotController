package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Base Two Driver TeleOp", group = "Robot")
@Disabled
public class BaseTeleOpMode extends OpMode {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor slideTiltLeft;
    public DcMotor slideTiltRight;

    public DcMotor slideMotorLeft;
    public DcMotor slideMotorRight;

    final int SLIDES_UP = 3300;
    final int SLIDES_DOWN = 0;

    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");


        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");

        slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotor.Direction.FORWARD);

        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideTiltLeft = hardwareMap.get(DcMotor.class, "slideTiltLeft");
        slideTiltLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTiltLeft.setDirection(DcMotor.Direction.REVERSE);

        slideTiltRight = hardwareMap.get(DcMotor.class, "slideTiltRight");
        slideTiltRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTiltRight.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveBase(double axial, double lateral, double yaw, double straightSpeed, double strafeSpeed, double turnSpeed) {
        axial *= straightSpeed;
        lateral *= strafeSpeed;
        yaw *= turnSpeed;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double max;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void loop() {
        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("Slide Tilt Left", "%d", slideTiltLeft.getCurrentPosition());
        telemetry.addData("Slide Tilt Right", "%d", slideTiltRight.getCurrentPosition());
        telemetry.addData("Run Time", getRuntime());
        telemetry.update();
    }
}
