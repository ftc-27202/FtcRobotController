package org.firstinspires.ftc.teamcode;

import android.telephony.euicc.DownloadableSubscription;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
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
    public Servo armPivotLeft;
    public Servo armPivotRight;
    public Servo wristPivot;
    public Servo clawPivot;
    public double slideTiltTarget = 0;
    public final double SLIDETILTPOWER = .4;
    public DcMotor slideMotorLeft;
    public DcMotor slideMotorRight;

    final int SLIDES_UP = 3300;
    final int SLIDES_DOWN = 0;

    final int SLIDE_TILT_UP = 30;
    final int SLIDE_TILT_DOWN = 100;

    final double ARM_TICKS_PER_DEGREE =
            28
                * 5800 / 312
                * 100 / 30
                * 1/360;


    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideTiltLeft = hardwareMap.get(DcMotor.class, "slideTiltLeft");
        slideTiltLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTiltLeft.setDirection(DcMotor.Direction.FORWARD);
        slideTiltLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideTiltRight = hardwareMap.get(DcMotor.class, "slideTiltRight");
        slideTiltRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideTiltRight.setDirection(DcMotor.Direction.REVERSE);
        slideTiltRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideTiltLeft.setTargetPosition(SLIDE_TILT_UP);
        slideTiltRight.setTargetPosition(SLIDE_TILT_UP);
        slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideMotorRight");

        slideMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotor.Direction.FORWARD);

        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPivotLeft = hardwareMap.get(Servo.class, "armPivotLeft");
        armPivotLeft.setDirection(Servo.Direction.FORWARD);

        armPivotRight = hardwareMap.get(Servo.class, "armPivotRight");
        armPivotLeft.setDirection(Servo.Direction.REVERSE);

        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        wristPivot = hardwareMap.get(Servo.class, "wristPivot");
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
        telemetry.addData("Slide Tilt Left Degrees", "%f",  (slideTiltLeft.getCurrentPosition() / ARM_TICKS_PER_DEGREE));
        telemetry.addData("Slide Tilt Right Degrees", "%f", (slideTiltRight.getCurrentPosition() / ARM_TICKS_PER_DEGREE));
        telemetry.addData("Arm Pivot Left", "%f", armPivotLeft.getPosition());
        telemetry.addData("Arm Pivot Right", "%f", armPivotRight.getPosition());
        telemetry.addData("Run Time", getRuntime());
        telemetry.update();
    }
}
