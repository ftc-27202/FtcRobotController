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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Jeff Base Two Driver TeleOp", group = "Robot")
@Disabled
public class JeffBaseTeleOpMode extends OpMode {
    public DcMotor leftSlide;
    public DcMotor rightSlide;
    public Limelight3A limelight;
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor armMotor;
    public CRServo intake;
    public Servo wrist;
    public Servo bucket;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
//    final double ARM_COLLECT = 225 * ARM_TICKS_PER_DEGREE;
    final double ARM_COLLECT = 224 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 200 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
//    final double ARM_DEPOSIT = 74 * ARM_TICKS_PER_DEGREE;
    final double ARM_DEPOSIT = 78 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    double LLSPEED = .4;
    final int SLIDE_GROUND = 0;
    final int SLIDE_HALF = 1350;
    final int SLIDE_HIGH = 2650;
    final double SLIDE_STALL_TIME = 2.0;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.0;
//    final double WRIST_SPECIMEN = 0.4;
    final double WRIST_SPECIMEN = 0.3;
//    final double WRIST_FOLDED_OUT = 1.0;
    final double WRIST_FOLDED_OUT = 0.69;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    final double BUCKET_CATCH = 0.45;
    final double BUCKET_DUMP = 0;

//    final double BUCKET_CATCH = 0.80;
//    final double BUCKET_DUMP = 0.3;
    // Target tolerance for limelight, if target is within this value from centre of LL's cam, robot will not move
    final double LLTargetTolerance = 3;
    /* Variables that are used to set the arm to a specific position */

    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    int slideError = 0;
    double armPositionFudgeFactor;
   int slideTargetPosition;
    double lastSlideActionTime = getRuntime();
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(WRIST_FOLDED_IN);

        bucket = hardwareMap.get(Servo.class, "bucket");
        bucket.setPosition(BUCKET_CATCH);
    }

    @Override
    public void loop() {
        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotor).isOverCurrent()) {
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }

        /* send telemetry to the driver of the arm's current position and target position */

        telemetry.addData("Left Linear Slide: ", "%d", leftSlide.getCurrentPosition());
        telemetry.addData("Right Linear Slide: ", "%d", rightSlide.getCurrentPosition());
        telemetry.addData("arm Target: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.addData("intake: ", intake.getPower());
        telemetry.addData("wrist: ", wrist.getPosition());
        telemetry.addData("bucket: ", bucket.getPosition());
        telemetry.addData("Run Time", getRuntime());
        telemetry.update();
    }
}