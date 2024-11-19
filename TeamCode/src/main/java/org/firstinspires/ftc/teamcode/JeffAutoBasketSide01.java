package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@Config
@Autonomous(name = "AA_Auto01 (Basket Side)", group = "Autonomous")
public class JeffAutoBasketSide01 extends LinearOpMode {
    public class Slide {
        private DcMotorEx leftSlide;
        private DcMotorEx rightSlide;

        final int SLIDE_GROUND = 0;
        final int SLIDE_HALF = 1350;
        final int SLIDE_HIGH = 2650;
        final double SLIDE_STALL_TIME = 2.0;

        public Slide(HardwareMap hardwareMap) {
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);
            leftSlide.setTargetPosition(SLIDE_GROUND);
            rightSlide.setTargetPosition(SLIDE_GROUND);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class SlidesUpHigh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(0.8);
                    rightSlide.setPower(0.8);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("rightSlidePos", posRightSlide);
                if (posRightSlide < SLIDE_HIGH) {
                    leftSlide.setTargetPosition(SLIDE_HIGH);
                    rightSlide.setTargetPosition(SLIDE_HIGH);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesUpHigh() {
            return new SlidesUpHigh();
        }

        public class SlidesDownGround implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(0.8);
                    rightSlide.setPower(0.8);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("rightSlidePos", posRightSlide);
                if (posRightSlide > (SLIDE_GROUND + 100)) {
                    leftSlide.setTargetPosition(SLIDE_GROUND);
                    rightSlide.setTargetPosition(SLIDE_GROUND);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownGround() {
            return new SlidesDownGround();
        }

        public class SlidesDownHalf implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(0.8);
                    rightSlide.setPower(0.8);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("rightSlidePos", posRightSlide);
                if (posRightSlide > SLIDE_HALF) {
                    leftSlide.setTargetPosition(SLIDE_HALF);
                    rightSlide.setTargetPosition(SLIDE_HALF);
                    return true;
                } else {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action SlidesDownHalf() {
            return new SlidesDownHalf();
        }

        public class SlidesUpHalf implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(0.8);
                    rightSlide.setPower(0.8);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("rightSlidePos", posRightSlide);
                if (posRightSlide < SLIDE_HIGH) {
                    leftSlide.setTargetPosition(SLIDE_HALF);
                    rightSlide.setTargetPosition(SLIDE_HALF);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesUpHalf() {
            return new SlidesUpHalf();
        }
    }

    public class Arm {
        private DcMotorEx armMotor;

        final double ARM_TICKS_PER_DEGREE =
                28 // number of encoder ticks per rotation of the bare motor
                        * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                        * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                        * 1 / 360.0; // Ticks per degree, not per rotation
        final double ARM_COLLAPSED_INTO_ROBOT = 0;
        final double ARM_COLLECT = 225 * ARM_TICKS_PER_DEGREE;
        final double ARM_CLEAR_BARRIER = 215 * ARM_TICKS_PER_DEGREE;
        final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
        final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
        final double ARM_LEVEL_ONE_ASCENT = 125 * ARM_TICKS_PER_DEGREE;
//        final double ARM_DEPOSIT = 74 * ARM_TICKS_PER_DEGREE;
        final double ARM_DEPOSIT = 80 * ARM_TICKS_PER_DEGREE;
        final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) armMotor).setVelocity(2100);
        }

        public class ArmCollect implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_COLLECT) {
                    armMotor.setTargetPosition((int) ARM_COLLECT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmCollect() {
            return new ArmCollect();
        }

        public class ArmDeposit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos > ARM_DEPOSIT) {
                    armMotor.setTargetPosition((int) ARM_DEPOSIT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmDeposit() {
            return new ArmDeposit();
        }

        public class ArmCollapsedIntoRobot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos > ARM_COLLAPSED_INTO_ROBOT) {
                    armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmCollapsedIntoRobot() {
            return new ArmCollapsedIntoRobot();
        }

        public class ArmScoreSampleInLow implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_SCORE_SAMPLE_IN_LOW) {
                    armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_LOW);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmScoreSampleInLow() {
            return new ArmScoreSampleInLow();
        }
    }

    public class Bucket {
        private Servo bucket;

        final double BUCKET_CATCH = 0.8;
        final double BUCKET_DUMP = 0.3;

        public Bucket(HardwareMap hardwareMap) {
            bucket = hardwareMap.get(Servo.class, "bucket");
        }

        public class BucketDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(BUCKET_DUMP);
                packet.put("BucketPos", bucket.getPosition());
                sleep(500);
                return false;
            }
        }

        public Action BucketDump() {
            return new BucketDump();
        }

        public class BucketCatch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(BUCKET_CATCH);
                packet.put("BucketPos", bucket.getPosition());
                return false;
            }
        }

        public Action BucketCatch() {
            return new BucketCatch();
        }
    }

    public class Wrist {
        private Servo wrist;

        /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
        final double WRIST_FOLDED_IN = 0.0;
        final double WRIST_SPECIMEN = 0.4;
        final double WRIST_FOLDED_OUT = 1.0;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        public class WristOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_FOLDED_OUT);
                return false;
            }
        }

        public Action WristOut() {
            return new WristOut();
        }

        public class WristIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_FOLDED_IN);
                return false;
            }
        }

        public Action WristIn() {
            return new WristIn();
        }
    }

    public class Intake {
        private CRServo intake;

        /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
        final double INTAKE_COLLECT = -1.0;
        final double INTAKE_OFF = 0.0;
        final double INTAKE_DEPOSIT = 1;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
        }

        public class IntakeCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(INTAKE_COLLECT);
                return false;
            }
        }

        public Action IntakeCollect() {
            return new IntakeCollect();
        }

        public class IntakeDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(INTAKE_DEPOSIT);
                return false;
            }
        }

        public Action IntakeDeposit() {
            return new IntakeDeposit();
        }

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(INTAKE_OFF);
                sleep(500);
                return false;
            }
        }

        public Action IntakeOff() {
            return new IntakeOff();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Slide slide = new Slide(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToHighBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-43, -60));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition1 = trajDriveToHighBasket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-29, -27), Math.toRadians(160));

        TrajectoryActionBuilder trajDriveForwardToCollectSample1 = trajDriveToCollectSamplePosition1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33, -23), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder trajDriveToHighBasket2 = trajDriveForwardToCollectSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-45, -60), Math.toRadians(0));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition2 = trajDriveToHighBasket2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 2))
                .strafeToConstantHeading(new Vector2d(-55, 2))
                .strafeToConstantHeading(new Vector2d(-55, -54))
                .strafeToConstantHeading(new Vector2d(-55, 2))
                .strafeToConstantHeading(new Vector2d(-60, 2))
                .strafeToConstantHeading(new Vector2d(-60, -54))
                .strafeToConstantHeading(new Vector2d(48, -60));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }

        Action actDriveToHighBasket = trajDriveToHighBasket.build();
        Action actDriveToCollectSamplePosition1 = trajDriveToCollectSamplePosition1.build();
        Action actDriveForwardToCollectSample1 = trajDriveForwardToCollectSample1.build();
        Action actDriveToHighBasket2 = trajDriveToHighBasket2.build();
        Action actDriveToCollectSamplePosition2 = trajDriveToCollectSamplePosition2.build();

        waitForStart();
        this.resetRuntime();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                            bucket.BucketCatch(),
                            slide.SlidesUpHigh(),
                            actDriveToHighBasket
                        ),
                        bucket.BucketDump(),
                        new ParallelAction(
                            slide.SlidesDownHalf(),
                            actDriveToCollectSamplePosition1,
                            wrist.WristOut(),
                            intake.IntakeCollect(),
                            arm.ArmScoreSampleInLow(),
                            new SequentialAction(
                                arm.ArmCollect(),
                                slide.SlidesDownGround()
                            )
                        ),
                        actDriveForwardToCollectSample1,
                        new ParallelAction(
                            bucket.BucketCatch(),
                            wrist.WristIn(),
                            arm.ArmDeposit()
                        ),
                        new ParallelAction(
                            intake.IntakeDeposit(),
                            new SequentialAction(
                                arm.ArmScoreSampleInLow(),
                                intake.IntakeOff(),
                                slide.SlidesUpHigh()
                            ),
                            actDriveToHighBasket2
                        ),
                        bucket.BucketDump(),
                        new ParallelAction(
                            bucket.BucketCatch(),
                            new SequentialAction(
                                arm.ArmCollapsedIntoRobot(),
                                slide.SlidesDownGround()
                            ),
                            actDriveToCollectSamplePosition2
                        )
                )
        );

        telemetry.addData("Duration", this.getRuntime());
        telemetry.update();
    }
}

