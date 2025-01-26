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
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "AAA_Auto (Gripper)", group = "Autonomous")
public class Gripper_BasketSide extends LinearOpMode {

    final int SLIDE_GROUND = 0;
    final int SLIDE_CATCH = 600;
    final int SLIDE_HALF = 1350;
    final int SLIDE_HIGH = 2650;
    final double SLIDE_STALL_TIME = 2.0;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_DEPOSIT = 75 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BUCKET = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_PARALLEL_TO_GROUND = 173 * ARM_TICKS_PER_DEGREE;  // verified
    final double ARM_PREPARE_TO_COLLECT = 185 * ARM_TICKS_PER_DEGREE;  // verified
    //        final double ARM_COLLECT = 194 * ARM_TICKS_PER_DEGREE;
    final double ARM_COLLECT_OUT_TO_IN = 192 * ARM_TICKS_PER_DEGREE;
    final double BUCKET_CATCH = 0.5;
    final double BUCKET_DUMP = 0.1;

    // For physical install, 0.0 = is facing upwards (viewpoint when collecting), slightly off due to BWT link servo block hole placement issue
    final double ELBOW_DEPOSIT = 0.15;
    final double ELBOW_COLLECT = 0.85;

    // For physical install, 1.0 = Gripper out-most position
    final double GRIPPER_IN = 0.5;
    final double GRIPPER_GRABBING_OUTWARDS = 0.65;
    final double GRIPPER_GRABBING_INWARDS = 0.70;
    final double GRIPPER_OUT = 1.0;

    // For physical install, 1.0 = Flag is facing all the way down
    final double FLAG_DOWN = 1.0;
    final double FLAG_SCORE = 0.35;

    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX = 7;
    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_RED_INDEX = 8;
    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_BLUE_INDEX = 9;

    public class Slide {
        private DcMotorEx leftSlide;
        private DcMotorEx rightSlide;

        public Slide(HardwareMap hardwareMap) {
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
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
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
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
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
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
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide > (SLIDE_HALF + 100)) {
                    leftSlide.setTargetPosition(SLIDE_HALF);
                    rightSlide.setTargetPosition(SLIDE_HALF);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownHalf() {
            return new SlidesDownHalf();
        }

        public class SlidesDownCatch implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide > (SLIDE_CATCH)) {
                    leftSlide.setTargetPosition(SLIDE_CATCH);
                    rightSlide.setTargetPosition(SLIDE_CATCH);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownCatch() {
            return new SlidesDownCatch();
        }
    }

    public class Arm {
        private DcMotorEx armMotor;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) armMotor).setVelocity(2100);
        }

        public class ArmCollectOutToIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_COLLECT_OUT_TO_IN - 5) {  // 2 is the buffer to avoid delay
                    armMotor.setTargetPosition((int) ARM_COLLECT_OUT_TO_IN);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmCollectOutToIn() {
            return new ArmCollectOutToIn();
        }

        public class ArmDeposit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
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

        public class ArmClearBucket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_CLEAR_BUCKET) {
                    armMotor.setTargetPosition((int) ARM_CLEAR_BUCKET);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmClearBucket() {
            return new ArmClearBucket();
        }

        public class ArmCollapsedIntoRobot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
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

        public class ArmParallelToGround implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_PARALLEL_TO_GROUND) {
                    armMotor.setTargetPosition((int) ARM_PARALLEL_TO_GROUND);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmParallelToGround() {
            return new ArmParallelToGround();
        }

        public class ArmPrepareToCollect implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
//                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_PREPARE_TO_COLLECT) {
                    armMotor.setTargetPosition((int) ARM_PREPARE_TO_COLLECT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmPrepareToCollect() {
            return new ArmPrepareToCollect();
        }
    }

    public class Bucket {
        private Servo bucket;

        public Bucket(HardwareMap hardwareMap) {
            bucket = hardwareMap.get(Servo.class, "bucket");
        }

        public class BucketDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(BUCKET_DUMP);
//                packet.put("BucketPos", bucket.getPosition());
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
//                packet.put("BucketPos", bucket.getPosition());
                return false;
            }
        }

        public Action BucketCatch() {
            return new BucketCatch();
        }
    }

    public class Elbow {
        private Servo elbow;

        public Elbow(HardwareMap hardwareMap) {
            elbow = hardwareMap.get(Servo.class, "elbow");
        }

        public class ElbowDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(ELBOW_DEPOSIT);
                return false;
            }
        }

        public Action ElbowDeposit() {
            return new ElbowDeposit();
        }

        public class ElbowCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(ELBOW_COLLECT);
                return false;
            }
        }

        public Action ElbowCollect() {
            return new ElbowCollect();
        }
    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }

        public class GripperIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_IN);
                return false;
            }
        }

        public Action GripperIn() {
            return new GripperIn();
        }

        public class GripperGrabOutwards implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_GRABBING_OUTWARDS);
                return false;
            }
        }

        public Action GripperGrabOutwards() {
            return new GripperGrabOutwards();
        }

        public class GripperGrabInwards implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_GRABBING_INWARDS);
                return false;
            }
        }

        public Action GripperGrabInwards() {
            return new GripperGrabInwards();
        }

        public class GripperOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_OUT);
                return false;
            }
        }

        public Action GripperOut() {
            return new GripperOut();
        }
    }

    public class Flag {
        private Servo flag;

        public Flag(HardwareMap hardwareMap) {
            flag = hardwareMap.get(Servo.class, "flag");
        }

        public class FlagDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flag.setPosition(FLAG_DOWN);
                return false;
            }
        }

        public Action FlagDown() {
            return new FlagDown();
        }

        public class FlagScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flag.setPosition(FLAG_SCORE);
                return false;
            }
        }

        public Action FlagScore() {
            return new FlagScore();
        }
    }

    public class LimeLightVision {
        private Limelight3A limelight3A;

        public LimeLightVision(HardwareMap hardwareMap) {
            limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX);
        }

        public class LimeLightVisionStart implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public Action LimeLightVisionStart() {
            return new LimeLightVisionStart();
        }
    }

    public class FTCTelemetry {
        double auto_starttime;
        double lap_starttime;
        double lap1_duration;
        double lap2_duration;
        double total_duration;

        public FTCTelemetry(HardwareMap hardwareMap) {
        }

        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                auto_starttime = System.currentTimeMillis();
                lap_starttime = auto_starttime;
                return false;
            }
        }

        public Action ResetTimer() {
            return new ResetTimer();
        }

        public class UpdateSample1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lap1_duration = (System.currentTimeMillis() - lap_starttime) / 1000;
                lap_starttime = System.currentTimeMillis();
                return false;
            }
        }

        public Action UpdateSample1() {
            return new UpdateSample1();
        }

        public class UpdateSample2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lap2_duration = (System.currentTimeMillis() - lap_starttime) / 1000;
                total_duration = (System.currentTimeMillis() - auto_starttime) / 1000;
                packet.put("Sample 1 Duration", lap1_duration);
                packet.put("Sample 2 Duration", lap2_duration);
                packet.put("Total    Duration", total_duration);
                lap_starttime = System.currentTimeMillis();
                return false;
            }
        }

        public Action UpdateSample2() {
            return new UpdateSample2();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Slide slide = new Slide(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);
        Flag flag = new Flag(hardwareMap);
        LimeLightVision limelight = new LimeLightVision(hardwareMap);
        FTCTelemetry ftctelemetry = new FTCTelemetry(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToHighBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-44, -60));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition1 = trajDriveToHighBasket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51, -44), Math.toRadians(90));
//                .strafeToSplineHeading(new Vector2d(-30, -33), Math.toRadians(160));

        TrajectoryActionBuilder trajDriveToHighBasket2 = trajDriveToCollectSamplePosition1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-52, -48), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition2 = trajDriveToHighBasket2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-38, -25), Math.toRadians(180));

        TrajectoryActionBuilder trajDriveToHighBasket3 = trajDriveToCollectSamplePosition2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-54, -45), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition3 = trajDriveToHighBasket3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-47, -25), Math.toRadians(180));

        TrajectoryActionBuilder trajDriveToHighBasket4 = trajDriveToCollectSamplePosition3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-57, -48), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToPark = trajDriveToHighBasket4.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-10, -66), Math.toRadians(0))
                .strafeTo(new Vector2d(32, -66));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }

        Action actDriveToHighBasket = trajDriveToHighBasket.build();
        Action actDriveToCollectSamplePosition1 = trajDriveToCollectSamplePosition1.build();
        Action actDriveToHighBasket2 = trajDriveToHighBasket2.build();
        Action actDriveToCollectSamplePosition2 = trajDriveToCollectSamplePosition2.build();
        Action actDriveToHighBasket3 = trajDriveToHighBasket3.build();
        Action actDriveToCollectSamplePosition3 = trajDriveToCollectSamplePosition3.build();
        Action actDriveToHighBasket4 = trajDriveToHighBasket4.build();
        Action actDriveToPark = trajDriveToPark.build();

        waitForStart();

        this.resetRuntime();

        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                ftctelemetry.ResetTimer(),
                // Score preloaded sample to high basket
                new ParallelAction(
                    bucket.BucketCatch(),
                    slide.SlidesUpHigh(),
                    actDriveToHighBasket,
                    elbow.ElbowCollect()
                ),
                bucket.BucketDump(),
                new SleepAction(0.4),
                ftctelemetry.UpdateSample1(),
                // Drive to collect 1st sample from mat
                new ParallelAction(
                    arm.ArmPrepareToCollect(),
                    slide.SlidesDownCatch(),
                    actDriveToCollectSamplePosition1,
                    elbow.ElbowCollect(),
                    gripper.GripperOut(),
                    bucket.BucketCatch()
                ),
                arm.ArmCollectOutToIn(),
                gripper.GripperGrabInwards(),
                new SleepAction(0.20),
                new ParallelAction(
                    actDriveToHighBasket2,
                    new SequentialAction(
                        new ParallelAction(
                            elbow.ElbowDeposit(),
                            arm.ArmDeposit()
                        ),
                        gripper.GripperOut(),
                        new SleepAction(0.20),
                        arm.ArmClearBucket(),
                        slide.SlidesUpHigh()
                    )
                ),
                flag.FlagScore(),
                bucket.BucketDump(),
                ftctelemetry.UpdateSample2(),
                new SleepAction(0.4)

                    // Drive to collect 2nd sample from mat


                // temporary, just to hold the last position for a few seconds.
//                new SleepAction(60)


//                new ParallelAction(
//                    new SequentialAction(
//                        new ParallelAction(
//                            arm.ArmDeposit(),
//                            wrist.WristIn()
//                        ),
//                        new SequentialAction(
//                            intake.IntakeDeposit(),
//                            new SleepAction(0.5),
//                            intake.IntakeOff()
//                        ),
//                        new ParallelAction(
//                            arm.ArmClearBucket(),
//                            new SequentialAction(
//                                new SleepAction(0.15),
//                                slide.SlidesUpHigh()
//                            )
//                        )
//                    ),
//                    actDriveToHighBasket2
//                ),
//                bucket.BucketDump(),
//                new SleepAction(0.5),
//                new ParallelAction(
//                    slide.SlidesDownGround(),
//                    actDriveToCollectSamplePosition2,
//                    wrist.WristOut(),
//                    intake.IntakeCollect(),
//                    arm.ArmClearBarrier(),
//                    bucket.BucketCatch()
//                ),
//                new ParallelAction(
//                    arm.ArmCollect()
//                        ,
//                    actDriveForwardToCollectSample2
//                )
//                    ,
//                intake.IntakeOff(),
//                new ParallelAction(
//                    new SequentialAction(
//                        new ParallelAction(
//                            arm.ArmDeposit(),
//                            wrist.WristIn()
//                        ),
//                        new SequentialAction(
//                            intake.IntakeDeposit(),
//                            new SleepAction(0.5),
//                            intake.IntakeOff()
//                        ),
//                        intake.IntakeOff(),
//                        new ParallelAction(
//                            arm.ArmClearBucket(),
//                            new SequentialAction(
//                                new SleepAction(0.15),
//                                slide.SlidesUpHigh()
//                            )
//                        )
//                    ),
//                    actDriveToHighBasket3
//                ),
//                bucket.BucketDump(),
//                new SleepAction(0.5),
//                new ParallelAction(
//                    slide.SlidesDownGround(),
//                    actDriveToCollectSamplePosition3,
//                    wrist.WristOut(),
//                    intake.IntakeCollect(),
//                    arm.ArmClearBarrier(),
//                    bucket.BucketCatch()
//                ),
//                new ParallelAction(
//                    arm.ArmCollectSample3()
//                        ,
//                    actDriveForwardToCollectSample3
//                )
//                    ,
//                    new SleepAction(0.5),
//                intake.IntakeOff(),
//                new ParallelAction(
//                    new SequentialAction(
//                        new ParallelAction(
//                            arm.ArmDeposit(),
//                            wrist.WristIn()
//                        ),
//                        new SequentialAction(
//                            intake.IntakeDeposit(),
//                            new SleepAction(0.5),
//                            intake.IntakeOff()
//                        ),
//                        new ParallelAction(
//                            arm.ArmClearBucket(),
//                            new SequentialAction(
//                                new SleepAction(0.15),
//                                slide.SlidesUpHigh()
//                            )
//                        )
//                    ),
//                    actDriveToHighBasket4
//                ),
//                bucket.BucketDump(),
//                new SleepAction(0.5),
//                new ParallelAction(
//                    bucket.BucketCatch(),
//                    arm.ArmCollapsedIntoRobot(),
//                    slide.SlidesDownGround(),
//                    actDriveToPark
//                )
            )
        );

//        telemetry.addData("Where is my output", 0);
//        telemetry.addData("Duration", this.getRuntime());
//        telemetry.update();
    }
}

