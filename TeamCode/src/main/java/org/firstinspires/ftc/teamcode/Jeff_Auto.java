 /* Copyright (c) 2023 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.roadrunner.Pose2d;
 import com.acmerobotics.roadrunner.Trajectory;
 import com.acmerobotics.roadrunner.Vector2d;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.Servo;

 import org.firstinspires.ftc.teamcode.MecanumDrive;

 /*
  * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
  * The code assumes a Holonomic (Mecanum or X Drive) Robot.
  *
  * For an introduction to AprilTags, see the ftc-docs link below:
  * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
  *
  * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
  * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
  * https://ftc-docs.firstinspires.org/apriltag-detection-values
  *
  * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
  * driving towards the tag to achieve the desired distance.
  * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
  * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
  *
  * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
  * The motor directions must be set so a positive power goes forward on all wheels.
  * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
  * so you should choose to approach a valid tag ID.
  *
  * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
  * Manually drive the robot until it displays Target data on the Driver Station.
  *
  * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
  * Release the Left Bumper to return to manual driving mode.
  *
  * Under "Drive To Target" mode, the robot has three goals:
  * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
  * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
  * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
  *
  * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
  * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
  *
  * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
  */

 @Autonomous(name = "AA_Auto 1: Basket Side", group = "drive")

 public class Jeff_Auto extends LinearOpMode {
     public DcMotor leftSlide;
     public DcMotor rightSlide;
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
     final double ARM_COLLECT = 225 * ARM_TICKS_PER_DEGREE;
     final double ARM_CLEAR_BARRIER = 215 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
     final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
     final double ARM_LEVEL_ONE_ASCENT = 125 * ARM_TICKS_PER_DEGREE;
     final double ARM_DEPOSIT = 74 * ARM_TICKS_PER_DEGREE;
     final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;

     final int SLIDE_GROUND = 0;
     final int SLIDE_HALF = 1350;
     final int SLIDE_HIGH = 2650;
     final double SLIDE_STALL_TIME = 2.0;

     /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
     final double INTAKE_COLLECT = -1.0;
     final double INTAKE_OFF = 0.0;
     //     final double INTAKE_DEPOSIT = 0.5;
     final double INTAKE_DEPOSIT = 1;

     /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
     final double WRIST_FOLDED_IN = 0.0;
     final double WRIST_SPECIMEN = 0.4;
     final double WRIST_FOLDED_OUT = 1.0;

     final double BUCKET_CATCH = 0.8;
     final double BUCKET_DUMP = 0.3;

     /* Variables that are used to set the arm to a specific position */
     int slideTargetPosition;
     double armMotorTargetPosition;
     double wristTargetPosition;

     private boolean SequenceComplete = false;

     @Override
     public void runOpMode() {
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
         armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
         armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         ((DcMotorEx) armMotor).setVelocity(2100);

         intake = hardwareMap.get(CRServo.class, "intake");
         intake.setPower(INTAKE_OFF);

         wrist = hardwareMap.get(Servo.class, "wrist");
         wrist.setPosition(WRIST_FOLDED_IN);

         bucket = hardwareMap.get(Servo.class, "bucket");
         leftSlide.setPower(0.8);
         rightSlide.setPower(0.8);

         slideTargetPosition = SLIDE_GROUND;
         leftSlide.setTargetPosition(slideTargetPosition);
         rightSlide.setTargetPosition(slideTargetPosition);
         leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         // Roadrunner initialization
         MecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         // Starting Pose (x, y, and heading)
         // read: https://learnroadrunner.com/trajectories.html#coordinate-system
         Pose2d startPose = new Pose2d(-41, -60, 0);
         drive.setPoseEstimate(startPose);

         Trajectory traj1 = drive.trajectoryBuilder(startPose)
                 .back(6)
                 .build();

//         Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                 .lineToLinearHeading(new Pose2d(-54, 6, 0))
//                 .build();
         Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                 .lineToLinearHeading(new Pose2d(-54, 0, 0))
                 .build();

//         Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                 .strafeRight(10)
//                 .build();

         Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-48, -8))
                 .build();

         //        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .lineToLinearHeading(new Pose2d(-30,0,180 + 1e-6))
//                .build();

         Trajectory traj5 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(180))), false)
                 .strafeTo(new Vector2d(-38,-18))
                 .build();

         Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                 .forward(10)
                 .build();

         Trajectory traj7 = drive.trajectoryBuilder(traj6.end().plus(new Pose2d(0, 0, Math.toRadians(180))), false)
                 .strafeTo(new Vector2d(-41,-62))
                 .build();

         Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                 .strafeLeft(6)
                 .build();

         Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                 .forward(84)
                 .build();

         // Wait for the game to start (driver presses START)
         waitForStart();

         bucket.setPosition(BUCKET_CATCH);

         if (isStopRequested()) return;
         while (!isStopRequested() && !SequenceComplete) {
             drive.followTrajectory(traj1);

             // Raise slider to High Basket
             slideTargetPosition = SLIDE_HIGH;
             leftSlide.setTargetPosition(slideTargetPosition);
             rightSlide.setTargetPosition(slideTargetPosition);
             while (leftSlide.getCurrentPosition() < slideTargetPosition) {
                 sleep(10);
             };
/*
             // Bucket -> Dump
             bucket.setPosition(BUCKET_DUMP);
             while (bucket.getPosition() > BUCKET_DUMP) {
                 sleep(10);
             };

             // wait for the sample to fall
             sleep(500);

             // Bucket -> Catch
             bucket.setPosition(BUCKET_CATCH);

             // Move the slider to Half
             slideTargetPosition = SLIDE_HALF;
             leftSlide.setTargetPosition(slideTargetPosition);
             rightSlide.setTargetPosition(slideTargetPosition);

             // Navigate to next position
             drive.followTrajectory(traj2);
             drive.followTrajectory(traj3);
             drive.turn(Math.toRadians(185) - 1e-6);
             drive.followTrajectory(traj5);

             wristTargetPosition = WRIST_FOLDED_OUT;
             wrist.setPosition(wristTargetPosition);
             intake.setPower(INTAKE_COLLECT);

             armMotorTargetPosition = ARM_COLLECT;
             armMotor.setTargetPosition((int) armMotorTargetPosition);
             while (armMotor.getCurrentPosition() < armMotorTargetPosition) {
                 sleep(10);
             }

             // Move the slider to Ground for Sample
             slideTargetPosition = SLIDE_GROUND;
             leftSlide.setTargetPosition(slideTargetPosition);
             rightSlide.setTargetPosition(slideTargetPosition);

             drive.followTrajectory(traj6);

             // Bucket -> Catch
             bucket.setPosition(BUCKET_CATCH);

             wristTargetPosition = WRIST_FOLDED_IN;
             wrist.setPosition(wristTargetPosition);

             armMotorTargetPosition = ARM_DEPOSIT;
             armMotor.setTargetPosition((int) armMotorTargetPosition);
             while (armMotor.getCurrentPosition() > armMotorTargetPosition) {
                 sleep(10);
             }

             sleep(750);
             intake.setPower(INTAKE_DEPOSIT);
             sleep(500);

             armMotorTargetPosition = ARM_SCORE_SAMPLE_IN_LOW;
             armMotor.setTargetPosition((int) armMotorTargetPosition);

             drive.turn(Math.toRadians(187.5) - 1e-6);
             drive.followTrajectory(traj7);
             drive.turn(Math.toRadians(10) - 1e-6);

             intake.setPower(INTAKE_OFF);

             // Raise slider to High Basket
             slideTargetPosition = SLIDE_HIGH;
             leftSlide.setTargetPosition(slideTargetPosition);
             rightSlide.setTargetPosition(slideTargetPosition);

             while (leftSlide.getCurrentPosition() < SLIDE_HALF) {
                 sleep(10);
             };

             armMotorTargetPosition = ARM_COLLAPSED_INTO_ROBOT;
             armMotor.setTargetPosition((int) armMotorTargetPosition);

             while (leftSlide.getCurrentPosition() < slideTargetPosition) {
                 sleep(10);
             };

             // Bucket -> Dump
             bucket.setPosition(BUCKET_DUMP);
             while (bucket.getPosition() > BUCKET_DUMP) {
                 sleep(10);
             };

             sleep(500);

             slideTargetPosition = SLIDE_GROUND;
             leftSlide.setTargetPosition(slideTargetPosition);
             rightSlide.setTargetPosition(slideTargetPosition);
             while (leftSlide.getCurrentPosition() > SLIDE_HALF) {
                 sleep(10);
             };

             bucket.setPosition(BUCKET_CATCH);

             drive.followTrajectory(traj8);
             drive.followTrajectory(traj9);
*/
             SequenceComplete = true;
         }
     }
 }