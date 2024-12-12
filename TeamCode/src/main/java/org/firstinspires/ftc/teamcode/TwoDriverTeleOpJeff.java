/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Jeff Two Driver TeleOp", group = "Robot")

//@Disabled
public class TwoDriverTeleOpJeff extends JeffBaseTeleOpMode {

    public void startStrafing(int power) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);
    }

    public void loop () {
        //if left_trigger: speed = 0.6; else speed = 1.0
        double speed = gamepad1.right_trigger > 0 ? 0.6 : 1.0;
        double turn_speed = gamepad1.right_trigger > 0 ?  0.2 : 1.0;
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x * turn_speed;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * speed);
        rightFrontDrive.setPower(rightFrontPower * speed);
        leftBackDrive.setPower(leftBackPower * speed);
        rightBackDrive.setPower(rightBackPower * speed);

        if (gamepad1.a) {
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad1.x) {
            intake.setPower(INTAKE_OFF);
        } else if (gamepad1.b) {
            intake.setPower(INTAKE_DEPOSIT);
        }

        if (gamepad1.right_bumper) {
            armPosition = ARM_COLLECT;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad1.left_bumper) {
            wrist.setPosition(WRIST_FOLDED_OUT);
            armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad1.y) {
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        } else if (gamepad1.dpad_left) {
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        } else if (gamepad1.dpad_right) {
            armPosition = ARM_SCORE_SPECIMEN;
            wrist.setPosition(WRIST_SPECIMEN);
        } else if (gamepad1.dpad_up) {
            armPosition = ARM_DEPOSIT;
            wrist.setPosition(WRIST_FOLDED_IN);
        } else if (gamepad1.dpad_down) {
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.left_trigger);

        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (gamepad2.dpad_up) {
            slideTargetPosition = SLIDE_HIGH;
            lastSlideActionTime = getRuntime();
        } else if (gamepad2.dpad_down) {
            slideTargetPosition = SLIDE_GROUND;
            lastSlideActionTime = getRuntime();
        } else if (gamepad2.dpad_left) {
            slideTargetPosition = SLIDE_HALF;
            lastSlideActionTime = getRuntime();
        }

        if (gamepad2.left_trigger > 0) {
            bucket.setPosition(BUCKET_CATCH);
        } else if (gamepad2.right_trigger > 0) {
            bucket.setPosition(BUCKET_DUMP);
        }

        if (gamepad2.y) {
            limelight.pipelineSwitch(0);
        } else if (gamepad2.b) {
            limelight.pipelineSwitch(1);
        } else if (gamepad2.x){
            limelight.pipelineSwitch(2);
        }

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
           slideTargetPosition -= (int) (gamepad2.right_stick_y * 10.0);
           slideError -= (int) (gamepad2.right_stick_y * 10.0);
        }

        //Limelight stuff starts here
        LLResult result = limelight.getLatestResult();
        telemetry. addData("Pipeline:", result.getPipelineIndex());
        //gamepad2.dpad_right will target the robot to a seen sample (rn only yellow works)
        // if gamepad2.dpad_right is pressed and target is seen, identify direction and try to move towards the target until target is within tolerance
        if (gamepad2.dpad_right && (result != null && result.isValid()) ) {
            //gets results from LL
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            //move left or right based on data
            //Target tolerance is how many units the centre of the target has to be offset for the robot to decide to move
            // Right now it is arbitrarily set at 5, but can be changed, IDK what it should actually be
            //Set as constant in jeff base teleop
            //right is positive I think?
            if (tx >= LLTargetTolerance ){
                telemetry.addData("Move Right", tx);
                leftFrontDrive.setPower(LLSPEED);
                rightFrontDrive.setPower(-LLSPEED);
                leftBackDrive.setPower(-LLSPEED);
                rightBackDrive.setPower(LLSPEED);
            }else if((-1*LLTargetTolerance) >= tx){
                telemetry.addData("Move Left", tx);
                // Fill in with code to actually strafe the robot slowly to the left
                leftFrontDrive.setPower(-LLSPEED);
                rightFrontDrive.setPower(LLSPEED);
                leftBackDrive.setPower(LLSPEED);
                rightBackDrive.setPower(-LLSPEED);
            }else if((Math.abs(tx)) < LLTargetTolerance ){
                telemetry.addData("Target within tolerance, current offset:", tx);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }

        } else if (result != null && result.isValid()) {
            // if LL detects target but right dpad not pressed, just display target in telemetry
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        }  else if (gamepad2.dpad_right) {
            telemetry.addLine("No Targets Found"); //If gamepad is pressed but no result, telemetry says no targets
        } else {
            telemetry.addData("Limelight", "No Targets or LL not working"); //Also no targets even if gamepad not pressed
        }
        //slides not in position
        if (getRuntime() >= lastSlideActionTime + SLIDE_STALL_TIME) {
           final double leftSlideRemaining = Math.abs(leftSlide.getTargetPosition() - leftSlide.getCurrentPosition());
           final double rightSlideRemaining = Math.abs(rightSlide.getTargetPosition() - rightSlide.getCurrentPosition());

           if (leftSlideRemaining > 200 || rightSlideRemaining > 200) {
               leftSlide.setPower(0.0);
               rightSlide.setPower(0.0);
               telemetry.addLine("SLIDE(S) STUCK!");
               return;
           }
        }

        //prevents extensions being 42 inches or more
        if (armMotor.getTargetPosition() > ARM_SCORE_SPECIMEN && (leftSlide.getTargetPosition() > SLIDE_HALF || rightSlide.getTargetPosition() > SLIDE_HALF)) {
            slideTargetPosition = SLIDE_HALF;
        }

        leftSlide.setTargetPosition(slideTargetPosition - slideError);
        leftSlide.setPower(1.0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPosition(slideTargetPosition - slideError);
        rightSlide.setPower(1.0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        super.loop();
    }
}