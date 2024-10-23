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

    public void loop () {
        //if left_trigger: speed = 0.6; else speed = 1.0
        double speed = gamepad1.right_trigger > 0 ? 0.6 : 1.0;
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

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
            leftSlide.setTargetPosition(SLIDE_HIGH);
            leftSlide.setPower(2.0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightSlide.setTargetPosition(SLIDE_HIGH);
            rightSlide.setPower(2.0);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lastSlideActionTime = getRuntime();
        } else if (gamepad2.dpad_down) {
            leftSlide.setTargetPosition(SLIDE_GROUND);
            leftSlide.setPower(2.0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightSlide.setTargetPosition(SLIDE_GROUND);
            rightSlide.setPower(2.0);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lastSlideActionTime = getRuntime();
        } else if (gamepad2.dpad_left) {
            leftSlide.setTargetPosition(SLIDE_HALF);
            leftSlide.setPower(2.0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightSlide.setTargetPosition(SLIDE_HALF);
            rightSlide.setPower(2.0);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lastSlideActionTime = getRuntime();
        }

        if (gamepad2.left_trigger > 0) {
            bucket.setPosition(BUCKET_CATCH);
        } else if (gamepad2.right_trigger > 0) {
            bucket.setPosition(BUCKET_DUMP);
        }

        if (gamepad2.x) {
            wrist.setPosition(WRIST_FOLDED_OUT);
        } else if (gamepad2.y) {
            wrist.setPosition(WRIST_FOLDED_IN);
        }

        //prevent overheating
        if (
                //it's been SLIDE_STALL_TIME
                getRuntime() >= lastSlideActionTime + SLIDE_STALL_TIME &&
                //slides not in position
                (
                        //left slide not in target position
                        (
                                leftSlide.getTargetPosition() + 5 < leftSlide.getCurrentPosition() ||
                                leftSlide.getTargetPosition() - 5 > leftSlide.getCurrentPosition()
                        ) ||
                        // or right slide not in target position
                        (
                                rightSlide.getTargetPosition() + 5 < rightSlide.getCurrentPosition() ||
                                rightSlide.getTargetPosition() - 5 > rightSlide.getCurrentPosition()
                        )
                )

        ) {
            leftSlide.setPower(0.0);
            rightSlide.setPower(0.0);
        }

        super.loop();
    }
}