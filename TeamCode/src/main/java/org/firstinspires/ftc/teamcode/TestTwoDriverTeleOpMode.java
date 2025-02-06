
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

@TeleOp(name = "Testing12 Two Driver TeleOp", group = "Robot")

public class TestTwoDriverTeleOpMode extends BaseTeleOpMode {

    public void loop () {
        double straightSpeed = gamepad1.left_bumper ? 0.4 : (1 - gamepad1.left_trigger);
        double strafeSpeed = gamepad1.left_bumper ? 0.4 : (1 - gamepad1.left_trigger);
        double turnSpeed = gamepad1.right_bumper ? 0.2 : (1 - gamepad1.right_trigger);

        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        slideTiltLeft.setTargetPosition((int)slideTiltTarget);
        slideTiltRight.setTargetPosition((int)slideTiltTarget);
        ((DcMotorEx) slideTiltLeft).setVelocity(700);
        slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) slideTiltRight).setVelocity(700);
        slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveBase(axial, lateral, yaw, straightSpeed, strafeSpeed, turnSpeed);

        //slideTiltLeft.setPower(-gamepad1.right_stick_y);
        //slideTiltRight.setPower(-gamepad1.right_stick_y);
        if (gamepad1.dpad_up){
            slideTiltTarget = 125*ARM_TICKS_PER_DEGREE;
        } else if (gamepad1.dpad_down) {
            slideTiltTarget = 25*ARM_TICKS_PER_DEGREE;

        }
        /*if (gamepad1.x) {
            slideTiltLeft.setTargetPosition(0);
            slideTiltLeft.setPower(0.2);
            slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTiltRight.setTargetPosition(0);
            slideTiltRight.setPower(0.2);
            slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.y) {
            slideTiltLeft.setTargetPosition(100);
            slideTiltLeft.setPower(0.2);
            slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTiltRight.setTargetPosition(100);
            slideTiltRight.setPower(0.2);
            slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.b) {
            slideTiltLeft.setTargetPosition(500);
            slideTiltLeft.setPower(0.2);
            slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTiltRight.setTargetPosition(500);
            slideTiltRight.setPower(0.2);
            slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        */
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);

        slideMotorLeft.setPower(-gamepad2.left_stick_y);
        slideMotorRight.setPower(-gamepad2.left_stick_y);

        if (gamepad2.dpad_up) {
            slideMotorLeft.setTargetPosition(SLIDES_UP);
            slideMotorLeft.setPower(1.0);
            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotorRight.setTargetPosition(SLIDES_UP);
            slideMotorRight.setPower(1.0);
            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.dpad_down) {
            slideMotorLeft.setTargetPosition(SLIDES_DOWN);
            slideMotorLeft.setPower(1.0);
            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideMotorRight.setTargetPosition(SLIDES_DOWN);
            slideMotorRight.setPower(1.0);
            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.a) {
            armPivotLeft.setPosition(.3);
            armPivotRight.setPosition(.3);


        } else if (gamepad2.x) {
            armPivotLeft.setPosition(0);
            armPivotRight.setPosition(0);


        } else if (gamepad2.b) {
            armPivotLeft.setPosition(.9);
            armPivotRight.setPosition(.9);
        }
        if (gamepad2.dpad_left){
            wristPivot.setPosition(0);
        } else if (gamepad2.dpad_right){
            wristPivot.setPosition(1);
        }
        if (gamepad2.left_bumper){
            clawPivot.setPosition(0);
        }else if (gamepad2.right_bumper){
            clawPivot.setPosition(1);
        }

        if (gamepad1.a){
            claw.setPosition(0);
        }else if (gamepad1.b){
            claw.setPosition(0.5);
        }
        telemetry.addData("ClawPos",claw.getPosition());
        telemetry.addData("ClawPivotPos", clawPivot.getPosition());
        telemetry.addData("slideMotorLeft", slideMotorLeft.getCurrentPosition());
        telemetry.addData("slideMotorRight", slideMotorRight.getCurrentPosition());
        telemetry.addData("Trigger", gamepad2.left_trigger);

        super.loop();
    }
}
/*
* if (isGoingDown && slideTiltLeft.getCurrentPosition() > (100.0 * ARM_TICKS_PER_DEGREE)) {
            slideTiltLeft.setPower(0);
            slideTiltRight.setPower(0);
        } else {
            slideTiltLeft.setPower(SLIDE_TILT_POWER);
            slideTiltRight.setPower(SLIDE_TILT_POWER);
        }*/
