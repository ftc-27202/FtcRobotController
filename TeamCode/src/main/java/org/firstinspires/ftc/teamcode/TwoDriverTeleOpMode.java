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

@TeleOp(name = "Two Driver TeleOp", group = "Robot")

public class TwoDriverTeleOpMode extends BaseTeleOpMode {

    public void loop () {
        double straightSpeed = gamepad1.left_bumper ? 0.4 : (1 - gamepad1.left_trigger);
        double strafeSpeed = gamepad1.left_bumper ? 0.4 : (1 - gamepad1.left_trigger);
        double turnSpeed = gamepad1.right_bumper ? 0.2 : (1 - gamepad1.right_trigger);

        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        moveBase(axial, lateral, yaw, straightSpeed, strafeSpeed, turnSpeed);

        //slideTiltLeft.setPower(-gamepad2.right_stick_y);
        //slideTiltRight.setPower(-gamepad2.right_stick_y);

        if (gamepad1.dpad_up) {
            slideTiltLeft.setTargetPosition((int) (SLIDE_TILT_UP * ARM_TICKS_PER_DEGREE));
            slideTiltLeft.setPower(1.0);
            slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTiltRight.setTargetPosition((int) (SLIDE_TILT_UP * ARM_TICKS_PER_DEGREE));
            slideTiltRight.setPower(1.0);
            slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.dpad_down) {
            slideTiltLeft.setTargetPosition((int) (SLIDE_TILT_DOWN * ARM_TICKS_PER_DEGREE));
            slideTiltLeft.setPower(1.0);
            slideTiltLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTiltRight.setTargetPosition((int) (SLIDE_TILT_DOWN * ARM_TICKS_PER_DEGREE));
            slideTiltRight.setPower(1.0);
            slideTiltRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

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
            armPivotLeft.setPosition(0);
            armPivotRight.setPosition(0);

            wristPivot.setPosition(0);
        } else if (gamepad2.y) {
            armPivotLeft.setPosition(0.5);
            armPivotRight.setPosition(0.5);

            wristPivot.setPosition(0);
        } else if (gamepad2.b) {
            armPivotLeft.setPosition(1);
            armPivotRight.setPosition(1);

            wristPivot.setPosition(0);
        }

        if (gamepad2.left_trigger > 0) {
            wristPivot.setPosition(0.0);
        } else if (gamepad2.right_trigger > 0) {
            wristPivot.setPosition(1.0);
        }

        telemetry.addData("slideMotorLeft", slideMotorLeft.getCurrentPosition());
        telemetry.addData("slideMotorRight", slideMotorRight.getCurrentPosition());


        super.loop();
    }
}
