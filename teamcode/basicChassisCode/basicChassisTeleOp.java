///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.basicChassisCode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.basicChassisCode.basicChassisHardware;
//import org.firstinspires.ftc.teamcode.basicChassisCode.Mecanum;
//
//
//@TeleOp(name="TeleOPTEST", group="Dev")
//public class basicChassisTeleOp extends LinearOpMode {
//
//    basicChassisHardware robot = new basicChassisHardware();
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        final double SERVO_CLOSE = 0.2;
//        final double SERVO_OPEN = 0.5;
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Hello JET ", "robot is initialized");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            //I guess this block of code programs all the joystick motion through the Mecanum class
//
//            Mecanum.Motion motion = Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, gamepad1.left_stick_y,
//                    gamepad1.right_stick_x, -gamepad1.right_stick_y);
//
//            // Convert desired motion to wheel powers, with power clamping
//            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//            robot.frontLeft.setPower((wheels.frontLeft) * 0.33);
//            robot.frontRight.setPower((wheels.frontRight) * 0.33);
//            robot.backLeft.setPower((wheels.backLeft) * 0.33);
//            robot.backRight.setPower((wheels.backRight) * 0.33);
//
//            //Default case - stops the arm going up
//            if (!gamepad1.right_bumper && !gamepad1.left_bumper)
//            {
//                robot.arm.setPower(0);
//            }
//            //Move slides up
//            if(gamepad1.right_bumper)
//            {
//                robot.arm.setPower(0.7);
//            }
//            //Move slides down
//            if(gamepad1.left_bumper)
//            {
//                robot.arm.setPower(-0.7);
//            }
//
//
//
//
//
//        }
//    }
//}
