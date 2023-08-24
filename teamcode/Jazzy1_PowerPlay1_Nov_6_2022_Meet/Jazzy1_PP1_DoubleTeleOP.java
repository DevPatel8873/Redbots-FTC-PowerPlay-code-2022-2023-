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
//package org.firstinspires.ftc.teamcode.robot1_PowerPlay1_Nov_6_2022_Meet;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@TeleOp(name="robot1_PP1_DoubleTeleOP", group="Dev")
//public class robot1_PP1_DoubleTeleOP extends LinearOpMode {
//
//    robot1_PP1_Hardware robot = new robot1_PP1_Hardware();
//
//
//    //Drive constants
//    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    static final double     SPOOL_DIAMETER_INCHES   = 2.0;
//
//    static final double SERVO_CLOSE = 0.2;
//    static final double SERVO_OPEN = 0.5;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Hello JET and Akshawty ", "robot is initialized");
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
//            //Controller 1 - Chassis movement
//            Mecanum.Motion motion = Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, gamepad1.left_stick_y,
//                    gamepad1.right_stick_x, -gamepad1.right_stick_y);
//
//            // Convert desired motion to wheel powers, with power clamping
//            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//            robot.frontLeft.setPower((wheels.frontLeft) * 0.82);
//            robot.frontRight.setPower((wheels.frontRight) * 0.82);
//            robot.backLeft.setPower((wheels.backLeft) * 0.82);
//            robot.backRight.setPower((wheels.backRight) * 0.82);
//
//            //Default case - stops the arm going up
//            if(gamepad1.a)
//            {
//                robot.claw.setPosition(SERVO_OPEN);
//            }
//            if(gamepad1.b)
//            {
//                robot.claw.setPosition(SERVO_CLOSE);
//            }
//
//            //GamePad 2
//            if (!gamepad2.left_bumper && !gamepad2.right_bumper)
//            {
//                robot.armMain.setPower(0);
//            }
//            if(gamepad2.left_bumper)
//            {
//                robot.armMain.setPower(-0.7);
//            }
//            if(gamepad2.right_bumper)
//            {
//                robot.armMain.setPower(0.7);
//            }
//
//        }
//    }
//
//    public void resetMotors()
//    {
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    public void liftSlides(double speed, double inches)
//    {
//        int newTarget;
//        robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        if(opModeIsActive())
//        {
//            double countsPerInchArm = ((COUNTS_PER_MOTOR_REV) / (2 * 3.1415));
//            newTarget = robot.armMain.getCurrentPosition() + (int)((inches) * countsPerInchArm);
//            robot.armMain.setTargetPosition(newTarget);
//            robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.armMain.setPower(Math.abs(speed));
//
//            while ((robot.armMain.isBusy())) {
//                idle();
//            }
//
//            robot.armMain.setPower(0);
//            robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        }
//    }
//}
