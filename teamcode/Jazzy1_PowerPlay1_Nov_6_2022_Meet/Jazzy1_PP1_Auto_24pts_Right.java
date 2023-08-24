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
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////@Disabled
//@Autonomous(name="robot1_PP1_Auto_24pts_Right", group="Dev")
//public class robot1_PP1_Auto_24pts_Right extends LinearOpMode {
//
//    //Creates Robot
//    robot1_PP1_Hardware robot = new robot1_PP1_Hardware();
//
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    //Drive constants
//    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double     SPOOL_DIAMETER_INCHES   = 2.0;
//
//    static final double SERVO_CLOSE = 0.2;
//    static final double SERVO_OPEN = 0.5;
//
//    //Color sensor values
//    float hsvValues[] = {0F, 0F, 0F};
//    float hsvValues2[] = {0F, 0F, 0F};
//
//    final float values[] = hsvValues;
//    final float values2[] = hsvValues2;
//
//    final double SCALE_FACTOR = 255;
//
//    int sleeveParkingIndicator;
//
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize hardware
//        robot.init(hardwareMap);
//        robot.claw.setPosition(SERVO_CLOSE);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("robot Initialized ", "Max points, easy autonomous");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        //Write code here
//        encoderLifter(0.7, 17, 5);
//        encoderStrafeRight(0.2, 18.5, 5);
//        sleeveParkingIndicator = colorRight();
//        sleep(1000);
//        encoderStrafeRight(0.2, 22, 5);
//        encoderLifter(0.7, 18, 5);
//        encoderStrafeRight(0.2,2, 5);
//        //encoderDrive(0.1, 2.5, 3, 3);
//        sleep(100);
//        robot.claw.setPosition(SERVO_OPEN);
//        sleep(100);
//        //encoderDrive(0.1, -5, -5, 3);
//        encoderLifter(0.7, -31, 5);
//        encoderStrafeLeft(0.2, 13.5,2);
//
//        //blue
//        if(sleeveParkingIndicator == 1)
//        {
//            telemetry.addData("Sleeve parking indicator ","1");
//            encoderDrive(0.4, 24, 24, 5);
//            encoderStrafeRight(0.1,3,5);
//        }
//        //red
//        else if(sleeveParkingIndicator == 2)
//        {
//            telemetry.addData("Sleeve parking indicator ","2");
//        }
//        //green
//        else if(sleeveParkingIndicator == 3)
//        {
//            telemetry.addData("Sleeve parking indicator ","3");
//            encoderDrive(0.4, -24, -24, 5);
//            encoderStrafeLeft(0.2,2,5);
//        }
//        else
//        {
//            telemetry.addData("Error ","Color not detected");
//        }
//        telemetry.update();
//
//
//
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.
//    }
//
//    public void printEncoderVals()
//    {
//        telemetry.addData("frontLeft", "Encoder Position > %i", robot.frontLeft.getCurrentPosition());
//        telemetry.addData("frontRight", "Encoder Position > %i", robot.frontRight.getCurrentPosition());
//        telemetry.addData("backLeft", "Encoder Position > %i", robot.backLeft.getCurrentPosition());
//        telemetry.addData("backRight", "Encoder Position > %i", robot.backRight.getCurrentPosition());
//        telemetry.update();
//    }
//
//    public int colorRight()
//    {
//        robot.colorBlindRight.enableLed(true);
//        if(robot.colorBlindRight.blue() > robot.colorBlindRight.red()
//                && robot.colorBlindRight.blue() > robot.colorBlindRight.green())
//        {
//            robot.colorBlindRight.close();
//            return 1;
//        }
//        else if(robot.colorBlindRight.red() > robot.colorBlindRight.blue()
//                && robot.colorBlindRight.red() > robot.colorBlindRight.green())
//        {
//            robot.colorBlindRight.close();
//            return 2;
//        }
//        else if(robot.colorBlindRight.green() > robot.colorBlindRight.red()
//                && robot.colorBlindRight.green() > robot.colorBlindRight.blue())
//        {
//            robot.colorBlindRight.close();
//            return 3;
//        }
//        else
//        {
//            robot.colorBlindRight.close();
//            return 0;
//        }
//    }
//
//    public int colorLeft()
//    {
//        robot.colorBlindLeft.enableLed(true);
//        if(robot.colorBlindLeft.blue() > robot.colorBlindLeft.red()
//                && robot.colorBlindLeft.blue() > robot.colorBlindLeft.green())
//        {
//            robot.colorBlindLeft.close();
//            return 1;
//        }
//        else if(robot.colorBlindLeft.red() > robot.colorBlindLeft.blue()
//                && robot.colorBlindLeft.red() > robot.colorBlindLeft.green())
//        {
//            robot.colorBlindLeft.close();
//            return 2;
//        }
//        else if(robot.colorBlindLeft.green() > robot.colorBlindLeft.red()
//                && robot.colorBlindLeft.green() > robot.colorBlindLeft.blue())
//        {
//            robot.colorBlindLeft.close();
//            return 3;
//        }
//        else
//        {
//            robot.colorBlindLeft.close();
//            return 0;
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
//    public void encoderLifter(double speed, double inches, double timeoutS)
//    {
//        int newTarget;
//        resetMotors();
//        if(opModeIsActive())
//        {
//            double countsPerInchArm = ((COUNTS_PER_MOTOR_REV) / (2 * 3.1415));
//            newTarget = robot.armMain.getCurrentPosition() + (int)((inches) * countsPerInchArm);
//            robot.armMain.setTargetPosition(newTarget);
//            robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            robot.armMain.setPower(Math.abs(speed));
//
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.armMain.isBusy())) {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :",
//                        robot.armMain.getCurrentPosition());
//                telemetry.update();
//            }
//
//            robot.armMain.setPower(0);
//            robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        }
//        resetMotors();
//    }
//
//    public void encoderStrafeRight(double speed, double inches, double timeoutS)
//    {
//        double revsToInches = inches / 11;
//        int newFrontRightTarget = robot.frontRight.getCurrentPosition() - ((int)(revsToInches * COUNTS_PER_MOTOR_REV));
//        int newBackRightTarget = robot.backRight.getCurrentPosition() + ((int)(revsToInches * COUNTS_PER_MOTOR_REV));
//
//        robot.frontLeft.setTargetPosition(newBackRightTarget);
//        robot.frontRight.setTargetPosition(newFrontRightTarget);
//        robot.backLeft.setTargetPosition(newFrontRightTarget);
//        robot.backRight.setTargetPosition(newBackRightTarget);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontLeft.setPower(-speed);
//        robot.frontRight.setPower(speed);
//        robot.backLeft.setPower(speed);
//        robot.backRight.setPower(-speed);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
//                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d : %7d", newFrontRightTarget,newBackRightTarget);
//            telemetry.addData("Path2",  "Running at %7d : %7d : %7d : %7d", robot.frontLeft.getCurrentPosition(),robot.frontRight.getCurrentPosition(),robot.backLeft.getCurrentPosition(),robot.backRight.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.setAllPower(0.0);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetMotors();
//    }
//
//    public void encoderStrafeLeft(double speed, double inches, double timeoutS)
//    {
//        double revsToInches = inches / 11;
//        int newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (revsToInches * COUNTS_PER_MOTOR_REV);
//        int newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (revsToInches * COUNTS_PER_MOTOR_REV);
//
//        robot.frontLeft.setTargetPosition(newFrontLeftTarget);
//        robot.frontRight.setTargetPosition(newBackLeftTarget);
//        robot.backLeft.setTargetPosition(newBackLeftTarget);
//        robot.backRight.setTargetPosition(newFrontLeftTarget);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontLeft.setPower(-speed);
//        robot.frontRight.setPower(speed);
//        robot.backLeft.setPower(speed);
//        robot.backRight.setPower(-speed);;
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
//                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1", "Running to %7d", newFrontLeftTarget);
//            telemetry.addData("Path2", "Running at %7d", robot.frontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//
//
//        robot.setAllPower(0.0);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetMotors();
//    }
//
//    //To go backwards use negative distance, NOT SPEED
//    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
//    {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            newRightTarget = robot.backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            robot.frontLeft.setTargetPosition(newLeftTarget);
//            robot.backLeft.setTargetPosition(newLeftTarget);
//            robot.frontRight.setTargetPosition(newRightTarget);
//            robot.backRight.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.setAllPower(Math.abs(speed));
//
//
//            while (opModeIsActive() && runtime.seconds() < timeoutS
//                    && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
//                    && robot.backLeft.isBusy() && robot.backRight.isBusy())
//            {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d : %7d", newLeftTarget,newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d : %7d : %7d : %7d", robot.frontLeft.getCurrentPosition(),robot.frontRight.getCurrentPosition(),robot.backLeft.getCurrentPosition(),robot.backRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.setAllPower(0.0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            resetMotors();
//        }
//    }
//}
