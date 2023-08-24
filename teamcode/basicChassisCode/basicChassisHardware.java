///* Copyright (c) 2022 FIRST. All rights reserved.
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
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
////@Disabled
//public class basicChassisHardware{
//
//    /* Declare OpMode members. */
//    HardwareMap hwMap             =  null;
//
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.HardwareMap hwMap = null;
//
//    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
//    public DcMotor frontLeft  = null; //C: 0
//    public DcMotor frontRight   = null; //C: 1
//    public DcMotor backLeft  = null; //C: 2
//    public DcMotor backRight   = null; //C: 3
//
//    public DcMotor arm = null;
//
//    //public Servo claw = null;
//
//    public static final double COUNTS_PER_MOTOR_REV_312 = 537.7;
//    public static final double COUNTS_PER_MOTOR_REV_435 = 384.5;
//    public static final double ODOMETER_WHEEL_DIAMETER_INCHES = 2.0;
//    public static final double REV_COUNTS_PER_ENCODER_REV = 8192.0;
//
//
//    public basicChassisHardware(LinearOpMode opmode)
//    {
//        myOpMode = opmode;
//    }
//
//    public basicChassisHardware(){ }
//
//    public void init(HardwareMap ahwMap)    {
//        // Define and Initialize Motors and Servos
//        hwMap = ahwMap;
//
//        frontLeft      = hwMap.get(DcMotor.class, "frontLeft");
//        frontRight     = hwMap.get(DcMotor.class, "frontRight");
//        backLeft       = hwMap.get(DcMotor.class, "backLeft");
//        backRight      = hwMap.get(DcMotor.class, "backRight");
//
//
//
//
//        arm = hwMap.get(DcMotor.class, "arm");
//
//        //claw            = hwMap.get(Servo.class, "claw");
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//
//        arm.setDirection(DcMotor.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//
//        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//    }
//
//    public void setAllPower(double p)
//    {
//        setMotorPower(p,p,p,p);
//    }
//
//
//    public void setMotorPower(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed)
//    {
//        frontLeft.setPower(frontLeftSpeed);
//        frontRight.setPower(frontRightSpeed);
//        backRight.setPower(backRightSpeed);
//        backLeft.setPower(backLeftSpeed);
//    }
//
//
//
//}
