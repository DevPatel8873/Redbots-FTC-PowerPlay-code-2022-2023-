/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Jet_PowerPlay_Dec_10_2022_Meet;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.AxisDirection;



//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
public class Jet_PP_Hardware {

    /* Declare OpMode members. */
    HardwareMap hwMap             =  null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.HardwareMap hwMap = null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)


    public DcMotor armMain = null;

    public Servo claw = null;
    public Servo turret = null;

    public ColorSensor colorBlindRight;
    public ColorSensor colorBlindLeft;

    public TouchSensor magnetSens;

    // Define Drive constants; THESE CAN BE APPLIED TO ALL FILES
    public static final double SLIDES_SPEED = 1;

    public static final double CLAW_CLOSE = -0.2;
    public static final double CLAW_OPEN = 0.32;
    public static final double TURRET_FULL = 0.81;
    public static final double TURRET_CLOSE = 0.145;

    //These are all calibrated to 312 encoder not 435 so not to scale but work
    //As of now they are calibrated :)
    public static final double SUB_POS = 0.0;
    public static final double GRD_POS = 2;
    public static final double LOW_POS = 14;
    public static final double MED_POS = 24;
    public static final double HIGH_POS = 33;

    public static final double COUNTS_PER_MOTOR_REV_312 = 537.7;
    public static final double COUNTS_PER_MOTOR_REV_435 = 384.5;
    public static final double COUNTS_PER_MOTOR_REV_60  = 2786.2;
    public static final double COUNTS_PER_MOTOR_REV_84  = 1992.6;
    public static final double SPOOL_DIAMETER_INCHES = 1.37;

    //public SampleMecanumDrive drive;

    public DcMotor frontLeft  = null; //C: 0
    public DcMotor backLeft  = null; //C: 1
    public DcMotor frontRight   = null; //C: 3
    public DcMotor backRight   = null; //C: 2

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Camera and Apriltags
    public static OpenCvCamera camera;
    public static AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static final double CAM_FX = 578.272;
    public static final double CAM_FY = 578.272;
    public static final double CAM_CX = 402.145;
    public static final double CAM_CY = 221.506;
    public static final double APRILTAGSIZE = 0.166;



    public Jet_PP_Hardware(LinearOpMode opmode)
    {
        myOpMode = opmode;
    }

    public Jet_PP_Hardware(){ }

    public void init(HardwareMap ahwMap)    {
        // Define and Initialize Motors and Servos
        hwMap = ahwMap;

        //drive = new SampleMecanumDrive(hwMap);
        frontLeft       = hwMap.get(DcMotor.class, "frontLeft");
        frontRight      = hwMap.get(DcMotor.class, "frontRight");
        backLeft        = hwMap.get(DcMotor.class, "backLeft");
        backRight       = hwMap.get(DcMotor.class, "backRight");

        armMain        = hwMap.get(DcMotor.class, "armMain");

        claw           = hwMap.get(Servo.class, "claw");
        turret         = hwMap.get(Servo.class, "turret");

        colorBlindRight = hwMap.get(ColorSensor.class, "colorBlindRight");
        colorBlindLeft  = hwMap.get(ColorSensor.class, "colorBlindLeft");

        magnetSens      = hwMap.get(TouchSensor.class, "magnetSens");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        //BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(APRILTAGSIZE, CAM_FX, CAM_FY, CAM_CX, CAM_CY);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });




        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        armMain.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        turret.setDirection(Servo.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armMain.setPower(0);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        claw.setPosition(CLAW_CLOSE);
        turret.setPosition(TURRET_CLOSE);


    }

    public void setAllPower(double p)
    {
        setMotorPower(p,p,p,p);
    }


    public void setMotorPower(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed)
    {
        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backRight.setPower(backRightSpeed);
        backLeft.setPower(backLeftSpeed);
    }



}
