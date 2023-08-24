/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Jet_PowerPlay_Dec_10_2022_Meet.oldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jet_PowerPlay_Dec_10_2022_Meet.Jet_PP_Hardware;

@Disabled
@Autonomous(name="24pt RIGHT MED", group="Dev")
public class Jet_PP_Auto_24pts_RIGHT_MED_Jan8 extends LinearOpMode {

    //Creates Robot
    Jet_PP_Hardware robot = new Jet_PP_Hardware();

    private ElapsedTime     runtime = new ElapsedTime();

    //Drive constants
    static final double     COUNTS_PER_MOTOR_REV_312    = Jet_PP_Hardware.COUNTS_PER_MOTOR_REV_312 ;
    static final double     COUNTS_PER_MOTOR_REV_60    = Jet_PP_Hardware.COUNTS_PER_MOTOR_REV_60;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV_312 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     SPOOL_DIAMETER_INCHES   = Jet_PP_Hardware.SPOOL_DIAMETER_INCHES;

    static final double CLAW_CLOSE = Jet_PP_Hardware.CLAW_CLOSE;
    static final double CLAW_OPEN = Jet_PP_Hardware.CLAW_OPEN;
    static final double TURRET_FULL = Jet_PP_Hardware.TURRET_FULL;
    static final double TURRET_CLOSE = Jet_PP_Hardware.TURRET_CLOSE;

    static final double SLIDES_SPEED = Jet_PP_Hardware.SLIDES_SPEED;

    final double SUB_POS = Jet_PP_Hardware.SUB_POS;
    final double GRD_POS = Jet_PP_Hardware.GRD_POS;
    final double LOW_POS = Jet_PP_Hardware.LOW_POS;
    final double MED_POS = Jet_PP_Hardware.MED_POS;
    final double HIGH_POS = Jet_PP_Hardware.HIGH_POS;

    int sleeveParkingIndicator;


    @Override
    public void runOpMode() {

        // Initialize hardware
        robot.init(hardwareMap);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Jet Initialized ", "Max points, easy autonomous");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Write code here
        //Positioning -> LeftSide = 6in from end of wheel | RightSide = 4.5in from end of wheel (all @ middle of junction)
        resetMotors();
        encoderStrafeRight(0.4, 21, 5);
        sleep(750);
        sleeveParkingIndicator = colorRight();
        encoderStrafeRight(0.4, 20, 7);
        //encoderStrafeLeft(0.3, 15, 5);
        //encoderDrive(0.4, 28,28,5);
        //encoderStrafeRight(0.4,13.5,4);
        encoderLifter(1, 8, 3);
        sleep(1000);
        encoderDrive(0.1, 4, 4, 5);
        //encoderLifter(SLIDES_SPEED, -0.5, 3);
        sleep(1500);
        robot.claw.setPosition(CLAW_OPEN);
        sleep(750);
        encoderDrive(0.1, -4, -4, 2);
        encoderLifter(SLIDES_SPEED, -4, 2);



        encoderStrafeLeft(0.4, 14, 5);

        if(sleeveParkingIndicator == 1)
        {
            encoderDrive(0.2,24,24,2);
            robot.claw.setPosition(CLAW_OPEN);
        }
        else if(sleeveParkingIndicator == 2)
        {
            //encoderDrive(0.4, -24,-24,5);
            robot.claw.setPosition(CLAW_OPEN);
        }
        else
        {
            encoderDrive(0.4, -28,-28,5);
            robot.claw.setPosition(CLAW_OPEN);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
    public void printEncoderVals()
    {
        telemetry.addData("frontLeft", "Encoder Position > %i", robot.frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", "Encoder Position > %i", robot.frontRight.getCurrentPosition());
        telemetry.addData("backLeft", "Encoder Position > %i", robot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", "Encoder Position > %i", robot.backRight.getCurrentPosition());
        telemetry.update();
    }

    public int colorRight()
    {
        robot.colorBlindRight.enableLed(true);
        int blue = robot.colorBlindRight.blue();
        int green = robot.colorBlindRight.green();
        int red = robot.colorBlindRight.red();
        robot.colorBlindRight.close();
        if(blue > red && blue > green)
        {
            return 1;
        }
        else if(red > blue && red > green)
        {
            return 2;
        }
        else if(green > red && green > blue)
        {
            return 3;
        }
        else
        {
            return 0;
        }
    }

    public int colorLeft()
    {
        robot.colorBlindLeft.enableLed(true);
        int blue = robot.colorBlindLeft.blue();
        int green = robot.colorBlindLeft.green();
        int red = robot.colorBlindLeft.red();
        robot.colorBlindLeft.close();
        if(blue > red && blue > green)
        {
            return 1;
        }
        else if(red > blue && red > green)
        {
            return 2;
        }
        else if(green > red && green > blue)
        {
            return 3;
        }
        else
        {
            return 0;
        }
    }

    public void servoScore()
    {
        robot.turret.setPosition(TURRET_FULL);
        sleep(1000);
        robot.claw.setPosition(CLAW_OPEN);
        sleep(1000);
        robot.turret.setPosition(TURRET_CLOSE);
        sleep(1000);
    }

    public void resetMotors()
    {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void encoderLifter(double speed, double inches, double timeoutS)
    {
        int newTarget;
        if(opModeIsActive())
        {
            newTarget = robot.armMain.getCurrentPosition() + (int)((inches) * (COUNTS_PER_MOTOR_REV_60 / (SPOOL_DIAMETER_INCHES * 3.1415)));
            robot.armMain.setTargetPosition(newTarget);
            robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();


            robot.armMain.setPower(Math.abs(speed));
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.armMain.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :", newTarget);
                telemetry.addData("Path2",  "Running at %7d :",
                        robot.armMain.getCurrentPosition());
                telemetry.update();
            }

            robot.armMain.setPower(0);
            robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        }

        sleep(250);

    }

    public void encoderStrafeRight(double speed, double inches, double timeoutS)
    {
        double revsToInches = inches / 11;
        int newFrontRightTarget = robot.frontRight.getCurrentPosition() - ((int)(revsToInches * COUNTS_PER_MOTOR_REV_312));
        int newBackRightTarget = robot.backRight.getCurrentPosition() + ((int)(revsToInches * COUNTS_PER_MOTOR_REV_312));

        //int newArmTarget = robot.armMain.getCurrentPosition() + (int)((armHeight) * (COUNTS_PER_MOTOR_REV_60 / (SPOOL_DIAMETER_INCHES * 3.1415)));

        robot.frontLeft.setTargetPosition(newBackRightTarget);
        robot.frontRight.setTargetPosition(newFrontRightTarget);
        robot.backLeft.setTargetPosition(newFrontRightTarget);
        robot.backRight.setTargetPosition(newBackRightTarget);
        //robot.armMain.setTargetPosition(newArmTarget);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);
        //robot.armMain.setPower(Math.abs(SLIDES_SPEED));

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d : %7d", newFrontRightTarget,newBackRightTarget);
            telemetry.addData("Path2",  "Running at %7d : %7d : %7d : %7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
            telemetry.update();
        }

        robot.setAllPower(0.0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        robot.armMain.setPower(0);
//        robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetMotors();
        sleep(250);

    }

    public void encoderStrafeLeft(double speed, double inches, double timeoutS)
    {
        double revsToInches = inches / 11;
        int newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - (int) (revsToInches * COUNTS_PER_MOTOR_REV_312);
        int newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (revsToInches * COUNTS_PER_MOTOR_REV_312);

        //int newArmTarget = robot.armMain.getCurrentPosition() + (int)((armHeight) * (COUNTS_PER_MOTOR_REV_60 / (SPOOL_DIAMETER_INCHES * 3.1415)));


        robot.frontLeft.setTargetPosition(newFrontLeftTarget);
        robot.frontRight.setTargetPosition(newBackLeftTarget);
        robot.backLeft.setTargetPosition(newBackLeftTarget);
        robot.backRight.setTargetPosition(newFrontLeftTarget);
        //robot.armMain.setTargetPosition(newArmTarget);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);
        //robot.armMain.setPower(Math.abs(SLIDES_SPEED));

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", newFrontLeftTarget);
            telemetry.addData("Path2", "Running at %7d", robot.frontLeft.getCurrentPosition());
            telemetry.update();
        }


        robot.setAllPower(0.0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        robot.armMain.setPower(0);
//        robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetMotors();
        sleep(250);

    }

    //To go backwards use negative distance, NOT SPEED
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            //int newArmTarget = robot.armMain.getCurrentPosition() + (int)((armHeight) * (COUNTS_PER_MOTOR_REV_60 / (SPOOL_DIAMETER_INCHES * 3.1415)));

            //robot.armMain.setTargetPosition(newArmTarget);

            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.setAllPower(Math.abs(speed));
            //robot.armMain.setPower(Math.abs(SLIDES_SPEED));


            while (opModeIsActive() && runtime.seconds() < timeoutS
                    && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
                    && robot.backLeft.isBusy() && robot.backRight.isBusy())
            {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d : %7d", newLeftTarget,newRightTarget);
                telemetry.addData("Path2",  "Running at %7d : %7d : %7d : %7d", robot.frontLeft.getCurrentPosition(),robot.frontRight.getCurrentPosition(),robot.backLeft.getCurrentPosition(),robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setAllPower(0.0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            robot.armMain.setPower(0);
//            robot.armMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            resetMotors();
        }
        sleep(250);

    }

    public double getCurrHeight()
    {
        return (robot.armMain.getCurrentPosition() / COUNTS_PER_MOTOR_REV_60) * (SPOOL_DIAMETER_INCHES * 3.1415);
    }
}

