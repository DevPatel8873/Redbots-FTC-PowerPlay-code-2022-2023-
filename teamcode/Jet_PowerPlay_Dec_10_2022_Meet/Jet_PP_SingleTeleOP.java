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
//package org.firstinspires.ftc.teamcode.Jet_PowerPlay_Dec_10_2022_Meet;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
////@Disabled
//@TeleOp(name="Jet_PP_SingleTeleOP", group="Dev")
//public class Jet_PP_SingleTeleOP extends LinearOpMode {
//
//    Jet_PP_Hardware robot = new Jet_PP_Hardware();
//
//    final double COUNTS_PER_MOTOR_REV = 537.7;
//    final double SPOOL_DIAMETER_INCHES = 1.37;
//
//    boolean LAST_gamePad1_A_Press = false;
//    boolean LAST_gamePad1_B_Press = false;
//    boolean LAST_gamePad1_X_Press = false;
//    boolean LAST_gamePad1_Y_Press = false;
//
//    boolean LAST_gamePad1_Rbump_Press = false;
//    boolean LAST_gamePad1_LBump_Press = false;
//
//    boolean LAST_gamePad1_dUP_Press = false;
//    boolean LAST_gamePad1_dDown_Press = false;
//    boolean LAST_gamePad1_dLeft_Press = false;
//    boolean LAST_gamePad1_dRight_Press = false;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        final double CLAW_CLOSE = Jet_PP_Hardware.CLAW_CLOSE;
//        final double CLAW_OPEN = Jet_PP_Hardware.CLAW_OPEN;
//        final double TURRET_FULL = Jet_PP_Hardware.TURRET_FULL;
//        final double TURRET_CLOSE = Jet_PP_Hardware.TURRET_CLOSE;
//
//        final double SUB_POS = Jet_PP_Hardware.SUB_POS;
//        final double GRD_POS = Jet_PP_Hardware.GRD_POS;
//        final double LOW_POS = Jet_PP_Hardware.LOW_POS;
//        final double MED_POS = Jet_PP_Hardware.MED_POS;
//        final double HIGH_POS = Jet_PP_Hardware.HIGH_POS;
//
//        int currentPos = 0;
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Hello JET ", "Budget turret is initialized");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            boolean CURR_gamePad1_A_Press = gamepad1.a;
//            boolean CURR_gamePad1_B_Press = gamepad1.b;
//            boolean CURR_gamePad1_X_Press = gamepad1.x;
//            boolean CURR_gamePad1_Y_Press = gamepad1.y;
//
//            boolean CURR_gamePad1_Rbump_Press = gamepad1.right_bumper;
//            boolean CURR_gamePad1_LBump_Press = gamepad1.left_bumper;
//
//            boolean CURR_gamePad1_dUP_Press = gamepad1.dpad_up;
//            boolean CURR_gamePad1_dDown_Press = gamepad1.dpad_down;
//            boolean CURR_gamePad1_dLeft_Press = gamepad1.dpad_left;
//            boolean CURR_gamePad1_dRight_Press = gamepad1.dpad_right;
//
//            //I guess this block of code programs all the joystick motion through the Mecanum class
//
//            Mecanum.Motion motion = Mecanum.joystickToMotion(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y);
//
//            // Convert desired motion to wheel powers, with power clamping
//            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//            robot.frontLeft.setPower((wheels.frontLeft) * 0.82);
//            robot.frontRight.setPower((wheels.frontRight) * 0.82);
//            robot.backLeft.setPower((wheels.backLeft) * 0.82);
//            robot.backRight.setPower((wheels.backRight) * 0.82);
//
//
//            if(CURR_gamePad1_A_Press && !LAST_gamePad1_A_Press)
//            {
//                robot.claw.setPosition(CLAW_OPEN);
//            }
//            if(CURR_gamePad1_B_Press && !LAST_gamePad1_B_Press)
//            {
//                robot.claw.setPosition(CLAW_CLOSE);
//            }
//
//            if(!gamepad1.left_bumper && !gamepad1.right_bumper)
//            {
//                robot.armMain.setPower(0);
//
//            }
//            if(gamepad1.right_bumper)
//            {
//                robot.armMain.setPower(1);
//            }
//            if(gamepad1.left_bumper)
//            {
//                robot.armMain.setPower(-1);
//            }
//
//
//            LAST_gamePad1_A_Press = CURR_gamePad1_A_Press;
//            LAST_gamePad1_B_Press = CURR_gamePad1_B_Press;
//            LAST_gamePad1_X_Press = CURR_gamePad1_X_Press;
//            LAST_gamePad1_Y_Press = CURR_gamePad1_Y_Press;
//
//            LAST_gamePad1_Rbump_Press = CURR_gamePad1_Rbump_Press;
//            LAST_gamePad1_LBump_Press = CURR_gamePad1_LBump_Press;
//
//            LAST_gamePad1_dUP_Press = CURR_gamePad1_dUP_Press;
//            LAST_gamePad1_dDown_Press = CURR_gamePad1_dDown_Press;
//            LAST_gamePad1_dLeft_Press = CURR_gamePad1_dLeft_Press;
//            LAST_gamePad1_dRight_Press = CURR_gamePad1_dRight_Press;
//        }
//    }
//    public void liftArm(double inches, double s)
//    {
//        double speed = s;
//        int newTarget;
//        if (opModeIsActive()) {
//            newTarget = robot.armMain.getCurrentPosition() + (int)((inches) * (COUNTS_PER_MOTOR_REV / (SPOOL_DIAMETER_INCHES * 3.1415)));
//            robot.armMain.setTargetPosition(newTarget);
//
//            robot.armMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.armMain.setPower(Math.abs(speed));
//
//        }
//    }
//}
