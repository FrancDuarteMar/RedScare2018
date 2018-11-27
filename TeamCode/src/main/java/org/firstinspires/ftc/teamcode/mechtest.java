package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Mech Test ", group="Red Scare")
public class mechtest extends OpMode {

        private DcMotor frontLeftMotor = null;
        private DcMotor frontRightMotor = null;
        private DcMotor backLeftMotor = null;
        private DcMotor backRightMotor = null;
        private float drive = 0;


        @Override
        public void init() {
            frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
            backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");

//        initGoldAlignDetector();
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //craneMotor.isBusy();




        }

        @Override
        public void loop() {

//        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
//        telemetry.addData("X Pos" , detector.getXPosition());
            telemetry.update();

            /* DRIVING */


            if (!gamepad1.dpad_left && !gamepad1.dpad_right) {

                frontLeftMotor.setPower(gamepad1.left_stick_y);
                backLeftMotor.setPower(gamepad1.left_stick_y);
                frontRightMotor.setPower(gamepad1.right_stick_y);
                backRightMotor.setPower(gamepad1.right_stick_y);

                if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                    drive = 1;
                } else {
                    drive = 0;
                }
            }

            if (drive == 0) {
                if (gamepad1.dpad_right) {
                    frontLeftMotor.setPower(.5);
                    backLeftMotor.setPower(-.5);
                    frontRightMotor.setPower(.5);
                    backRightMotor.setPower(-.5);

                } else if (gamepad1.dpad_left) {
                    frontLeftMotor.setPower(-.5);
                    backLeftMotor.setPower(.5);
                    frontRightMotor.setPower(-.5);
                    backRightMotor.setPower(.5);

                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }
            }



//            //left stick going backwards by pushing forward
//            if (gamepad1.left_stick_y < 0) {
//                frontLeftMotor.setPower(.75);
//                backLeftMotor.setPower(.75);
//            }
//            //right stick going backwards by pushing backward
//            if (gamepad1.right_stick_y < 0) {
//                frontRightMotor.setPower(.75);
//                backRightMotor.setPower(.75);
//            }
//
//            if (gamepad1.right_stick_y == 0) {
//                frontRightMotor.setPower(0);
//                backRightMotor.setPower(0);
//            }
//            //left stick going forwards
//            if (gamepad1.left_stick_y > 0) {
//                frontLeftMotor.setPower(-.75);
//                backLeftMotor.setPower(-.75);
//            }
//            //right stick going forwards
//            if (gamepad1.right_stick_y > 0) {
//                frontRightMotor.setPower(-.75);
//                backRightMotor.setPower(-.75);
//            }
//            //left stick at zero
//            if (gamepad1.left_stick_y == 0) {
//                frontLeftMotor.setPower(0);
//                backLeftMotor.setPower(0);
//
//            }

//       //right stick at zero
//        if (gamepad1.right_stick_y == 0 && (gamepad1.right_stick_x == 0 )) {
//            frontRightMotor.setPower(0);
//            backRightMotor.setPower(0);
//        }
//
//        //right on right stick, turns right
//        if (gamepad1.left_stick_y > 0) {
//            backLeftMotor.setPower(.75);
//            backRightMotor.setPower(-.75);
//            frontLeftMotor.setPower(.75);
//            frontRightMotor.setPower(-.75);
//        }
//
//        //left on right turns left
//        if (gamepad1.left_stick_y < 0) {
//            backLeftMotor.setPower(-.75);
//            backRightMotor.setPower(.75);
//            frontLeftMotor.setPower(-.75);
//            frontRightMotor.setPower(.75);
//        }
//
//        if (gamepad1.right_stick_y == 0){
//            frontRightMotor.setPower(0);
//            frontLeftMotor.setPower(0);
//            backRightMotor.setPower(0);
//            backLeftMotor.setPower(0);
//        }
//
//        if (gamepad1.left_stick_y == 0) {
//            backLeftMotor.setPower(0);
//            backRightMotor.setPower(0);
//            frontLeftMotor.setPower(0);
//            frontRightMotor.setPower(0);
//        }
//
//        if (gamepad1.right_stick_y > 0){
//            frontRightMotor.setPower(.75);
//            frontLeftMotor.setPower(.75);
//            backRightMotor.setPower(.75);
//            backLeftMotor.setPower(.75);
//        }
//
//        if (gamepad1.right_stick_y < 0){
//            frontRightMotor.setPower(-.75);
//            frontLeftMotor.setPower(-.75);
//            backRightMotor.setPower(-.75);
//            backLeftMotor.setPower(-.75);
//        }



       /* if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y ){
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }*/




            /* TURN AROUND */

            //turn 180(around)
            if (gamepad1.y) {
                turn(180);
            }







            /* PICK UP MECHANISM, NOT WORKING ON TEST BED BECAUSE THERE IS NO COLLECTOR */
/*
            //pickup Mechanism with Dpad

            //bumper initiates pickup mechanism
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                pickupMotor.setPower(.75);
            }
            //dpad down releases the balls

            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                pickupMotor.setPower(-.75);
            }


            telemetry.addData("enconder ", craneMotor.getCurrentPosition());
            telemetry.update();

            */

            //no pressing, stops all movement
/*
        if(gamepad1.right_trigger >1){
            servoMain.setPosition(-1);
        }
        if (gamepad1.left_trigger <1){
            servoMain.setPosition(0);
        }
*/
//        if (gamepad1.a) {
////            detector.enable();
//        }
//        if (gamepad1.x) {
////            detector.disable();
//        }
        }

        public void turn ( double degrees){
            degrees = 9.5 * degrees;
            if (degrees > 0) {
                while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees) {

                    frontRightMotor.setPower(-.35);
                    backRightMotor.setPower(-.35);
                    frontLeftMotor.setPower(.35);
                    backLeftMotor.setPower(.35);
                }
            }
            if (degrees < 0) {
                degrees *= -1;
                while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees) {

                    frontRightMotor.setPower(.35);
                    backRightMotor.setPower(.35);
                    frontLeftMotor.setPower(-.35);
                    backLeftMotor.setPower(-.35);
                }
            }
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            //   reset();

        }


/*
//D-pad crane motor code

        if (gamepad1.dpad_up){
            craneMotor.setPower(-.1);
        }
        else if (gamepad1.dpad_down) {
            craneMotor.setPower(.1);
        }
        else {
            craneMotor.setPower(0);
        }*/

        /*frontRightMotor.setPower(gamepad1.right_stick_y);
        craneMotor.setPower(gamepad1.right_trigger);
        craneMotor.setPower(gamepad1.left_trigger);*/

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE


//    private void initGoldAlignDetector() {
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
//
//    }

    }

