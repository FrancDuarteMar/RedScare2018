package org.firstinspires.ftc.teamcode.TeleOPTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@TeleOp(name=" REV TEST  10 ", group="Red Scare")
public class revDrive extends OpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    //private DcMotor pickupMotor = null;
   // private DcMotor craneMotor = null;
    private float drive = 0;

    private double speed = 1.05;


    //setting limits
    private int craneTop = 3000; //4000
    private int craneLow = 50;



    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
       // pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");
        //craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");





        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

     //   craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        telemetry.update();

        /* DRIVING */


        if (!gamepad1.dpad_left && !gamepad1.dpad_right) {   //might use && instead of ||

            frontLeftMotor.setPower(-gamepad1.left_stick_y / speed);
            backLeftMotor.setPower(-gamepad1.left_stick_y / speed);
            frontRightMotor.setPower(-gamepad1.right_stick_y / speed);
            backRightMotor.setPower(-gamepad1.right_stick_y / speed);

            // ONLY ALLOWS STRAFING WHEN NOT MOVING FORWARD OR BACKWARDS //

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                drive = 1; //sets driving to active
            } else {
                drive = 0; //says there's no driving
            }
        }

        // STRAFE //

        //proportional strafe
        //set value strafe
        if (drive == 0) {
            if (gamepad1.dpad_left) { //used to be left
                frontLeftMotor.setPower(1 / speed); //.5
                backLeftMotor.setPower(-1 / speed); //-1 *
                frontRightMotor.setPower(1 / speed); //*
                backRightMotor.setPower(-1 / speed);

            } else if (gamepad1.dpad_right) { //used to be right
                frontLeftMotor.setPower(-1 / speed);
                backLeftMotor.setPower(1 / speed);
                frontRightMotor.setPower(-1 / speed);
                backRightMotor.setPower(1 / speed);

            } else {
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }

            /*
            if (drive == 0) {  //only possible when not driving forward
                if (gamepad1.dpad_right) {
                    frontLeftMotor.setPower(gamepad1.left_stick_y / speed);
                    backLeftMotor.setPower(-gamepad1.left_stick_y / speed);
                    frontRightMotor.setPower(gamepad1.right_stick_x / speed);
                    backRightMotor.setPower(-gamepad1.right_stick_x / speed);

                } else if (gamepad1.dpad_left) {
                    frontLeftMotor.setPower(-gamepad1.left_stick_y / speed);
                    backLeftMotor.setPower(gamepad1.left_stick_y / speed);
                    frontRightMotor.setPower(-gamepad1.right_stick_x / speed);
                    backRightMotor.setPower(gamepad1.right_stick_x / speed);

                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }
            }

             */


        // SPEEDS  //

        // REGULAR //
        if (gamepad1.a) {
            speed = 1.05;
        }

        // SLOW //
        if (gamepad1.b) {
            speed = 3;
        }

        // FAST //
        if (gamepad1.x) {
            speed = .25;
        }

        //  RESET ENCODERS  //
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            reset();
        }

        // CRANE MECHANISM WITH MOTOR //

//        if (gamepad1.dpad_up && (craneMotor.getCurrentPosition() < craneTop)) {
//            craneMotor.setPower(1);
//        } else if (gamepad1.dpad_down && (craneMotor.getCurrentPosition() > craneLow)) {
//            craneMotor.setPower(-1);
//
//        } else {
//            craneMotor.setPower(0);
//        }
//
//
//        // PICKUP MECHANISM //
//
//        if (gamepad1.left_bumper) {
//            pickupMotor.setPower(1);
//        } else if (gamepad1.right_bumper) {
//            pickupMotor.setPower(-1);
//        } else {
//            pickupMotor.setPower(0);
//        }


        // Turning 180//

        if (gamepad1.y) {
            turn(180);
        }
    }




    public void turn(double degrees){
        degrees = 19.75 * degrees; //9.5 old , 19 is too much
        if(degrees > 0) {
            while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees) {

                frontRightMotor.setPower(-.35);
                backRightMotor.setPower(-.35);
                frontLeftMotor.setPower(.35);
                backLeftMotor.setPower(.35);
            }
        }
        if(degrees < 0) {
            degrees*=-1;
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
        reset();
    }

    public void reset() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

}

