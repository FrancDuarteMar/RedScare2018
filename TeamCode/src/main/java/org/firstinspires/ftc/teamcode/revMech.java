package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@TeleOp(name="Rev Mech TeleOp ", group="Red Scare")
public class revMech extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor liftMotor;
    DcMotor craneMotor;
    DcMotor intakeMotor;
    DcMotor dumpMotor;
  //  Servo dumpServo;
    Servo tokenServo;


    int dumpTop = -122;
    private float drive = 0;
    private double speed = 1.05;


    //setting limits for robot lift/crane
    private int craneTop = 2200 ;      //Luke 2860                                         //4000 // new rev 2500; old 3000
    private int craneLow = 200;

    private int liftTop = -2507; //luke: -2428                                                                //used to be -2919
    private int liftBottom = -100;




    @Override
    public void init() {
        // Initializing and maping hardware
        backLeftMotor = hardwareMap.dcMotor.get("Back left motor");
        backRightMotor = hardwareMap.dcMotor.get("Back right motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front left motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "Crane Motor");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift Motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        dumpMotor = hardwareMap.get(DcMotor.class, "Dump Motor");


//        dumpServo = hardwareMap.get(Servo.class, "Dump Servo");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Crane encodervalue:", craneMotor.getCurrentPosition());
        telemetry.update();
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
                stopMotors();
            }
        }

        // SPEEDS  //

        // REGULAR //
        if (gamepad1.a) {
            speed = 1.05;
        }

        // SLOW //
        if (gamepad1.b) {
            speed = 3;
        }



        // Dump Minerals Servo //
        if (gamepad1.x) {
            dumpMotor.setTargetPosition(dumpTop);
            dumpMotor.setPower(.3);
        }
        if (gamepad1.y){
            dumpMotor.setTargetPosition(0);
            dumpMotor.setPower(.3);

        }


        // Crane MECHANISM //

        if (gamepad1.dpad_up && (craneMotor.getCurrentPosition() > craneLow)) {
            craneMotor.setPower(-1);

        } else if (gamepad1.dpad_down && (craneMotor.getCurrentPosition() < craneTop )) {
            craneMotor.setPower(1);

        } else {
            craneMotor.setPower(0);


        }



        // Mineral Lift //
        if (gamepad1.left_bumper && (liftMotor.getCurrentPosition()< liftBottom)) {
            liftMotor.setPower(1/speed);
        } else if (gamepad1.right_bumper && (liftMotor.getCurrentPosition()> liftTop)) {
            liftMotor.setPower(-1/speed);
        } else {
            liftMotor.setPower(0);
        }


        // Mineral Intake //
        if (gamepad1.left_trigger > 0){
            intakeMotor.setPower(1);
        }
        if (gamepad1.right_trigger > 0){
            intakeMotor.setPower(-1);
        }
        if ((gamepad1.left_trigger == 0 ) && (gamepad1.right_trigger == 0)){
            intakeMotor.setPower(0);
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

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void stopMotors (){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }


//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

}

