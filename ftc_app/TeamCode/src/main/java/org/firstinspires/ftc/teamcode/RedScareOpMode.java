package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp(name="Red Scare Op Mode", group="Red Scare")
public class RedScareOpMode extends OpMode {

    private DcMotor frontLeftMotor =  null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private DcMotor pickupMotor = null;




    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData( "encodervalue:", craneMotor.getCurrentPosition());
        telemetry.update();


    }

    @Override
    public void loop() {


        //sticks controlling driving code

        //left stick going backwards by pushing forward
        if (gamepad1.left_stick_y < 0) {
            frontLeftMotor.setPower(1);
            backLeftMotor.setPower(1);
        }
        //right stick going backwards by pushing backward
        if (gamepad1.right_stick_y < 0) {
            frontRightMotor.setPower(1);
            backRightMotor.setPower(1);
        }
        //left stick going forwards
        if (gamepad1.left_stick_y > 0) {
            frontLeftMotor.setPower(-1);
            backLeftMotor.setPower(-1);
        }
        //right stick going forwards
        if (gamepad1.right_stick_y > 0) {
            frontRightMotor.setPower(-1);
            backRightMotor.setPower(-1);
        }
        //left stick at zero
        if (gamepad1.left_stick_y == 0) {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }
        //right stick at zero
        if (gamepad1.right_stick_y == 0) {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }




        //lifting mechanism with Bumpers

        //right bumper goes up
        if (gamepad1.dpad_up) {
            craneMotor.setPower(-.5);
        }
        //left bumper goes down
        else if (gamepad1.dpad_down) {
            craneMotor.setPower(.5);
        }
        //no bumpers being pushed
        else {
            craneMotor.setPower(0);
        }





        //pickup Mechanism with Dpad

        //dpad up initiates pickup mechanism
        if (gamepad1.right_bumper) {
            pickupMotor.setPower(.5);
        }
        //dpad down releases the balls

        else if (gamepad1.left_bumper) {
            pickupMotor.setPower(-.5);
    }
        //no pressing, stops all movement
        else {
            pickupMotor.setPower(0);
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



    }
}
