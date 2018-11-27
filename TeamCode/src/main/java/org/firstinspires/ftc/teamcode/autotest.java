package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="auto test", group="Linear Opmode")


public class autotest extends OpMode{


    private DcMotor frontLeftMotor =  null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private Servo servoMain = null;
    private DcMotor pickupMotor = null;

    //1 inch will equal ticks
    private int inchPer = 0; //set encoder ticks here for inches

    //1 cm will equal ticks
    private int cmPer = 0; //set encoder ticks here for inches

   // calling for position of motors

//    private int flPos = frontLeftMotor.getCurrentPosition();
//    private int blPos = backLeftMotor.getCurrentPosition();
//    private int frPos = frontRightMotor.getCurrentPosition();
//    private int brPos = backRightMotor.getCurrentPosition();
    private double speed = 1.25; //speed is initialy set to half the speed of bot
    private int target = 1400;

//    private double flPos = frontLeftMotor.getCurrentPosition();
//    private double blPos = backLeftMotor.getCurrentPosition();
//    private double frPos = frontRightMotor.getCurrentPosition();
//    private double brPos = backRightMotor.getCurrentPosition();
//    private double speed = 1.25; //speed is initialy set to half the speed of bot


    @Override
    public void init() {


        // Defining each device
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");
        servoMain = hardwareMap.get(Servo.class, "servoMain");

        //reverses the direction for motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //runs using encoders
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Telemetry for distance
        telemetry.addData("Status", "Initialized");
        telemetry.update();




//        telemetry.addData("Front Left", flPos);
//        telemetry.addData("Front Right", frPos);
//        telemetry.addData("Back Left", blPos);
//        telemetry.addData("Back Right ", brPos);
//        telemetry.update();

    }

    @Override
    public void loop() {
        telemetry.update();

        frontLeftMotor.setPower(gamepad1.left_stick_y / speed);
        backLeftMotor.setPower(gamepad1.left_stick_y/ speed);
        frontRightMotor.setPower(gamepad1.right_stick_y / speed);
        backRightMotor.setPower(gamepad1.right_stick_y /speed);

        frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);

        if (gamepad1.y && gamepad1.right_bumper) {
            craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        if (gamepad1.y) {

            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (frontLeftMotor.getCurrentPosition() < target) {
                frontLeftMotor.setPower(.25);
                frontRightMotor.setPower(.25);
                backRightMotor.setPower(.25);
                backLeftMotor.setPower(.25);
            }

        }

        if (gamepad1.x) {

            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }


        if (gamepad1.a) {
            speed = 1.25;
        }
        if (gamepad1.b) {
            speed = 4;
        }
telemetry.update();

        telemetry.addData("Front Left", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right", frontRightMotor.getCurrentPosition());
        telemetry.addData("Back Left", backLeftMotor.getCurrentPosition());
        telemetry.addData("Back Right ", backRightMotor.getCurrentPosition());
        telemetry.update();
    }
}
