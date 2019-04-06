package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Timer;
import java.util.TimerTask;


@TeleOp(name="Fast Bot ", group="A")
public class fastBot extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    Servo directionServo;

    double servoPos = 0;

    double speed = .5;


    //setting limits for robot lift/crane



    @Override
    public void init() {
        // Initializing and maping hardware
        backRightMotor = hardwareMap.dcMotor.get("Back Right Motor");
        backLeftMotor = hardwareMap.dcMotor.get("Back Left Motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front Left Motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front Right Motor");



        directionServo = hardwareMap.servo.get("Direction Servo");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); //used to be front left now is mapped to back left through different name
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        telemetry.update();

        /* DRIVING */

        servoPos = gamepad1.right_stick_x;

        directionServo.setPosition(servoPos);

        frontLeftMotor.setPower(gamepad1.left_stick_y * speed);
        frontRightMotor.setPower(gamepad1.left_stick_y * speed);

        backLeftMotor.setPower(gamepad1.left_stick_y * speed);
        backRightMotor.setPower(gamepad1.left_stick_y * speed);

        if (gamepad1.a) {
            speed = .5;
        }

        if (gamepad1.x){
            speed = 1;
        }







        if (gamepad1.dpad_up){
            frontRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(1);

//            rightMotor.setPower(1);

        }

        else if (gamepad1.dpad_down ){
            frontRightMotor.setPower(-1);
            frontLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);//            rightMotor.setPower(-1);
        }
        else{
            stopMotors();
        }
    }
    public void stopMotors (){
//        rightMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);//
    }


//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

}

