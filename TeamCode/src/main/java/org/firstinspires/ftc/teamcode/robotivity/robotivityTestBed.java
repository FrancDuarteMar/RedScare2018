package org.firstinspires.ftc.teamcode.robotivity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Timer;
import java.util.TimerTask;

@Disabled
@TeleOp(name="Robotivity Test Bed  ", group="A")
public class robotivityTestBed extends OpMode {

    DcMotor backLeftMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontRightMotor;

    Servo directionServo;

    double servoPos = 0;

    double speed = .5;


    //setting limits for robot lift/crane


    @Override
    public void init() {
        // Initializing and maping hardware
        backLeftMotor = hardwareMap.dcMotor.get("Back Left Motor");
        backRightMotor = hardwareMap.dcMotor.get("Back Right Motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front Left Motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front Right Motor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE); //used to be front left now is mapped to back left through different name
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        telemetry.update();


        if (gamepad1.a) {
            speed = .5;
        }

        if (gamepad1.x) {
            speed = 1;
        }

        frontLeftMotor.setPower(gamepad1.left_stick_y);
        backLeftMotor.setPower(gamepad1.left_stick_y);

        frontRightMotor.setPower(gamepad1.right_stick_y);
        backRightMotor.setPower(gamepad1.right_stick_y);

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

    }
}

