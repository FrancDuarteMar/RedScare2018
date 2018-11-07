package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Test OpMode", group="Iterative Opmode")
public class TestBed extends OpMode {

    private DcMotor backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;

    private static final float MOTOR_SPEED_BIAS = 1;

    @Override
    public void init() {
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        backLeftMotor.setPower(gamepad1.left_stick_y * MOTOR_SPEED_BIAS);
        frontLeftMotor.setPower(gamepad1.left_stick_y * MOTOR_SPEED_BIAS);
        backRightMotor.setPower(gamepad1.right_stick_y * MOTOR_SPEED_BIAS);
        frontRightMotor.setPower(gamepad1.right_stick_y * MOTOR_SPEED_BIAS);
    }
}
