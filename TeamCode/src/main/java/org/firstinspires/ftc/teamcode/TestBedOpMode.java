package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class TestBedOpMode extends OpMode {

    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private static final float MOTOR_SPEED_BIAS = 1;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftFrontMotor.setPower(gamepad1.left_stick_y * MOTOR_SPEED_BIAS);
        leftBackMotor.setPower(gamepad1.left_stick_y * MOTOR_SPEED_BIAS);
        rightFrontMotor.setPower(gamepad1.right_stick_y * MOTOR_SPEED_BIAS);
        rightBackMotor.setPower(gamepad1.right_stick_y * MOTOR_SPEED_BIAS);
    }
}
