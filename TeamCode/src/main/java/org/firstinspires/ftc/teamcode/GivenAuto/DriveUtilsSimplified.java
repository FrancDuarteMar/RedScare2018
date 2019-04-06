package org.firstinspires.ftc.teamcode.GivenAuto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class DriveUtilsSimplified {
    private static final DriveUtilsSimplified ourInstance = new DriveUtilsSimplified();

    private static final float DISTANCE_CONVERSION = 100.0f / 3;
    private static final float TURN_DISTANCE_CONVERSION = 9.5f;

    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    public static DriveUtilsSimplified getInstance() {
        return ourInstance;
    }

    private DriveUtilsSimplified() {
    }

    public void init(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;

        resetEncoders();
    }

    private void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(float distanceInCentimeters) {
        while (leftFrontMotor.getCurrentPosition() < distanceInCentimeters * DISTANCE_CONVERSION) {
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(1);
            rightFrontMotor.setPower(1);
            rightBackMotor.setPower(1);
        }
        stop();
    }

    public void driveBackward(float distanceInCentimeters) {
        while (leftFrontMotor.getCurrentPosition() > distanceInCentimeters * DISTANCE_CONVERSION) {
            leftFrontMotor.setPower(-1);
            leftBackMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
            rightBackMotor.setPower(-1);
        }
        stop();
    }

    public void turnLeft(float degrees) {
        // Calculate distance based on degrees
        float distance = degrees * TURN_DISTANCE_CONVERSION;

        while (rightFrontMotor.getCurrentPosition() < distance) {
            leftFrontMotor.setPower(-0.5);
            leftBackMotor.setPower(-0.5);
            rightFrontMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
        }
        stop();
    }

    public void turnRight(float degrees) {
        // Calculate distance based on degrees
        float distance = degrees * TURN_DISTANCE_CONVERSION;

        while (leftFrontMotor.getCurrentPosition() < distance) {
            leftFrontMotor.setPower(0.5);
            leftBackMotor.setPower(0.5);
            rightFrontMotor.setPower(-0.5);
            rightFrontMotor.setPower(-0.5);
        }
        stop();

    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        resetEncoders();
    }
}
