package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveUtils {
    private static final DriveUtils ourInstance = new DriveUtils();
    private static final float DISTANCE_CONVERSION = 100.0f / 3;

    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    public static DriveUtils getInstance() {
        return ourInstance;
    }

    private DriveUtils() {
    }

    public void init(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;
    }

    public void driveForward(float distanceInCentimeters) {
        int initialPosition = leftFrontMotor.getCurrentPosition();
        while (leftFrontMotor.getCurrentPosition() - initialPosition < distanceInCentimeters * DISTANCE_CONVERSION) {
            go(1);
        }
        stop();
    }

    public void driveBackward(float distanceInCentimeters) {
        int initialPosition = leftFrontMotor.getCurrentPosition();
        while (leftFrontMotor.getCurrentPosition() - initialPosition < distanceInCentimeters * DISTANCE_CONVERSION) {
            go(-1);
        }
        stop();
    }

    public void turnLeft(float degrees) {
        // Calculate distance based on degrees

    }

    public void turnRight(float degrees) {
        // Calculate distance based on degrees

    }

    public void go(float power) {
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
