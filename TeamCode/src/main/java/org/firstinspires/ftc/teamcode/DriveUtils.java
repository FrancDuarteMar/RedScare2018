package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.DriveUtils.DriveSpeed.FULL;
import static org.firstinspires.ftc.teamcode.DriveUtils.DriveSpeed.HALF;

public class DriveUtils {
    private static final DriveUtils ourInstance = new DriveUtils();
    private static final float DISTANCE_CONVERSION = 100.0f / 3;
    private static final float TURN_DISTANCE_CONVERSION = 9.8f;

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
        driveForward(distanceInCentimeters, HALF);
    }

    public void driveForward(float distanceInCentimeters, DriveSpeed driveSpeed) {
        while (leftFrontMotor.getCurrentPosition() < distanceInCentimeters * DISTANCE_CONVERSION) {
            float power = getDriveSpeedPower(driveSpeed);
            leftFrontMotor.setPower(power);
            leftBackMotor.setPower(power);
            rightFrontMotor.setPower(power);
            rightBackMotor.setPower(power);
        }
        stop();
    }

    public void driveBackward(float distanceInCentimeters) {
        driveBackward(distanceInCentimeters, HALF);
    }

    public void driveBackward(float distanceInCentimeters, DriveSpeed driveSpeed) {
        while (leftFrontMotor.getCurrentPosition() > distanceInCentimeters * DISTANCE_CONVERSION) {
            float power = getDriveSpeedPower(driveSpeed);
            leftFrontMotor.setPower(-power);
            leftBackMotor.setPower(-power);
            rightFrontMotor.setPower(-power);
            rightBackMotor.setPower(-power);
        }
        stop();
    }

    public void turnLeft(float degrees) {
        turnLeft(degrees, HALF);
    }

    public void turnLeft(float degrees, DriveSpeed driveSpeed) {
        // Calculate distance based on degrees
        float distance = degrees * TURN_DISTANCE_CONVERSION;
        float power = getDriveSpeedPower(driveSpeed);

        while (rightFrontMotor.getCurrentPosition() < distance) {
            leftFrontMotor.setPower(-power);
            leftBackMotor.setPower(-power);
            rightFrontMotor.setPower(power);
            rightFrontMotor.setPower(power);
        }
        stop();
    }

    public void turnRight(float degrees) {
        turnRight(degrees, HALF);
    }

    public void turnRight(float degrees, DriveSpeed driveSpeed) {
        // Calculate distance based on degrees
        float distance = degrees * TURN_DISTANCE_CONVERSION;
        float power = getDriveSpeedPower(driveSpeed);

        while (leftFrontMotor.getCurrentPosition() < distance) {
            leftFrontMotor.setPower(power);
            leftBackMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            rightFrontMotor.setPower(-power);
        }
        stop();

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
        resetEncoders();
    }

    private float getDriveSpeedPower(DriveSpeed driveSpeed) {
        if (driveSpeed == FULL) {
            return 1.0f;
        } else if (driveSpeed == HALF) {
            return 0.5f;
        } else {
            return 0.25f;
        }
    }

    enum DriveSpeed {
        FULL, HALF, PRECISE
    }
}
