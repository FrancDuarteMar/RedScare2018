package org.firstinspires.ftc.teamcode.AutoIMUTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="Gyro Tele ", group="Red Scare")
public class teleGyro extends OpMode {

    BNO055IMU imu;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor craneMotor;
    DcMotor liftMotor;
    Servo dumpServo;

    //var
    private double pos = -2;
    private double cent = 30.48;


    private float drive = 0;
    private double speed = 1.05;


    //setting limits
    private int craneTop = 2100 ; //4000 // new rev 2500; old 3000
    private int craneLow = 200;

    private int liftTop = -2500; //used to be -2919
    private int liftBottom = -200;

    // GYRO \\
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    Orientation angles;


    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");

        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        dumpServo = hardwareMap.get(Servo.class, "servoMain");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


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


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.update();

        telemetry.addData("Mode", "running");
        telemetry.update();

        correction = checkDirection();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();


        if (gamepad1.b) {

            turn(90   );
        }

        if (gamepad1.y ) {
            turn(-90);
        }

        if (gamepad1.left_bumper) {
            turn(180);
        }

        if (gamepad1.right_bumper) {
            turn(-180);
        }







/*
        /* DRIVING


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

        //proportional strafe
        //set value strafe
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
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
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


        // SERVO TEST //
        if (gamepad1.x) {
            dumpServo.setPosition(1);
        }
        if (gamepad1.a && gamepad1.b){
            dumpServo.setPosition(0);
        }


        // PICKUP MECHANISM //

        if (gamepad1.dpad_down && (craneMotor.getCurrentPosition() > craneLow)) {
            craneMotor.setPower(-1);
            telemetry.addData("lift encodervalue:", craneMotor.getCurrentPosition());
            telemetry.update();

        } else if (gamepad1.dpad_up && (craneMotor.getCurrentPosition() < craneTop )) {
            craneMotor.setPower(1);
            telemetry.addData("lift encodervalue:", craneMotor.getCurrentPosition());
            telemetry.update();

        } else {
            craneMotor.setPower(0);
            telemetry.addData("lift encodervalue:", craneMotor.getCurrentPosition());
            telemetry.update();

        }



        if (gamepad1.left_bumper && (liftMotor.getCurrentPosition()< liftBottom)) {
            liftMotor.setPower(1/speed);
            telemetry.addData("elevator encodervalue:", liftMotor.getCurrentPosition());
            telemetry.update();
        } else if (gamepad1.right_bumper && (liftMotor.getCurrentPosition()> liftTop)) {
            liftMotor.setPower(-1/speed);
            telemetry.addData("elevator encodervalue:", liftMotor.getCurrentPosition());
            telemetry.update();
        } else {
            liftMotor.setPower(0);
            telemetry.addData("elevator encodervalue:", liftMotor.getCurrentPosition());
            telemetry.update();
        }




        if (gamepad1.y) {
            turn(180);
        }
*/
    }




    public void turn(double deg){
        //degrees = 19.75 * degrees; //9.5 old , 19 is too much
        if(deg > 0) {
            while (getGyroYaw() < deg) {

                frontRightMotor.setPower(-.35);
                backRightMotor.setPower(-.35);
                frontLeftMotor.setPower(.35);
                backLeftMotor.setPower(.35);
            }
        }
        if(deg < 0) {
            deg*=-1;
            while (getGyroYaw()  > deg) {

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


    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

         if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
//        frontLeftMotor.setPower(leftPower);
//        backLeftMotor.setPower(leftPower);
//
//        frontRightMotor.setPower(rightPower);
//        backRightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {
            }

            while ( getAngle() > degrees) {
            }
        }
        else    // left turn.
            while ( getAngle() < degrees) {
            }

        // turn the motors off.
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);

        // reset angle tracking on new heading.
        resetAngle();
    }

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

    public double getGyroYaw() {
        return (double) angles.firstAngle;
    }
}

