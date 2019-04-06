package org.firstinspires.ftc.teamcode.AutoIMUTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
@TeleOp(name="Auto Gyro Turn Test  ", group="Red Scare")
public class gyroAuto extends OpMode  {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Servo dumpServo;
    DcMotor craneMotor;


    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;
    Acceleration accel;

    int nullValue;

    double angleError;





    public void init() {

        backLeftMotor = hardwareMap.dcMotor.get("Back left motor");
        backRightMotor = hardwareMap.dcMotor.get("Back right motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front left motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        dumpServo = hardwareMap.get(Servo.class, "servoMain");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


       // frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


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


        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        composeTelemetry();

        nullValue = 0;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        accel = imu.getGravity();


    }

    public void loop(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.update();

        if (gamepad1.b) {
            pRotate(.5,90);
        }
        if (gamepad1.x) {

            pRotate(.5,-90);

        }
        if (gamepad1.y) {

            pRotate(.5,-180);
        }
        if (gamepad1.a) {
        pRotate(.5,180);
        }
    }




        public void startMotors ( double rP, double lP){
        backLeftMotor.setPower(-lP);
        backRightMotor.setPower(rP);
        frontLeftMotor.setPower(-lP);
        frontRightMotor.setPower(rP);
    }

        public void stopMotors () {
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }


        //P loop for PID
        public void pRotate ( double pow, double angle){
        resetGyro();

        double power = pow;
        double angleTo = angle;
        double error;

        getAngles();

        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;

        while ((Math.abs(currentAngle)) < angleTo ) {
            getAngles();                                //update angles
            currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if (power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if (power < -1) {
                power = -1;
            }

            startMotors(-power, power);                 //set the motors to turn
            telemetry.addData("PID", power);
            telemetry.update();
            previousError = error;
        }
        telemetry.update();
        stopMotors();                                  //stop motion
        //  Double d = angle;
        //  int rotated = d.intValue();
        //  Double ticks = rotated / 90.0;
        // int ticksI = ticks.intValue();
//      updateFacing(ticksI);
    }

        public void pRotateNoReset ( double pow, double angle){
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        double error;
        getAngles();
        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;
        //do while
        while (Math.abs(currentAngle) < angleTo - 2 ) {
            getAngles();                                //update angles
            currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if (power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if (power < -1) {
                power = -1;
            }
            startMotors(-power, power);                 //set the motors to turn
            telemetry.addData("PID", power);
            telemetry.update();
            previousError = error;

        }
        telemetry.update();
        stopMotors();                                  //stop motion
//        Double d = angle;
//        int rotated = d.intValue();
//        Double ticks = rotated / 90.0;
//        int ticksI = ticks.intValue();
//        updateFacing(ticksI);
    }


        public int getEncoderAvg () {
        return ((Math.abs(backLeftMotor.getCurrentPosition())) + (Math.abs(backRightMotor.getCurrentPosition()))
                + (Math.abs(frontLeftMotor.getCurrentPosition())) + (Math.abs(frontRightMotor.getCurrentPosition())))
                / 4;
    }
        public void setNullValue () {
        nullValue = getEncoderAvg();
    }

        public void getAngles () {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }


        public double getGyroYaw () {
        return (double) angles.firstAngle;
    }

        public void resetGyro () {
        imu.initialize(parameters);
    }

        public void composeTelemetry () {

        telemetry.addLine()
                .addData("yaw", new Func<String>() {
                    @Override
                    public String value() {
                        return angles.firstAngle + "";
                    }
                });

        telemetry.addLine()
                .addData("encoder", new Func<String>() {
                    @Override
                    public String value() {
                        return (getEncoderAvg() - nullValue) + "";
                    }
                });
    }

    }

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE


