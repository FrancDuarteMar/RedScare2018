package org.firstinspires.ftc.teamcode.AutoIMUTests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;



import java.util.Locale;
@Disabled
@TeleOp(name="Gyro Sensor Given", group=" Opmode")

public class gyroGiven extends OpMode {


// Initialize motors

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

   // private double heading = 0;

    //double angleError;
    //int nullValue;


    //setting limits
    private int craneTop = 2992;
    private int craneLow = 0;
    private int craneLim = 50;

    //var
    private double pos = -2;
    private double cent = 30.48;


    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();


    //VISION


    public void init(){

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


        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        composeTelemetry();

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        accel = imu.getGravity();

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void loop() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


       // double heading = angles.firstAngle;


        // Wait for the game to start (driver presses PLAY)
        //initVuforia();

        // waitForStart();
        //runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.update();


        if (gamepad1.b) {
            turn(90);
        }
        if (gamepad1.x) {

            turn(-90);

        }
        if (gamepad1.y) {
            turn(-180);

        }
        if (gamepad1.a) {
            turn(180);
        }

    }
        //turn(TYPE In DEGREES;
        //driveForward(TYPE IN CENTIMETERS);
        //driveBackward(TYPE IN CENTIMETERS IN -);


    //CRANE//

    public void drop() {
        while (craneMotor.getCurrentPosition() < craneTop ) {

//one if positive, the other negative. Test with "Main Test Bed" code
            craneMotor.setPower(.75);                            //for one motor named craneMotor
        }

        //stops the lift system
        craneMotor.setPower(0);
    }


    public void lift() {
        while (craneMotor.getCurrentPosition() > craneLim ) {

            craneMotor.setPower(-1);
        }

        craneMotor.setPower(0);
    }


    // RESET ENCODER //

    public void reset() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // SERVO //

    public void servoSet() {
        dumpServo.setPosition(.1);
    }


    // GYRO //

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            accel  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }

                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("deg", formatAngle(angles.angleUnit, angles.firstAngle))
//                .addData( "Deg2", AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
                .addData("Deg3",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);

               /* .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return accel.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(accel.xAccel*accel.xAccel
                                        + accel.yAccel*accel.yAccel
                                        + accel.zAccel*accel.zAccel));
                    }
                });
                */
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }



    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void getAngles() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }


    public void resetGyro() {
        imu.initialize(parameters);
    }
    public double getGyroYaw() {
        return (double) angles.firstAngle;
    }

    private void runMotors(double rP, double lP){
        frontLeftMotor.setPower(-lP);
        backLeftMotor.setPower(-lP);

        frontRightMotor.setPower(rP);
        backRightMotor.setPower(rP);
    }
    private void stopMotors() {
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void rotate(double pow, double angle) throws InterruptedException{
        resetGyro();

        double power = pow;
        double angleTurn = angle;
        double error;
        getAngles();
        double currentAngle = getGyroYaw();
        double prevError = angleTurn - currentAngle;

        while (Math.abs(currentAngle) < angleTurn - 2) {
            getAngles();
            currentAngle = getGyroYaw();
            error = angleTurn - Math.abs(currentAngle);
            power = (pow * (error) * .005) + .215;
            getAngles();
            if (power >1 ){
                power = 1;
            }
            if (power < -1){
                power= -1;
            }
            runMotors(-power,power);
            telemetry.addData("PID", power);
            telemetry.update();
            prevError = error;

        }
        telemetry.update();
        stopMotors();
    }
    public void pRotateNoReset(double pow, double angle) throws InterruptedException {
        //setting needed variables
        double power = pow;
        double angleTo = angle;
        double error;
        getAngles();
        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;
        //do while
        while (Math.abs(currentAngle) < angleTo - 2) {
            getAngles();                                //update angles
            currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
            error = angleTo - Math.abs(currentAngle);             //calculate error
            power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
            getAngles();
            if(power > 1) {                             //check to see power is legal amount
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }
            runMotors(-power, power);                 //set the motors to turn
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
































    public void driveForward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() < distance && frontRightMotor.getCurrentPosition() < distance) {

            frontRightMotor.setPower(.75);
            backRightMotor.setPower(.75);
            frontLeftMotor.setPower(.75);
            backLeftMotor.setPower(.75);
        }
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        reset();
    }


    public void driveBackward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() > distance && frontRightMotor.getCurrentPosition() > distance ) {

            frontRightMotor.setPower(-.75);
            backRightMotor.setPower(-.75);
            frontLeftMotor.setPower(-.75);
            backLeftMotor.setPower(-.75);
        }
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        reset();
    }


    // STRAFE // ** Might need to use * .707 to the dist **

    public void sLeft(double dist) {
        dist = (34.33 * dist);

        while ((frontLeftMotor.getCurrentPosition() > dist) && ((Math.abs(backRightMotor.getCurrentPosition())) > dist) ) {

            frontLeftMotor.setPower(.5);
            backRightMotor.setPower(-.5);
            frontRightMotor.setPower(.5);
            backRightMotor.setPower(-.5);
        }

        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public void sRight(double dist) {
        dist = (34.33 * dist);

        while (((Math.abs(frontLeftMotor.getCurrentPosition())) > dist) && (backRightMotor.getCurrentPosition() > dist) ) {

            frontLeftMotor.setPower(-.5);
            backLeftMotor.setPower(.5);
            frontRightMotor.setPower(-.5);
            backRightMotor.setPower(.5);
        }

        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    // TURN //

    public void turn(double angle) {
        resetGyro();
        double angleTo = angle;

        getAngles();

        double currentAngle = getGyroYaw();


        if (angleTo > 0) {
            while (currentAngle <angleTo) {

                frontRightMotor.setPower(-.35);
                backRightMotor.setPower(-.35);
                frontLeftMotor.setPower(.35);
                backLeftMotor.setPower(.35);
            }
        }
        if (angleTo < 0) {
            angleTo *= -1;
            while (currentAngle > angleTo) {

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
        resetGyro();
    }
}




























