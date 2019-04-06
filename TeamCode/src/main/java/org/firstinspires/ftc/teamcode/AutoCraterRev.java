package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name=" Auto Crater Start w/ REV IMU", group="Red Scare ")
public class AutoCraterRev extends LinearOpMode  {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Servo dumpServo;
    Servo tokenServo;
    DcMotor craneMotor;


    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;
    Acceleration accel;

    double tp = .75;
    int nullValue;

    //setting limits
    private int craneTop = 2200 ; //4000 // new rev 2500; old 3000
    private int craneLim = 200;

    private int liftTop = -2000; //used to be -2919
    private int liftBottom = -100;


    private double pos = -2;
    private double cent = 30.48;

    boolean start;

    double angleError;

    private ElapsedTime runtime = new ElapsedTime();

    //VISION

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " ARCKAZn/////AAABmdrafuTjwkiSloOYXfj9e1IgxCaAOM9hmza6aoD+C6sEuz1p3aVtrGJvz6n1vJlN2QLnZukpt/Qp11mJwl/qwapHeZyFFYLNGHSRRiEKLeYV7tz/JJJicVhCyK3PjS/1DCjrzOEaC1qhfwQX0vBvmsQ7vebdP5VHmH7Ei36OIjuVKHAFUoQ/9x6uTvq/p5AujVPvjx4I08H+ss9thfQ0dpXgAPm62RW3w141csoGkIPtaTpPkfhJdzLBuDopIRwNeUmZX3GHGwCZA70j6GxHJD0vTL0WUGMdUPtr3PjgmnmxBwMU17+uHL1o8qvOY1Jv26KkV9Z2BJFCfxu+Aj3sKcGibt4VqPJ8EGNQarJvh6Ds ";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void runOpMode(){

        // Initializing and maping hardware
        backLeftMotor = hardwareMap.dcMotor.get("Back left motor");
        backRightMotor = hardwareMap.dcMotor.get("Back right motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front left motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "Crane Motor");

//        dumpServo = hardwareMap.get(Servo.class, "Dump Servo");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Encoder Setup
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

        craneMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //IMU Parameters
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        composeTelemetry();


        start= false;
        craneHold(.8);
        tokenServo.setPosition(0);

        initVuforia();

        waitForStart();
        start = true;
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drop(.8);
        driveBackward(-10);
        lift();
        driveForward(10);
        vision();


        // RIGHT //
        if(pos == 1) {
            pRotate(.5, 110);
            driveForward(1.6*cent);
            driveBackward(-1.6*cent);
            pRotate(.5, -25);
            depotDrive();
        }
        // CENTER //
        else if (pos == 0) {
            pRotate(.5, 85); // initial rotate to drive forward
            driveForward(3*cent);
            driveBackward(-3 * cent);
            depotDrive();
        }
        // LEFT//
        else {
            pRotate(.5, 60);
            driveForward(1.6*cent);
            driveBackward(-1.6*cent);
            pRotate(.5, 25);
            depotDrive();

        }


    }



    // PID MOVEMENTS //
//    public void moveBackwardPID(double pow, double dist)  {
//
//        resetGyro();
//        double angle;
//        telemetry.update();
//
//        double encDist = 34.3333 * dist;
//        double error;
//        double power;
//        setNullValue();
//
//        int currentEncoder = getEncoderAvg() - nullValue;
//        while (encDist < currentEncoder && opModeIsActive()) {
//            telemetry.update();
//            getAngles();
//            angle = getGyroYaw();
//
//            currentEncoder = getEncoderAvg() - nullValue;
//
//            error = (double) (encDist - currentEncoder) / encDist;
//
//            power = (pow * error) + .25;
//
//            Range.clip(power, -1, 1);
//
//            telemetry.addData("Power", power);
//            telemetry.addData("LeftPower", backLeftMotor.getPower());
//            telemetry.addData("RightPower", backRightMotor.getPower());
//
//            if (angle > 2) {
//                startMotors((-power * .75), -power);
//                telemetry.update();
//                telemetry.addData("LeftPower", backLeftMotor.getPower() + "");
//                telemetry.addData("RightPower", backRightMotor.getPower() + "");
//            } else if (angle < -2) { //if off to the right, correct
//                startMotors(-power, (-power * .75));
//                telemetry.addData("LeftPower", backLeftMotor.getPower() + "");
//                telemetry.addData("RightPower", backRightMotor.getPower() + "");
//                telemetry.update();
//            } else { //if heading is fine keep moving straight
//                startMotors(-power, -power);
//                telemetry.update();
//                telemetry.addData("LeftPower", backLeftMotor.getPower() + "");
//                telemetry.addData("RightPower", backRightMotor.getPower() + "");
//
//            }
//            idle();
//        }
//        stopMotors();
//        telemetry.update();
//        angleError = getGyroYaw();
//    }





    // MOTOR MOVEMENTS //
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





    // TURNING WITH IMU //
    public void pRotate ( double pow, double angle){
        resetGyro();

        double power = pow;
        double angleTo = angle;
        double error;

        getAngles();

        double currentAngle = getGyroYaw();
        double previousError = angleTo - currentAngle;
        if (angleTo > 0) {
            while ((Math.abs(currentAngle)) < angleTo && opModeIsActive()) {
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
                idle();
            }
        }

        if (angleTo <0){
            while (currentAngle < (Math.abs(angleTo)) && opModeIsActive()) {
                getAngles();                                //update angles
                currentAngle = getGyroYaw();                 //set the currentAngle to angle returned from gyro
                error = Math.abs(angleTo) - currentAngle;             //calculate error
                power = (pow * (error) * .005) + .215;               //set the power based on distance from goal (3-13-16 constant = .015)
                getAngles();
                if (power > 1) {                             //check to see power is legal amount
                    power = 1;
                }
                if (power < -1) {
                    power = -1;
                }

                startMotors(power, -power);                 //set the motors to turn
                telemetry.addData("PID", power);
                telemetry.update();
                previousError = error;
                idle();
            }
        }
        telemetry.update();
        // stopMotors();                                  //stop motion
        //  Double d = angle;
        //  int rotated = d.intValue();
        //  Double ticks = rotated / 90.0;
        // int ticksI = ticks.intValue();
//      updateFacing(ticksI);
    }





    // IMU FUNCTIONS //
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





    // VISION //
    public void vision() {


        if (tfod != null) {
            tfod.shutdown();
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }


            if (tfod != null) {
                while (pos == -2 && opModeIsActive()) {



                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    pos = -1;
                                    tfod.shutdown();



                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    pos = 1;
                                    tfod.shutdown();


                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    pos = 0;
                                    tfod.shutdown();



                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }





    // INITIAL LIFT MECH //
    public void drop(double speed) {
        while (craneMotor.getCurrentPosition() < craneTop && opModeIsActive()) {

//one if positive, the other negative. Test with "Main Test Bed" code
            craneMotor.setPower(speed);                            //for one motor named craneMotor
        }

        //stops the lift system
        craneMotor.setPower(0);

    }


    public void craneHold(double speed){
        pos = 0;
        while (start == false && opModeIsActive()){
            craneMotor.setTargetPosition(0);
            craneMotor.setPower(speed);
        }
        start = true;
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void lift() {
        while (craneMotor.getCurrentPosition() > craneLim && opModeIsActive()) {

            craneMotor.setPower(-1);
        }

        craneMotor.setPower(0);
    }





    //REGULAR DRIVING //
    public void driveForward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() < distance && frontRightMotor.getCurrentPosition() < distance && opModeIsActive()) {

            frontRightMotor.setPower(.5);
            backRightMotor.setPower(.5);
            frontLeftMotor.setPower(-.5);
            backLeftMotor.setPower(-.5);
        }
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        reset();
    }

    public void driveBackward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() > distance && frontRightMotor.getCurrentPosition() > distance && opModeIsActive()) {

            frontRightMotor.setPower(-.5);
            backRightMotor.setPower(-.5);
            frontLeftMotor.setPower(.5);
            backLeftMotor.setPower(.5);
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
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }





    // END //
    public void servoSet() {
        tokenServo.setPosition(1 );
        idle();
    }

    public void depotDrive(){
        pRotate(5, -50);
        driveForward(3.25*cent);
        pRotate(.5,-85);
        driveForward( 6* cent);
        servoSet();
        driveBackward(-7*cent);
    }


}

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE


