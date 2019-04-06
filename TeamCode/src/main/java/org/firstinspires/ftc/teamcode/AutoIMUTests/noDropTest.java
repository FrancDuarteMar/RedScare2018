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
import com.qualcomm.robotcore.util.ElapsedTime;
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

@Disabled
@Autonomous(name="No Drop Gyro Test  ", group="Red Scare")
public class noDropTest extends LinearOpMode  {

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

    double tp = .75;
    int nullValue;

    //setting limits
    private int craneTop = 2100 ; //4000 // new rev 2500; old 3000
    private int craneLim = 200;

    private int liftTop = -2500; //used to be -2919
    private int liftBottom = -200;

    private double pos = -2;
    private double cent = 30.48;


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

        backLeftMotor = hardwareMap.dcMotor.get("Back left motor");
        backRightMotor = hardwareMap.dcMotor.get("Back right motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front left motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        dumpServo = hardwareMap.get(Servo.class, "servoMain");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


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



        initVuforia();



        waitForStart();

        reset();
        runtime.reset();

        //dumpServo.setPosition(.75);
      //  drop();
        driveForward(10);
    //    lift();
        driveBackward(-10);
        vision();

        if (pos == 1) { //RIGHT
            pRotate(.75,-90);
            driveForward(1.2 * cent);
            pRotate(-.75, 45);
            driveForward( 4 * cent);
            pRotate(.75,-90);
            driveForward(3.7 * cent);
            servoSet();
            pRotate(.75,-10);
//            driveForward(8.5 * cent);
            driveBackward(-8.5 * cent);
        } else if (pos == 0) { //CENTER
            pRotate(.75,-90);
            driveForward(5.75 * cent);
            pRotate(.75,40);
            //driveForward(.2 * cent);
            servoSet();
            // turn(10);
            driveBackward(-8 * cent);

        } else { //LEFT
            pRotate(.75,-90);
            driveForward(1.2 * cent);        //goes forward
            pRotate(.75,-45);
            driveForward(3 * cent);         //knocks mineral
            pRotate(.75,95);
            driveForward(4 * cent); //goes into depot
            //turn(30 );
            //driveForward(80);
            //turn(20);
            servoSet(); //drops token
            //turn(-90);
            //driveForward(8.5);
            driveBackward(-8.5 * cent); //goes backward into crater

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
        if (angleTo > 0) {
            while ((Math.abs(currentAngle)) < angleTo) {
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
        }

        if (angleTo <0){
            while (currentAngle < (Math.abs(angleTo))) {
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
            }
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

                    CameraDevice.getInstance().setFlashTorchMode(true);

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
                                    CameraDevice.getInstance().setFlashTorchMode(false);


                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    pos = 1;
                                    tfod.shutdown();
                                    CameraDevice.getInstance().setFlashTorchMode(false);

                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    pos = 0;
                                    tfod.shutdown();
                                    CameraDevice.getInstance().setFlashTorchMode(false);


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

    public void drop() {
        while (craneMotor.getCurrentPosition() < craneTop && opModeIsActive()) {

//one if positive, the other negative. Test with "Main Test Bed" code
            craneMotor.setPower(.75);                            //for one motor named craneMotor
        }

        //stops the lift system
        craneMotor.setPower(0);

    }

    public void lift() {
        while (craneMotor.getCurrentPosition() > craneLim && opModeIsActive()) {

            craneMotor.setPower(-1);
        }

        craneMotor.setPower(0);
    }

    public void driveForward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() < distance && frontRightMotor.getCurrentPosition() < distance && opModeIsActive()) {

            frontRightMotor.setPower(.75);
            backRightMotor.setPower(.75);
            frontLeftMotor.setPower(-.75);
            backLeftMotor.setPower(-.75);
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

            frontRightMotor.setPower(-.75);
            backRightMotor.setPower(-.75);
            frontLeftMotor.setPower(.75);
            backLeftMotor.setPower(.75);
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

    public void servoSet() {
        dumpServo.setPosition(.1);
    }



}

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE


