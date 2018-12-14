
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Auto Mech Depot Theory", group="Linear Opmode")

public class autoDepotTheory extends LinearOpMode {


// Initialize motors

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private Servo dumpServo = null;
    private DcMotor pickupMotor = null;


    //setting limits
    private int craneTop = 2992;
    private int craneLow = 0;
    private int craneLim = 50;

    //var
    private double pos = -2;
    private double cent = 30.48;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //VISION

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " ARCKAZn/////AAABmdrafuTjwkiSloOYXfj9e1IgxCaAOM9hmza6aoD+C6sEuz1p3aVtrGJvz6n1vJlN2QLnZukpt/Qp11mJwl/qwapHeZyFFYLNGHSRRiEKLeYV7tz/JJJicVhCyK3PjS/1DCjrzOEaC1qhfwQX0vBvmsQ7vebdP5VHmH7Ei36OIjuVKHAFUoQ/9x6uTvq/p5AujVPvjx4I08H+ss9thfQ0dpXgAPm62RW3w141csoGkIPtaTpPkfhJdzLBuDopIRwNeUmZX3GHGwCZA70j6GxHJD0vTL0WUGMdUPtr3PjgmnmxBwMU17+uHL1o8qvOY1Jv26KkV9Z2BJFCfxu+Aj3sKcGibt4VqPJ8EGNQarJvh6Ds ";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        pickupMotor = hardwareMap.get(DcMotor.class, " pickupMotor");
        dumpServo = hardwareMap.get(Servo.class, "servoMain");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickupMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        initVuforia();

        waitForStart();
        runtime.reset();


        //turn(TYPE In DEGREES;
        //driveForward(TYPE IN CENTIMETERS);
        //driveBackward(TYPE IN CENTIMETERS IN -);

        //ACROSS = 2.82


        dumpServo.setPosition(1);
        drop();
        driveForward(10);
        lift();
        driveBackward(-10);
        vision();

        if (pos == 1) { //RIGHT
            turn(-90);
            driveForward(1.2 * cent);
            turn(45);
            driveForward(3 * cent);
            turn(-90);
            driveForward(3.7 * cent);
            servoSet();
//            driveForward(8.5 * cent);
            driveBackward(-8.5 * cent);
        } else if (pos == 0) { //CENTER
            turn(-90);
            driveForward(5.75 * cent);
            turn(40);
            //driveForward(.2 * cent);
            servoSet();
            // turn(10);
            driveBackward(-8 * cent);

        } else { //LEFT
            turn(-90);
            driveForward(1.2 * cent);
            turn(-45);
            driveForward(3 * cent);
            turn(90);
            driveForward(4.5 * cent);
            //driveForward(80);
            //turn(20);
            servoSet();
            turn(-90);
            driveForward(8.5);
//            driveBackward(-8.5 * cent);

        }
    }




    // VISION//

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


    //CRANE//

    public void drop() {
        while (craneMotor.getCurrentPosition() < craneTop && opModeIsActive()) {

//one if positive, the other negative. Test with "Main Test Bed" code
            craneMotor.setPower(.75);                            //for one motor named craneMotor
            //pickupMotor.setPower(-1);                            //for 2 motors, 2nd called pickupMotor,
        }

        //stops the lift system
        craneMotor.setPower(0);
        //pickupMotor.setPower(0);                                //for 2 motors
    }


    public void lift() {
        while (craneMotor.getCurrentPosition() > craneLim && opModeIsActive()) {

            craneMotor.setPower(-1);
            //pickupMotor.setPower(-1); //for 2 motors
        }

        craneMotor.setPower(0);
        //pickupMotor.setPower(0); //for 2 motors
    }


    // DRIVING //


    public void driveForward(double distance) {
        distance = 34.3333 * distance;
        while (frontLeftMotor.getCurrentPosition() < distance && frontRightMotor.getCurrentPosition() < distance && opModeIsActive()) {

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
        while (frontLeftMotor.getCurrentPosition() > distance && frontRightMotor.getCurrentPosition() > distance && opModeIsActive()) {

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

        while ((frontLeftMotor.getCurrentPosition() > dist) && ((Math.abs(backRightMotor.getCurrentPosition())) > dist) && opModeIsActive()) {

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

        while (((Math.abs(frontLeftMotor.getCurrentPosition())) > dist) && (backRightMotor.getCurrentPosition() > dist) && opModeIsActive()) {

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

    public void turn(double degrees) {
        degrees = 17.5 * degrees; //9.5 old , 19 is too much
        if (degrees > 0) {
            while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees && opModeIsActive()) {

                frontRightMotor.setPower(-.35);
                backRightMotor.setPower(-.35);
                frontLeftMotor.setPower(.35);
                backLeftMotor.setPower(.35);
            }
        }
        if (degrees < 0) {
            degrees *= -1;
            while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees && opModeIsActive()) {

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


    public void servoSet() {
        dumpServo.setPosition(.1);
    }
}


//1000 ticks = 30cm
//1000/30= 33.33
//90 degrees = 950 ticks

//turn 90 = right
//turn -90 = left
//driveBackward needs negative

//how to use servo
//servoMain.setPosition(value);

//Begin Vision


// VISION //





























