
package org.firstinspires.ftc.teamcode.AutoTests.InitialAutoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
@Autonomous(name="test 10", group="Linear Opmode")

public class turn extends LinearOpMode {

// Initialize motors

    private DcMotor frontLeftMotor =  null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    //  private Servo dumpServo = null;
    //  private Servo dumpServo = null;
    private DcMotor pickupMotor = null;



    //setting limits
    private int craneTop = 2992; //old -3088 new 2992
    private int craneLow = 0;
    private int craneLim = 50; //old 61

    //var
    private double pos = 5;
    private double cent = 30.48;



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();





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
        // dumpServo = hardwareMap.get(Servo.class, "servoMain");

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
        waitForStart();
        runtime.reset();


        //turn(TYPE In DEGREES;
        //driveForward(TYPE IN CENTIMETERS);
        //driveBackward(TYPE IN CENTIMETERS IN -);


        //TEST CODE HERE\\






sLeft(20);
//turn(-90);



















    }


    //CRANE//

    public void drop(){
        while (craneMotor.getCurrentPosition() > craneTop) {

//one if positive, the other negative. Test with "Main Test Bed" code
            craneMotor.setPower(1);                            //for one motor named craneMotor
            //pickupMotor.setPower(-1);                            //for 2 motors, 2nd called pickupMotor,
        }

        //stops the lift system
        craneMotor.setPower(0);
        //pickupMotor.setPower(0);                                //for 2 motors
    }


    public void lift() {
        while (craneMotor.getCurrentPosition() < craneLim) {

            craneMotor.setPower(-1);
            //pickupMotor.setPower(-1); //for 2 motors
        }

        craneMotor.setPower(0);
        //pickupMotor.setPower(0); //for 2 motors
    }




    // DRIVING //

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





























