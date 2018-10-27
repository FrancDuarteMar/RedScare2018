
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CenterDepot", group="Linear Opmode")

public class CenterDepot extends LinearOpMode {


    private DcMotor frontLeftMotor =  null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private Servo servoMain = null;
    private DcMotor pickupMotor = null;

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
        servoMain = hardwareMap.get(Servo.class, "servoMain");


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

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //turn(TYPE I DEGREES;
        //driveForward(TYPE IN CENTIMETERS);


        //The following is for the crater beginning. Right and left is relative to the robot not an outside perspective.

        //drive for right; start at crater



        //Beginning of depot start. Left and right is relative to robot.

        //drive for center, start at depot

        driveForward(62);
        driveForward(60);
        servoMain.setPosition(-1);
        servoMain.setPosition(1);
        turn(120);
        driveForward(240);



    }

    public void drop(double drop){
        drop = 20.83125 * drop;
        while(frontLeftMotor.getCurrentPosition() < drop && craneMotor.getCurrentPosition() < drop){

            craneMotor.setPower(.75);
            pickupMotor.setPower(-.75);

        }
        craneMotor.setPower(0);
        pickupMotor.setPower(0);
        reset();
    }

    public void liftup(double lift){
        lift = 20.83125 * lift;
        while(pickupMotor.getCurrentPosition() < lift && craneMotor.getCurrentPosition() < lift){

            craneMotor.setPower(.75);
            pickupMotor.setPower(.75);
        }
        craneMotor.setPower(0);
        pickupMotor.setPower(0);
        reset();
    }

/*
    public void lift(double lift){
        lift = 20.83125 * lift;
        while(frontLeftMotor.getCurrentPosition() < lift && craneMotor.getCurrentPosition() < lift){

            craneMotor.setPower(-.75);
            pickupMotor.setPower(.75);


        }
        craneMotor.setPower(0);
        pickupMotor.setPower(0);
        reset();
    }
*/

    public void driveForward(double distance){
        distance = 33.3333 * distance;
        while(frontLeftMotor.getCurrentPosition() < distance && frontRightMotor.getCurrentPosition() < distance){

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
    public void driveBackward(double distance){
        distance = 33.3333 * distance;
        while(frontLeftMotor.getCurrentPosition() > distance && frontRightMotor.getCurrentPosition() > distance){

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
    public void turn(double degrees){
        degrees = 9.5 * degrees;
        if(degrees > 0) {
            while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees) {

                frontRightMotor.setPower(-.35);
                backRightMotor.setPower(-.35);
                frontLeftMotor.setPower(.35);
                backLeftMotor.setPower(.35);
            }
        }
        if(degrees < 0) {
            degrees*=-1;
            while (frontLeftMotor.getCurrentPosition() < degrees && frontRightMotor.getCurrentPosition() < degrees) {

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
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

    /*public void height(double degrees){
        degrees = 9.5 * degrees;
        if(degrees > 0) {
            while (craneMotor.AgetCurrentPosition() < degrees) {

                craneMotor.setPower(-.35);

            }
        }
        if(degrees < 0) {
            degrees*=-1;
            while (craneMotor.getCurrentPosition() < degrees) {

                craneMotor.setPower(.35);

            }
        }
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        craneReset();
    }
    public void craneReset() {
        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}*/


//1000 ticks = 30cm
//1000/30= 33.33
//90 degrees = 950 ticks

//turn 90 = right
//turn -90 = left
//driveBackward needs negative


//Begin Vision

//how to use servo
//servoMain.setPosition(value);
