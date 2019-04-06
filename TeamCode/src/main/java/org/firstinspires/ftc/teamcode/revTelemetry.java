package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Func;

@Disabled
@TeleOp(name=" Rev Telemetry", group="Red Scare ")
public class revTelemetry extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor liftMotor;
    DcMotor craneMotor;
    DcMotor intakeMotor;
    DcMotor dumpMotor;
   // Servo dumpServo;
    Servo tokenServo;



    private double speed = 1.05;
    private boolean drive = false;


    //setting limits for robot lift/crane
    private int craneTop = 2100 ; //4000 // new rev 2500; old 3000
    private int craneLow = 200;

    //mineral lift
    private int liftTop = -2500; //used to be -2919
    private int liftBottom = -200;


    @Override

    public void runOpMode(){
        backLeftMotor = hardwareMap.dcMotor.get("Back left motor");
        backRightMotor = hardwareMap.dcMotor.get("Back right motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front left motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front right motor");

        craneMotor = hardwareMap.get(DcMotor.class, "Crane Motor");
        liftMotor = hardwareMap.get(DcMotor.class, "Lift Motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        dumpMotor = hardwareMap.get(DcMotor.class, "Dump Motor");

        //dumpServo = hardwareMap.get(Servo.class, "Dump Servo");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");

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

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dumpMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumpMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        while(opModeIsActive()){
            telemetry.update();
            composeTelemetry();
        }




        if (gamepad1.b){
            drive = true;
        }
        if (gamepad1.x){
            drive = false;
        }


        if (drive == true) {
            if (gamepad1.dpad_up) {
                frontLeftMotor.setPower(.2);
                frontRightMotor.setPower(.2);
                backLeftMotor.setPower(.2);
                backRightMotor.setPower(.2);
            } else if (gamepad1.dpad_down) {
                frontRightMotor.setPower(-.2);
                frontLeftMotor.setPower(-.2);
                backRightMotor.setPower(-.2);
                backLeftMotor.setPower(-.2);
            } else {
                stopMotors();
            }
        }

        if (drive == true) {
            if (gamepad1.left_bumper) {
                craneMotor.setPower(-.2);
            }
           else if (gamepad1.right_bumper) {
                craneMotor.setPower(.2);
            }
            else {
                craneMotor.setPower(0);
            }
        }

        if (drive == true ){
            if (gamepad1.left_stick_y > 0){
                liftMotor.setPower(.2);
            }
            else if (gamepad1.right_stick_y > 0){
                liftMotor.setPower(-.2);
            }
            else{
                liftMotor.setPower(0);
            }

        }


        }




    public void turn(double degrees){
        degrees = 19.75 * degrees; //9.5 old , 19 is too much
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

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void stopMotors (){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }

    public void composeTelemetry () {

        telemetry.addLine()
                .addData("Crane Value:", craneMotor.getCurrentPosition());
        telemetry.addLine()
                .addData("Lift Value", liftMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("Back Left  Value:", backLeftMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("Back Right Value: ", backRightMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("front left value:", frontLeftMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("Front Right Value: ", frontRightMotor.getCurrentPosition());
        telemetry.addLine()
                .addData("Dump Value: ", dumpMotor.getCurrentPosition());




        // telemetry.addLine()
               // .addData("Intake Value: ", intakeMotor.getCurrentPosition());

    }

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

}

