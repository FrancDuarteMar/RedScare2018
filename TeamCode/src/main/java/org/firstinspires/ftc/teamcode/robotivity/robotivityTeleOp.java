package org.firstinspires.ftc.teamcode.robotivity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@TeleOp(name="Robotivity TeleOp ", group="B")
public class robotivityTeleOp extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    //DcMotor liftMotor;

    DcMotor craneMotor;
    DcMotor intakeMotor;
   // DcMotor dumpMotor;
    //  Servo dumpServo;
    Servo tokenServo;
    Servo intakeServo;



    int dumpTop = -122;
    private float drive = 0;
    private double speed = 1.05;


    //setting limits for robot lift/crane
    private int craneTop = 3900 ; //3940
    private int craneLow = 200;

    private int liftTop = -2507; //luke: -2428                                                                //used to be -2919
    private int liftBottom = -100;




    @Override
    public void init() {
        // Initializing and maping hardware
        backLeftMotor = hardwareMap.dcMotor.get("Back Left Motor");
        backRightMotor = hardwareMap.dcMotor.get("Back Right Motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front Left Motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front Right Motor");

        craneMotor = hardwareMap.get(DcMotor.class, "Crane Motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
    //    dumpMotor = hardwareMap.get(DcMotor.class, "Dump Motor");




//        dumpServo = hardwareMap.get(Servo.class, "Dump Servo");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");
        intakeServo = hardwareMap.get(Servo.class, "Intake Servo");


        frontRightMotor.setDirection(DcMotor.Direction.REVERSE); //used to be front left now is mapped to back left through different name
        backRightMotor.setDirection(DcMotor.Direction.REVERSE); //Used to be back left now is mapped to back left


//        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); //used to be front left now is mapped to back left through different name
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); //Used to be back left now is mapped to back left


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

       // dumpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Crane encodervalue:", craneMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.update();

        frontLeftMotor.setPower(gamepad1.left_stick_y / speed);
        backLeftMotor.setPower(gamepad1.left_stick_y / speed);


        frontRightMotor.setPower(gamepad1.right_stick_y / speed);
        backRightMotor.setPower(gamepad1.right_stick_y / speed);


        // SPEEDS  //

        // REGULAR //
        if (gamepad1.a) {
            speed = 1.05;
        }

        // SLOW //
        if (gamepad1.b) {
            speed = 3;
        }

//              intake          //
        if (gamepad1.x){
            intakeServo.setPosition(0);
        }
        if (gamepad1.y){
            intakeServo.setPosition(1);
        }



        // Crane MECHANISM //

        if (gamepad1.dpad_up && (craneMotor.getCurrentPosition() > craneLow)) {
            craneMotor.setPower(-1);

        } else if (gamepad1.dpad_down && (craneMotor.getCurrentPosition() < craneTop )) {
            craneMotor.setPower(1);

        } else {
            craneMotor.setPower(0);


        }




        // Mineral Intake //
        if (gamepad1.left_trigger > 0){
            intakeMotor.setPower(1);
        }
        if (gamepad1.right_trigger > 0){
            intakeMotor.setPower(-1);
        }
        if ((gamepad1.left_trigger == 0 ) && (gamepad1.right_trigger == 0)){
            intakeMotor.setPower(0);
        }



    }




//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

}

