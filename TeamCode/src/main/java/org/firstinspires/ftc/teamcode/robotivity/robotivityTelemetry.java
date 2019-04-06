package org.firstinspires.ftc.teamcode.robotivity;

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
@TeleOp(name=" Robotivity Telemetry", group="B ")
public class robotivityTelemetry extends LinearOpMode {
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
    private int craneTop = 3940;      //Luke 2860                                         //4000 // new rev 2500; old 3000
    private int craneLow = 200;

    private int liftTop = -2507; //luke: -2428                                                                //used to be -2919
    private int liftBottom = -100;


    @Override

    public void runOpMode() {
        backLeftMotor = hardwareMap.dcMotor.get("Back Left Motor");
        backRightMotor = hardwareMap.dcMotor.get("Back Right Motor");

        frontLeftMotor = hardwareMap.dcMotor.get("Front Left Motor");
        frontRightMotor = hardwareMap.dcMotor.get("Front Right Motor");

        craneMotor = hardwareMap.get(DcMotor.class, "Crane Motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
      //  dumpMotor = hardwareMap.get(DcMotor.class, "Dump Motor");


//        dumpServo = hardwareMap.get(Servo.class, "Dump Servo");
        tokenServo = hardwareMap.get(Servo.class, "Token Servo");
        intakeServo = hardwareMap.get(Servo.class, "Intake Servo");


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



        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            composeTelemetry();
        }
    }

        public void composeTelemetry () {

            telemetry.addLine()
                    .addData("Crane Value:", craneMotor.getCurrentPosition());

//                .addData("Back Left  Value:", backLeftMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("Back Right Value: ", backRightMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("front left value:", frontLeftMotor.getCurrentPosition());
//        telemetry.addLine()
//                .addData("Front Right Value: ", frontRightMotor.getCurrentPosition());




            // telemetry.addLine()
            // .addData("Intake Value: ", intakeMotor.getCurrentPosition());

        }

//LEFT IS ALWAYS NEGATIVE AND RIGHT IS POSITIVE

    }


