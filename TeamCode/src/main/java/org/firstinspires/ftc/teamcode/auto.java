package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="auto", group="Linear Opmode")


public class auto extends LinearOpMode{

    private DcMotor frontLeftMotor =  null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private Servo servoMain = null;
    private DcMotor pickupMotor = null;

    //1 inch will equal ticks
    private int inchPer = 0; //set encoder ticks here for inches

    //1 cm will equal ticks
    private int cmPer = 0; //set encoder ticks here for inches

    //calling for position of motors
    private double flPos = frontLeftMotor.getCurrentPosition();
    private double blPos = backLeftMotor.getCurrentPosition();
    private double frPos = frontRightMotor.getCurrentPosition();
    private double brPos = backRightMotor.getCurrentPosition();

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Front Left", flPos);
        telemetry.addData("Front Right", frPos);
        telemetry.addData("Back Left", blPos);
        telemetry.addData("Back Right ", brPos);
        telemetry.update();

    }
}
