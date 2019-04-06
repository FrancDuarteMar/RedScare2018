package org.firstinspires.ftc.teamcode.girls;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.IllegalFormatCodePointException;

@Disabled
@TeleOp(name = "F-Mechanicats Telemetry", group = "C")
public class telemetry extends OpMode {

    // all of our variables
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor lift = null;
    private Servo boxOpener = null;
    private DcMotor arm = null;
    private Servo tokenBox = null;
    private double CatOutAngle = 0.7;
    private double CatInAngle = .5;
    private double BoxOpen = .0;
    private double BoxClosed = .6;
    private int arm_up_position = -481;
    private int arm_down_position = -2000;
    private int arm_speed = 10;
    private double arm_up_power = 1;
    private double arm_down_power = .3;

    //    double liftTop = ;
//    dounle liftBottom = ;
    // intitalizations
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        motorBackRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        boxOpener = hardwareMap.get(Servo.class, "box_opener");
        arm = hardwareMap.get(DcMotor.class, "arm");
        tokenBox = hardwareMap.get(Servo.class, "token_box");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Set motor directions
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        //boxOpener.setPosition(Servo.MAX_POSITION);
        arm.setDirection(DcMotor.Direction.FORWARD);
        tokenBox.setPosition(CatInAngle);
        boxOpener.setPosition(BoxClosed);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Targetposition = 0;

//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    static int Count = 0;
    static int Targetposition = 0;

    // where buttons are assign
    // ed
    @Override
    public void loop() {

        telemetry.update();
//        composeTelemetry();



        arm.setTargetPosition(Targetposition);
        arm.setPower(arm_up_power);
        telemetry.addData("crane", lift.getCurrentPosition());
        telemetry.addData("set", Targetposition);
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.update();
    }

//    public void composeTelemetry() {
//
//        telemetry.addLine()
//                .addData("Crane Value:", Lift.getCurrentPosition());
////        telemetry.addLine()
////                .addData("Lift Value", liftMotor.getCurrentPosition());
////        telemetry.addLine()
////                .addData("Back Left  Value:", backLeftMotor.getCurrentPosition());
////        telemetry.addLine()
////                .addData("Back Right Value: ", backRightMotor.getCurrentPosition());
////        telemetry.addLine()
////                .addData("front left value:", frontLeftMotor.getCurrentPosition());
////        telemetry.addLine()
////                .addData("Front Right Value: ", frontRightMotor.getCurrentPosition());
////        telemetry.addLine()
////                .addData("Dump Value: ", dumpMotor.getCurrentPosition());
////
//
//    }
}
