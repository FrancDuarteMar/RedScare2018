package org.firstinspires.ftc.teamcode.AutoTests.InitialAutoTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/* FOR TEST BED */

@Disabled
@TeleOp(name="Telemetry Test", group="Red Scare")
public class telemetry extends OpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor craneMotor = null;
    private DcMotor pickupMotor = null;
    private Servo servoMain = null;


    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front left motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front right motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back left motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "Back right motor");
        craneMotor = hardwareMap.get(DcMotor.class, "craneMotor");
        pickupMotor = hardwareMap.get(DcMotor.class, "pickupMotor");
        servoMain = hardwareMap.get(Servo.class, "servoMain");

//        initGoldAlignDetector();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        telemetry.addData("Crane Position: ", craneMotor.getCurrentPosition());
        telemetry.update();

        telemetry.addData(" Front Left Motor Position: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData(" Back Left Motor Position: ", backLeftMotor.getCurrentPosition());
        telemetry.addData(" Front Right Motor Position: ", frontRightMotor.getCurrentPosition());
        telemetry.addData(" Back Right Motor Position: ", backRightMotor.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Servo Position: ", servoMain.getPosition());



        //craneMotor.isBusy();


    }

    @Override
    public void loop() {

        while (gamepad1.b){
//            telemetry.addData("Crane Position: ", craneMotor.getCurrentPosition());
//            telemetry.update();
//
//            telemetry.addData(" Front Left Motor Position: ", frontLeftMotor.getCurrentPosition());
//            telemetry.addData(" Back Left Motor Position: ", backLeftMotor.getCurrentPosition());
//            telemetry.addData(" Front Right Motor Position: ", frontRightMotor.getCurrentPosition());
//            telemetry.addData(" Back Right Motor Position: ", backRightMotor.getCurrentPosition());
//            telemetry.update();

            telemetry.addData("Servo Position: ", servoMain.getPosition());
            telemetry.addData("Servo Pos: ", servoMain.getDirection());
            telemetry.update();
        }





        /* DRIVING */


    }
}