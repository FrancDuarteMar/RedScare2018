package org.firstinspires.ftc.teamcode.girlsv2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.IllegalFormatCodePointException;


@TeleOp(name = "Mechanicats Op Mode", group = "Mechanicats")
public class MechanicatsOpMode extends OpMode {

    // all of our variables
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Lift= null;
    private Servo boxOpener = null;
    private DcMotor arm = null;
    private Servo tokenBox = null;
    private double CatOutAngle = 0.7;
    private double CatInAngle = .5;
    private double BoxOpen = 0;
    private double BoxClosed = .6;
    private int arm_up_position = -481;
    private int arm_down_position = -2000;
    private int arm_speed = 10;
    private double arm_up_power = 1;
    private double arm_down_power = .3;

    //private double spoolMax = 3800; //middle:2968(use 3000) over 4107
    //private double spoolMin = -130;


    private double spoolMax = 3550;
    private double spoolMin = -100;
    int speed = 1;

    // intitalizations
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorFrontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        motorBackRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        boxOpener = hardwareMap.get(Servo.class, "box_opener");
        arm = hardwareMap.get(DcMotor.class, "arm");
        tokenBox = hardwareMap.get(Servo.class,"token_box");
        Lift = hardwareMap.get(DcMotor.class,"lift");

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

        //  arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    static int Count = 0;
    static int Targetposition=0;

    // where buttons are assign
    // ed
    @Override
    public void loop() {
        motorFrontLeft.setPower(gamepad1.left_stick_y / speed);
        motorFrontRight.setPower(-gamepad1.right_stick_y / speed);
        motorBackLeft.setPower(gamepad1.left_stick_y /speed);
        motorBackRight.setPower(-gamepad1.right_stick_y /speed);

        if (gamepad1.right_bumper){
            speed = 1;
        }

        if (gamepad1.right_trigger > 0 ){
            speed = 3;
        }
        if (gamepad1.a) {
            boxOpener.setPosition(BoxOpen);
            //telemetry.addData("servo A", boxOpener.getPosition());
        }
        // we commented out the else to try and fx the box opening and closing
        //       else
        //     {
        //       boxOpener.setPosition(BoxClosed);
        // }
//
        //

        // these are the lift button assignments
//        if(gamepad1.dpad_up) {
//            Lift.setPower(-1);
//
//        }else if(gamepad1.dpad_down) {
//            Lift.setPower(1);
//        }else {
//            Lift.setPower(0);
//        }

        // this is encoders to make sure the spool doesnt unravel
        if (gamepad1.dpad_down && (Lift.getCurrentPosition() > spoolMin)) {
            Lift.setPower(-1);
            telemetry.addData("spoolValue",Lift.getCurrentPosition());

        }else if (gamepad1.dpad_up && (Lift.getCurrentPosition() < spoolMax)) {
            Lift.setPower(.75);
        }else {
            Lift.setPower(0);

        }
        // this is for the ball box
        if (gamepad1.b) {
            boxOpener.setPosition(BoxClosed);
            //telemetry.addData("servo B", boxOpener.getPosition());
        }
        //

        // this is for the arm
//        if (gamepad1.dpad_down) {
        if (gamepad1.left_trigger > .5) {//The trigger is an analog control similar to a stick
            if (Targetposition > arm_down_position) {
                Targetposition=Targetposition-arm_speed;
            }

//        } else if (gamepad1.dpad_up) {
        } else if (gamepad1.left_bumper) {//The bumper is an on/off button
            if (Targetposition < arm_up_position) {
                Targetposition=Targetposition+arm_speed;
            }
        }

        // telemetry so our robot can give us feedback
        arm.setTargetPosition(Targetposition);
        arm.setPower(arm_up_power);
        telemetry.addData("set", Targetposition);
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("spoolValue",Lift.getCurrentPosition());
        telemetry.update();
    }
}