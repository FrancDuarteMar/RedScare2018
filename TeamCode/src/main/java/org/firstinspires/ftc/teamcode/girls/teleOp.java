package org.firstinspires.ftc.teamcode.girls;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.IllegalFormatCodePointException;

@Disabled
@TeleOp(name = "F-Mechanicats Op Mode", group = "C")
public class teleOp extends OpMode {

    // all of our variables
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor Lift = null;
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

    double liftTop = 5044; //5044 //5300
    double liftBottom = 200;

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
        Lift = hardwareMap.get(DcMotor.class, "lift");

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

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    static int Count = 0;
    static int Targetposition = 0;

    // where buttons are assign
    // ed
    @Override
    public void loop() {
        motorFrontLeft.setPower(gamepad1.left_stick_y);
        motorFrontRight.setPower(-gamepad1.right_stick_y);
        motorBackLeft.setPower(gamepad1.left_stick_y);
        motorBackRight.setPower(-gamepad1.right_stick_y);

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


        if (gamepad1.dpad_down && (Lift.getCurrentPosition() > liftBottom)) {
            Lift.setPower(-.75);

        } else if (gamepad1.dpad_up && (Lift.getCurrentPosition() < liftTop)) {
            Lift.setPower(.75);
        } else {
            Lift.setPower(0);
        }

        if (gamepad1.b) {
            boxOpener.setPosition(BoxClosed);
            //telemetry.addData("servo B", boxOpener.getPosition());
        }
        //

        // this is for the arm
//        if (gamepad1.dpad_down) {
        if (gamepad1.left_trigger > .5) {//The trigger is an analog control similar to a stick
            if (Targetposition > arm_down_position) {
                Targetposition = Targetposition - arm_speed;
            }

//        } else if (gamepad1.dpad_up) {
        } else if (gamepad1.left_bumper) {//The bumper is an on/off button
            if (Targetposition < arm_up_position) {
                Targetposition = Targetposition + arm_speed;
            }
        }
        // telemetry so our robot can give us feedback
        arm.setTargetPosition(Targetposition);
        arm.setPower(arm_up_power);
        telemetry.addData("crane", Lift.getCurrentPosition());
        telemetry.addData("set", Targetposition);
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.update();
    }
}