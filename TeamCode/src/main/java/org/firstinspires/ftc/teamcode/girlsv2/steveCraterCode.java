package org.firstinspires.ftc.teamcode.girlsv2;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name = "Steve Crater  ", group = "Mechanicats")
public class steveCraterCode extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfW2aNH/////AAABmceeXSnBnUD7jO0zxs+mMDhYz1NL1zULmPQV57IHUgiDvtf+B7m2/SN7LU6dafSrXpqSPbNXRiYTswe3xBEWInCwQRTLpY3X2eQy6m5S7ibapiZCuj9Cz2ICZsZWNtnIYfdKbXr0xR3+p4snFR8s86EIdC9GHzteaVFbAJF6T4CKY9t2BEu2Zte3IqviOhBEHMOzFPgZIRTx3E5iF9RwsbyGeMVZ0twV1VOvcLEJgAFXIk48v8JUa19SRLgsqEI+HI63sjdIUKd3cj/yUNi18zIhJsVr+8+ihlDJeH7+hJ3gPxwaOtkDP9KuKWr5a8AMrbtTTeJoAEPgLkCaCQ6uNnlnDfZNnwn9qllgt7LKLWRm";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    static int Targetposition = 0;


    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor arm = null;
    private DcMotor lift = null;


    // variables for turns and distances
    private Servo tokenBox = null;
    private int elementTurnAngle = 15;
    private int elementDriveDistance = 28;
    private int elementDriveDistanceLong = 50;
    private int elementSecondDistance = 116;
    private int elementThirdDistance = 125;
    private int craterParkDistance = 170;
    private double CatOutAngle = 1;
    private double CatInAngle = .2;
    private double ticksPerRev = 1200;
    private double wheelDiameter = 10;//diameter in cm
    public double pi = 3.141;
    private double distanceFactor = ticksPerRev / (wheelDiameter * pi);
    private int elementTurnAround = 185;
    private int depotElementDist1 = 140;
    private int depotElementDist2 = 102;
    private int parkInOtherCrater = 105;
    private int turnIntoOtherCrater = 25;
    private int goldSeekAngle = 40;
    private int elementDriveSpot = 9;
    private int arm_up_position = -481;
    private int arm_down_position = -1700;
    private int arm_speed = 7;
    private double arm_up_power=.4;
    private double arm_down_power=.3;
    private int GoldCheckHeight = 450;
    private int FindGoldTimeout = 2000;

    private double turnSpeed = 0.5;
    private double fudge = 1.3;
    private double sampleSpeed = 0.7;
    private double tokenSpeed = 0.9;
    private double craterSpeed = 1;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private int robotHeading = 0;

    private int fixheading = 0;

//
//    private double spoolMax = 3500; //middle:2968(use 3000) over 4107 //used to be 3800(4/1)
//    private double spoolMin = -130;


    private double spoolMax = 3550;
    private double spoolMin = -100;


    public void initialize() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // confugurations
        motorFrontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        motorFrontRight = hardwareMap.get   (DcMotor.class, "front_right_motor");
        motorBackLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        motorBackRight = hardwareMap.get(DcMotor.class, "back_right_motor");
        tokenBox = hardwareMap.get(Servo.class, "token_box");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class,"lift");


        // we need to be using Mechanicats Rev in the configurations



        // Set motor directions
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        tokenBox.setPosition(CatInAngle);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Targetposition=0;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        modernRoboticsI2cGyro.calibrate();

        while (modernRoboticsI2cGyro.isCalibrating()) {

            telemetry.addData(">", "Calibrating...");
            telemetry.update();
            sleep(50);

            modernRoboticsI2cGyro.resetZAxisIntegrator();
            robotHeading = 0;
        }
        telemetry.addData(">", "Done calibrating...");
        telemetry.update();


        reset();

    }


    @Override
    public void runOpMode() {

        initialize();


        //Holding lift position
        telemetry.addData(">", "Setting up lift...");
        telemetry.update();

        lift.setTargetPosition(0);
        lift.setPower(-.75);


        // telemetry so Rosie can tell us when she is ready
        telemetry.addData(">", "Wait start...");
        telemetry.update();

//        waitForStart();
        telemetry.addData(">", "Ready To Go...");
        telemetry.update();
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "waiting for start...");
        telemetry.update();


        waitForStart();

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "about to drop");
        telemetry.update();


        drop();

        //fixheading= (modernRoboticsI2cGyro.getIntegratedZValue());

        telemetry.addData(">", "dropped");
        telemetry.update();

//        modernRoboticsI2cGyro.resetZAxisIntegrator();
//        robotHeading = 0;

        turn(-15);
        liftLower();
        turn(15);


//        while (robotHeading != 0 ){
//           turn(-fixheading);
//
//        }

        // ******************************************************** comment out one or the other depending in where we start ********************************************************
        // there are problems with vision
        //testVision();
        ScenarioCrater();
//       ScenarioInFrontOfDepot();

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void drop() {
        while ((lift.getCurrentPosition() < spoolMax) && opModeIsActive()) {

//one if positive, the other negative. Test with "Main Test Bed" code
            lift.setPower(.75);                            //for one motor named craneMotor
        }

        //stops the lift system
        lift.setPower(0);
        // fixheading= (modernRoboticsI2cGyro.getIntegratedZValue());
    }

    public void liftLower() {
        while ((lift.getCurrentPosition() > spoolMin) && opModeIsActive()) {

            lift.setPower(-1);
        }

        lift.setPower(0);
        idle();
    }


    private void testVision()
    {
        GoldCheckHeight = 0;//Set gold check height so all positions are displayed
        while(opModeIsActive()){
            isGoldThere();
        }
    }

    // this is all for finding the Gold Mineral
    private boolean isGoldThere()
    {
        List<Recognition> updatedRecognitions;
        int goldMineralX = -100;
        int MineralY = -100;
        boolean goldFound = false;
        boolean elementFound = false;
        ElapsedTime timeout = new ElapsedTime();

        timeout.reset();

        int elementCount = 0;
        float lowestMineralY = -100;
        boolean lowestMineralTypeIsGold= false;
        if (tfod != null) {
            do {
                // getUpdat
                // .....................................................+edRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    elementCount= updatedRecognitions.size();
                    elementFound = true;
                    for (Recognition recognition : updatedRecognitions){
                        if (recognition.getTop() > lowestMineralY){
                            lowestMineralY = recognition.getTop();
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                                lowestMineralTypeIsGold = true;
                            }
                            else{
                                lowestMineralTypeIsGold = false;

                            }

                        }
                    }
                }
                goldFound = lowestMineralTypeIsGold;
            }
            while ((!elementFound) && opModeIsActive() && timeout.milliseconds() < FindGoldTimeout);//While not found mineral, still in op mode or not timed out
            telemetry.addData("Number of Elements Found: ", elementCount );
            telemetry.addData("Gold Found", goldFound);


            telemetry.update();
            return goldFound;
        }
        else{
            return false;
        }
    }


    private boolean isGoldThere_old()
    {
        List<Recognition> updatedRecognitions;
        int goldMineralX = -100;
        int MineralY = -100;
        boolean goldFound = false;
        boolean elementFound = false;
        ElapsedTime timeout = new ElapsedTime();

        timeout.reset();

        if (tfod != null) {
            do {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {//Found at least one element
                        //Check if we found the gold by checking all elements in the list
                        for (Recognition recognition : updatedRecognitions) {
                            MineralY = (int) recognition.getTop();
                            if (MineralY > GoldCheckHeight) {
                                elementFound = true;
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    goldFound = true;
                                }
                            }
                        }
                    }
                }
            }
            while ((!elementFound) && opModeIsActive() && timeout.milliseconds() < FindGoldTimeout);//While not found mineral, still in op mode or not timed out

            // telemetry for when Rosie is looking for the Gold Mineral
            if (elementFound){
                telemetry.addLine("Elements found ")
                        .addData("-", updatedRecognitions.size());
            } else {
                telemetry.addLine("Elements found ")
                        .addData("-", 0);
            }
            if (goldFound) {
                telemetry.addLine("Gold found -")
                        .addData("x", goldMineralX)
                        .addData("y", MineralY);
            } else {
                telemetry.addLine("Gold not found");
                //telemetry.addData("Gold not found %d", 3);
            }
            telemetry.update();
            return goldFound;
        }
        else{
            return false;
        }
    }

    // telemetry so Rosie can tell us what she is doing
    public void ScenarioCrater2() {
        sampleElement();
    }

    public void ScenarioCrater() {
        sleep(1500);
        telemetry.addData(">", "Landing...");
        telemetry.update();
        landRobot();

        telemetry.addData(">", "Sampling...");
        telemetry.update();
        sampleElement();

        telemetry.addData(">", "Droping...");
        telemetry.update();
        dropToken();

        telemetry.addData(">", "Parking...");
        telemetry.update();
        parkInCrater();

        telemetry.addData(">", "Finished...");
        telemetry.update();

    }

    public void landRobot() {
        // lower robot
        // detach from lander
        arm.setTargetPosition(arm_up_position);
        arm.setPower(arm_up_power);
    }

    public void sampleElement() {
        // position 1 = left, 2 = middle, 3 = right
        // find position, turn to position, go to position
        int GoldPosition;

        // the flashlight has been commented out for testing
        //CameraDevice.getInstance().setFlashTorchMode(true);

        // shortened this distance - was 12
        driveBackward(elementDriveSpot, sampleSpeed);

        GoldPosition = getGoldPosition();

        //CameraDevice.getInstance().setFlashTorchMode(false);

        telemetry.addData(">", "Samplig gold...");
        telemetry.update();
        if ((GoldPosition == 1) || (GoldPosition == 3)) {
            //turn(-elementTurnAngle);
            // element drive dist long was 55
            driveBackward(elementDriveDistanceLong, sampleSpeed);
            sleep(300);
            // we subtracted 15 from the distance to the element because it was going too far
            driveForward(elementDriveDistanceLong, sampleSpeed);
            //turn(-(90 - elementTurnAngle));
        } else {//if (GoldPosition == 2) {
            //telemetry.addData(">", "Going to 2");
            //telemetry.update();
            // element drive dist was too long - ran into crater. used to be 57
            driveBackward(elementDriveDistance, sampleSpeed);

            sleep(300);
            driveForward(elementDriveDistance, sampleSpeed);
            //turn(90);
        } //else if (GoldPosition == 3) {
        //  turn(elementTurnAngle);
        //  driveForward(elementDriveDistance, sampleSpeed);
        //  driveBackward(elementDriveBackDistance, sampleSpeed);
        //  turn(-(90 + elementTurnAngle));
        //}

    }

    public int getGoldPosition() {
        sleep(500);
       // testVision();
        if (isGoldThere()){
            return 2;
        }
        else {
            turnAnlgeGyro(goldSeekAngle, turnSpeed);
            sleep(1000);
            if (isGoldThere()){
                return 1;
            }
            turnAnlgeGyro(-(2*goldSeekAngle), turnSpeed);
            sleep(200);
            return 3;

        }
    }


    public void dropToken() {
        // turn to token drop-off place
        // drive forward
        //turn(180);
        // changed from 90 - it was too much
        turnHeadingGyro(85, turnSpeed);
        sleep(200);
        driveBackward(elementSecondDistance, tokenSpeed);
        // tag was 44, changed it because it didnt turn far enough. 46 not enough
        turnAnlgeGyro( 48, turnSpeed);
        // made longer - it was too short. made longer again
        driveBackward(elementThirdDistance, tokenSpeed);
        tokenBox.setPosition(CatOutAngle);
        sleep(500);
        tokenBox.setPosition(CatInAngle);
        // drop token
    }

    public void parkInCrater() {
        // drive forwards into crater
        arm.setTargetPosition(0);
        arm.setPower(arm_down_power);
        // distance was 175, it was too much
        turnAnlgeGyro(10,.7);
        driveForward(craterParkDistance, craterSpeed);
    }

    public void ScenarioInFrontOfDepot() {

        sleep(1500);
        telemetry.addData(">", "Landing...");
        telemetry.update();
        landRobot();

        telemetry.addData(">", "Sampling...");
        telemetry.update();
        int GoldPosition;

        // finding the gold
        CameraDevice.getInstance().setFlashTorchMode(true);
        driveBackward(elementDriveSpot, sampleSpeed);
        GoldPosition = getGoldPosition();
        CameraDevice.getInstance().setFlashTorchMode(false);

        // sampling for 2 CENTER
        if (GoldPosition == 2){
            driveBackward(100,.7);//needs to be 43"
            tokenBox.setPosition(CatOutAngle);
            sleep(500);
            tokenBox.setPosition(CatInAngle);
            driveForward(100,.7);//needs to be 43"
            // going to crater
            turnAnlgeGyro(90,.7);
            driveBackward(90,.7);//Needs to be 42"
            turnAnlgeGyro(-152,.7);
            driveForward(48,.7);//Needs to be 19"
        }
        else if (GoldPosition == 1){
            driveBackward(60,.7);//needs to be 40"
            turnHeadingGyro(-20,.7); //used to be 30 before i changed it

            driveBackward(55,.7);
            tokenBox.setPosition(CatOutAngle);
            sleep(500);
            tokenBox.setPosition(CatInAngle);
            driveForward(200,.7);

        }
        else if (GoldPosition == 3){
            driveBackward(57,.7);
            turnHeadingGyro(35,.7);
            driveBackward(32,.7);
            tokenBox.setPosition(CatOutAngle);
            sleep(500);
            tokenBox.setPosition(CatInAngle);
            // going into crater
            driveForward(45,.7);
            turnHeadingGyro(-55,.7);
            driveForward(50,.7);
            turnAnlgeGyro(-45,.7);
            driveForward(85,.7);
            turnAnlgeGyro(11,.7);
            driveForward(5,.7);
            turnAnlgeGyro(11,.7);
            driveForward(5,.7);
            turnAnlgeGyro(11,.7);
            driveForward(5,.7);
            turnAnlgeGyro(11,.7);
            driveForward(18,.7);


        }

    }

    // this is just for testing new ideas
    public void ScenarioTest() {
        // this is testing stuff now
        // landing depot -> cube -> token area (tested but still slightly-ish off)
        // first forward was 49, but that was too high - will need to change equation
        // everything has been reduced by 10 (this includes first forward) - not yet tested
        //driveForward(25);
        //driveBackward(10);
        //turn(-135);
        //driveForward(65);

    }

    // this is where we assign the basic drives and turns
    public void driveForward(int distance, double speed) {
        driveStraight(distance, speed);
    }

    public void driveBackward(int distance, double speed) {
        driveStraight(-distance, speed);
    }

    public void turn(int degrees) {
        turnAnlgeGyro(-degrees, turnSpeed);
    }

    // here we set encoder values
    public void reset() {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void encoderCountMode() {
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderPositionMode() {
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Drive forward/backwards in the current tracked direction
    //Note, the heading (i.e. direction) will be the sum of all the turns we
    //have made (i.e. where we SHOULD be pointing) and NOT the direction the
    //robot was pointing when we started to drive. This should therefore
    //account for any changes caused by bumping, colliding etc...
    public void driveStraight(int distance, double speed) {
        driveHeading(distance, robotHeading, speed);
    }

    //Drive forward/backwards in the currently faced direction
    //This will not account for unknown changes in rotation
    public void driveLine(int distance, double speed) {
        int currentHeading;

        currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
        driveHeading(distance, currentHeading, speed);
    }

    //Drive distance with power setting speed, turning to face the specified heading
    //Negative distances will drive backwards
    //When backwards it will still be the FRONT of the robot facing the heading
    //Note, speed can be +ve or -ve and will be automatically corrected
    public void driveHeading(int distance, int targetHeading, double speed) {
        int error;
        int currentHeading;
        double speedCorrection = 0;
        double leftSpeed;
        double rightSpeed;
        // was .015 - too small?
        double gain = .025;
        int distanceTraveled;

        reset();

        if (distance > 0) {
            //Going forwards so make sure the speed is +ve
            speed = Math.abs(speed);
        } else {
            //Going backwards so make sure the speed is -ve
            speed = -Math.abs(speed);
        }

        do {
            //Find where we are currently pointing
            currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();

            //Calculate how far off we are
            error = (currentHeading - targetHeading);

            //Using the error calculate some correction factor
            speedCorrection = error * gain;

            //Adjust the left and right power to try and compensate for the error
            leftSpeed = speed + speedCorrection;
            rightSpeed = speed - speedCorrection;

            //Apply the power settings to the motors
            motorFrontLeft.setPower(leftSpeed);
            motorBackLeft.setPower(leftSpeed);
            motorFrontRight.setPower(rightSpeed);
            motorBackRight.setPower(rightSpeed);

            //Measure all 4 wheel encoders and average to find approximate distance the center of the robot has moved
            distanceTraveled = (int) (((motorFrontLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorBackRight.getCurrentPosition()) / (distanceFactor * 4)));

            // telemetry.addData(">", "H = %d E=%d SL=%f1.2 SR=%f1.2 D=%d",currentHeading, error,leftSpeed, rightSpeed, distanceTraveled);
            // telemetry.update();

        }
        while ((Math.abs(distanceTraveled) < Math.abs(distance)) && opModeIsActive());//Keep going until the magnitude of the distance we have traveled is greater than the magnitude of the distance we asked for

        //Done so turn off the motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }

    //Turn the robot by the specified angle regardless of errors
    //Note, this will turn from the current heading and not the tracked
    //heading
    public void rotateAnlgeGyro(int angleToTurn, double speed) {
        int targetHeading;
        int currentHeading;

        currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
        targetHeading = currentHeading + angleToTurn;
        turnHeadingGyro(targetHeading, speed);
    }

    //Turn the robot by the specified angle
    //Note, this will turn to the tracked position so will account for
    //inflicted errors in direction
    public void turnAnlgeGyro(int angleToTurn, double speed) {
        int targetHeading;

        targetHeading = robotHeading + angleToTurn;
        turnHeadingGyro(targetHeading, speed);
    }

    //Turn to face the specified heading.
    //This will account for inflicted errors in direction
    public void turnHeadingGyro(int targetHeading, double speed) {
        int error;
        int errorThreshold = 2;//How accurate do we need to be? Too high, wrong direction, too small will oscillate
        int currentHeading;
        double turnSpeed;
        double motorPower;

        do {
            currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
            error = (currentHeading - targetHeading);
            if (error == 0) {
                //Just on the off-chance we are actually at an error of zero
                //stop motors ASAP
                turnSpeed = 0.0;
            }
            if (Math.abs(error) > 15) {
                //If need to turn a lot then go at full speed
                turnSpeed = speed;
            } else {
                //Close to correct heading so slow down
                turnSpeed = speed * 0.5;
            }

            if (error > 0) {
                //Positive error so need to turn clockwise
                motorPower = -turnSpeed;
            } else {
                //Negative error so need to turn anti-clockwise
                motorPower = turnSpeed;
            }

            //Turn on the motors
            motorFrontLeft.setPower(-motorPower);
            motorBackLeft.setPower(-motorPower);
            motorFrontRight.setPower(motorPower);
            motorBackRight.setPower(motorPower);

//            telemetry.addData(">", "e = %d h = %d p = %0.2f",error, currentHeading, motorPower);
//            telemetry.update();
        }
        while ((Math.abs(error) > errorThreshold) && opModeIsActive());//Keep turning until close enough to correct heading

        //Turn off the motors
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        //Update the direction we think we are pointing
        robotHeading = targetHeading;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}