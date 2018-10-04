package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Op Mode Ian", group = "Ian")
public class OpModeIan extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector goldAlignDetector = new GoldAlignDetector();

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        DriveUtils.getInstance().init(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

        initGoldAlignDetector();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("goldAlignDetector.isAligned", goldAlignDetector.getAligned());
            telemetry.addData("goldAlignDetector.xPosition", goldAlignDetector.getXPosition());
        }
    }

    private void initGoldAlignDetector() {
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldAlignDetector.useDefaults();

        // Optional Tuning
        goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames

        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlignDetector.maxAreaScorer.weight = 0.005;

        goldAlignDetector.ratioScorer.weight = 5;
        goldAlignDetector.ratioScorer.perfectRatio = 1.0;

        goldAlignDetector.enable();
    }
}
