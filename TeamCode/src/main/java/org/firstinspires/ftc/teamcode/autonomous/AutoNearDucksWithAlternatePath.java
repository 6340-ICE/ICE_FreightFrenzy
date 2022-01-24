/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoNearDucksWithAlternatePath", group = "Autonomous")
@Disabled

public class AutoNearDucksWithAlternatePath extends LinearOpMode {
    private int teamColor;//1=Red -1= Blue

    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String targetZone = "D";
    private static final String VUFORIA_KEY =
            "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";
    private VuforiaLocalizer vuforia;
    public VuforiaStuff vuforiaStuff;
    private TFObjectDetector tfod;

    public AutoNearDucksWithAlternatePath(int TeamColor) {
        super();
        teamColor = TeamColor;
    }

    MecanumDrive6340 drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive6340(hardwareMap);
        //DetectionLogicForCapstone
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);

        Pose2d startPose = new Pose2d(-36, -72 * teamColor, Math.toRadians(90 * teamColor));

        drive.setPoseEstimate(startPose);
/*
        Trajectory goForwardFirst = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d( -36,-71), Math.toRadians(90))
                .build();

        Trajectory goToDuck = drive.trajectoryBuilder(goForwardFirst.end())
                .splineToConstantHeading(new Vector2d( -65,-71), Math.toRadians(90))
                .build();
*/
        Trajectory goToBasketTowerStrafe = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-12, -72 * teamColor))
                .build();

//Level Middle
        Trajectory goToDuckMiddleRed = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-59, -66.5 * teamColor, Math.toRadians(0*teamColor)),
                        drive.getVelocityConstraint(15.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory goToZeroMiddleRed = drive.trajectoryBuilder(goToDuckMiddleRed.end())
                .strafeTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToDuckMiddleBlue = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-60, -65 * teamColor))
                .build();
        Trajectory goToZeroMiddleBlue = drive.trajectoryBuilder(goToDuckMiddleBlue.end())
                .lineToLinearHeading(new Pose2d(-68, -30 * teamColor, Math.toRadians(0*teamColor)))
                .build();
        Trajectory tempMiddle;
        if(teamColor == 1){
            tempMiddle = goToZeroMiddleRed;
        }
        else{
            tempMiddle = goToZeroMiddleBlue;
        }
        Trajectory goToBasketTowerLevelMiddle = drive.trajectoryBuilder(tempMiddle.end())
                .lineTo(new Vector2d(-36.5, -30 * teamColor))
                .build();
        Trajectory goBackFromBasketTowerLevelMiddle = drive.trajectoryBuilder(goToBasketTowerLevelMiddle.end())
                .lineTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToStorageMiddle = drive.trajectoryBuilder(goBackFromBasketTowerLevelMiddle.end())
                .strafeTo(new Vector2d(-68, -46 * teamColor))
                .build();
        //remember to change x value for goToStorageMiddle.

//Level High

        Trajectory goToDuckHighRed = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-59, -66.5 * teamColor, Math.toRadians(0*teamColor)),
                        drive.getVelocityConstraint(15.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory goToZeroHighRed = drive.trajectoryBuilder(goToDuckHighRed.end())
                .strafeTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToDuckHighBlue = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-60, -65 * teamColor))
                .build();
        Trajectory goToZeroHighBlue = drive.trajectoryBuilder(goToDuckHighBlue.end())
                .lineToLinearHeading(new Pose2d(-68, -30 * teamColor, Math.toRadians(0*teamColor)))
                .build();
        Trajectory tempHigh;
        if(teamColor == 1){
            tempHigh = goToZeroHighRed;
        }
        else{
            tempHigh = goToZeroHighBlue;
        }
        Trajectory goToBasketTowerLevelHigh = drive.trajectoryBuilder(tempHigh.end())
                .lineTo(new Vector2d(-34, -30 * teamColor))
                .build();
        Trajectory goBackFromBasketTowerLevelHigh = drive.trajectoryBuilder(goToBasketTowerLevelHigh.end())
                .lineTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToStorageHigh = drive.trajectoryBuilder(goBackFromBasketTowerLevelHigh.end())
                .strafeTo(new Vector2d(-68, -46 * teamColor))
                .build();

//Level Low

        Trajectory goToDuckLowRed = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-59, -66.5 * teamColor, Math.toRadians(0*teamColor)),
                        drive.getVelocityConstraint(15.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory goToZeroLowRed = drive.trajectoryBuilder(goToDuckLowRed.end())
                .strafeTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToDuckLowBlue = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-60, -65 * teamColor))
                .build();
        Trajectory goToZeroLowBlue = drive.trajectoryBuilder(goToDuckLowBlue.end())
                .lineToLinearHeading(new Pose2d(-68, -30 * teamColor, Math.toRadians(0*teamColor)))
                .build();
        Trajectory tempLow;
        if(teamColor == 1){
            tempLow = goToZeroLowRed;
        }
        else{
            tempLow = goToZeroLowBlue;
        }
        Trajectory goToBasketTowerLevelLow = drive.trajectoryBuilder(tempLow.end())
                .lineTo(new Vector2d(-35, -30 * teamColor))
                .build();
        Trajectory goBackFromBasketTowerLevelLow = drive.trajectoryBuilder(goToBasketTowerLevelLow.end())
                .lineTo(new Vector2d(-68, -30 * teamColor))
                .build();
        Trajectory goToStorageLow = drive.trajectoryBuilder(goBackFromBasketTowerLevelLow.end())
                .strafeTo(new Vector2d(-68, -46 * teamColor))
                .build();
/*
        Trajectory goToWall = drive.trajectoryBuilder(gobackfromBasketTower.end())
                .splineToConstantHeading(new Vector2d( -12,-70*teamColor), Math.toRadians(0))
                .build();

        Trajectory goToShippingHub = drive.trajectoryBuilder(goToWall.end())
                .splineTo(new Vector2d( 36,-71*teamColor), Math.toRadians(0))
                .build();
*/
        /** Wait for the game to begin */
        drive.redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        drive.redLED1.setState(true);
        drive.redLED2.setState(true);
        drive.greenLED1.setState(false);
        drive.greenLED2.setState(false);
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Ready To Go Teammate. Let's Go ICE 6340!");
        telemetry.update();
        drive.redLED1.setState(false);
        drive.redLED2.setState(false);
        drive.greenLED1.setState(true);
        drive.greenLED2.setState(true);

        waitForStart();
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sleep(500);
        drive.ArmLifter(0,4);
        VuforiaStuff.capElementPositionData posData = null;
        posData = vuforiaStuff.vuforiascan(true, true);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        VuforiaStuff.capElementPos pos = null;
        pos = posData.capElementPosition;


        if (opModeIsActive()) {
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time() < 2.0) {
                telemetry.addData("Position", pos);
                telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
                telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
                telemetry.addData("RightYellowCount", posData.yellowCountRight);


                telemetry.update();
            }
        }


        //AUTONOMOUS STUFF RIGHT HERE
        targetZone = "D";
        //drive.shooter.setVelocity(1600);
        //   drive.followTrajectory(goForwardFirst);
        //drive.followTrajectory(goToBasketTowerStrafe);
       if(pos == VuforiaStuff.capElementPos.CENTER)
        {
            if(teamColor == 1.0) {
                drive.followTrajectory(goToDuckMiddleRed);
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroMiddleRed);
            }
            else{
                drive.followTrajectory(goToDuckMiddleBlue);
                drive.spinwheelright();
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroMiddleBlue);
            }

            LiftArmMiddle();
            drive.followTrajectory(goToBasketTowerLevelMiddle);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(goBackFromBasketTowerLevelMiddle);
            drive.ArmLifter(-1,4);
            drive.followTrajectory(goToStorageMiddle);
            sleep(500);




        }
        if(pos == VuforiaStuff.capElementPos.RIGHT)
        {
            if(teamColor == 1.0) {
                drive.followTrajectory(goToDuckHighRed);
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroHighRed);
            }
            else{
                drive.followTrajectory(goToDuckHighBlue);
                drive.spinwheelright();
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroHighBlue);
            }

            LiftArmHigh();
            drive.followTrajectory(goToBasketTowerLevelHigh);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(goBackFromBasketTowerLevelHigh);
            drive.ArmLifter(-1,4);
            drive.followTrajectory(goToStorageHigh);
            sleep(500);



        }
        if(pos == VuforiaStuff.capElementPos.LEFT)
        {
            if(teamColor == 1.0) {
                drive.followTrajectory(goToDuckLowRed);
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroLowRed);
            }
            else{
                drive.followTrajectory(goToDuckLowBlue);
                drive.spinwheelright();
                drive.spinwheel(teamColor);
                sleep(5000);
                drive.followTrajectory(goToZeroLowBlue);
            }

            LiftArmLow();
            drive.followTrajectory(goToBasketTowerLevelLow);
            drive.outTakeblocks();
            sleep(2000);
            drive.stopIntakeBlocks();
            drive.followTrajectory(goBackFromBasketTowerLevelLow);
            drive.ArmLifter(-1,4);
            drive.followTrajectory(goToStorageLow);
            sleep(500);


        }

//        LiftArm(pos);



        //      drive.followTrajectory(turnToZeroDegree);


        PoseStorage.currentPose = drive.getPoseEstimate();

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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void LiftArmBottom() {
        drive.ArmLifter(0,4);

    }
    private void LiftArmLow() {
        drive.ArmLifter(1,4);

    }

    private void
    LiftArmMiddle() {

        drive.ArmLifter(2,4);
    }

    private void LiftArmHigh() {
        drive.ArmLifter(3,4);


    }

    public void LiftArm(VuforiaStuff.capElementPos pos) {
        if (pos == VuforiaStuff.capElementPos.CENTER) {
            LiftArmMiddle();
        } else if (pos == VuforiaStuff.capElementPos.LEFT) {
            LiftArmLow();
        } else {
            LiftArmHigh();
        }

    }

}
