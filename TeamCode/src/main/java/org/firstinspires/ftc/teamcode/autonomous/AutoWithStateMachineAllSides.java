package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive6340;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
@Disabled
public class AutoWithStateMachineAllSides extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
// region STATES FOR LEVEL MIDDLE
        IDLE,       
        GoToBasketTowerLevelMiddle,   // First, follow a splineTo() trajectory
        OutakeBlocksMiddleOne,
        SlideBackFromTowerLevelMiddle1,
        GoBackToStartLevelMiddle,
        GoToWearhouseLevelMiddle,
        GoBackToStartLevelMiddleTwo,
        goToBasketTowerLevelMiddleTwo,
        OutakeBlocksMiddleTwo,
        SlideBackFromTowerLevelMiddle2,
        GoBackToStartLevelMiddle3,
        GoToWearhouseLevelMiddle3,
        GoBackToStartLevelMiddleTwo3,
        goToBasketTowerLevelMiddle3,
        OutakeBlocksMiddle3,
        SlideBackFromTowerLevelMiddle3,
        GoBackToStartLevelMiddleFinal,
        GoToWearhouseLevelMiddleFinal,
        GoToMiddleEndPositionOne,
        //endregion STATES FOR LEVEL MIDDLE        
   
        //region STATES FOR LEVEL HIGH
        GoToBasketTowerLevelHigh,   // First, follow a splineTo() trajectory
        OutakeBlocksHighOne,
        SlideBackFromTowerLevelHigh1,
        GoBackToStartLevelHigh,
        GoToWearhouseLevelHigh,
        GoBackToStartLevelHighTwo,
        goToBasketTowerLevelHighTwo,
        OutakeBlocksHighTwo,
        SlideBackFromTowerLevelHigh2,
        GoBackToStartLevelHigh3,
        GoToWearhouseLevelHigh3,
        GoBackToStartLevelHighTwo3,
        goToBasketTowerLevelHigh3,
        OutakeBlocksHigh3,
        SlideBackFromTowerLevelHigh3,
        GoBackToStartLevelHighFinal,
        GoToWearhouseLevelHighFinal,
        GoToHighEndPositionOne,
        //endregion STATES FOR LEVEL HIGH

        //region STATES FOR LEVEL Low
        GoToBasketTowerLevelLow,   // First, follow a splineTo() trajectory
        OutakeBlocksLowOne,
        SlideBackFromTowerLevelLow1,
        GoBackToStartLevelLow,
        GoToWearhouseLevelLow,
        GoBackToStartLevelLowTwo,
        goToBasketTowerLevelLowTwo,
        OutakeBlocksLowTwo,
        SlideBackFromTowerLevelLow2,
        GoBackToStartLevelLow3,
        GoToWearhouseLevelLow3,
        GoBackToStartLevelLowTwo3,
        goToBasketTowerLevelLow3,
        OutakeBlocksLow3,
        SlideBackFromTowerLevelLow3,
        GoBackToStartLevelLowFinal,
        GoToWearhouseLevelLowFinal,
        GoToLowEndPositionOne,
        //endregion STATES FOR LEVEL Low

    }
    private int teamColor;//1=Red -1= Blue
    private boolean slideToSide = false;

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

    public AutoWithStateMachineAllSides(int TeamColor, boolean SlideToSide) {
        super();
        teamColor = TeamColor;
        slideToSide = SlideToSide;

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize MecanumDrive6340
        MecanumDrive6340 drive = new MecanumDrive6340(hardwareMap);
        // Define our start pose
        // This assumes we start at x: 15, y: 10, heading: 180 degrees
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff(vuforia);

        Pose2d startPose = new Pose2d(12, -72 * teamColor, Math.toRadians(90 * teamColor));

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        //region TRAJECTORIES FOR LEVEL MIDDLE
        Trajectory goToBasketTowerLevelMiddle = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-8, -55.5 * teamColor))
                .build();
        Trajectory SlideBackFromTowerLevelMiddle1 = drive.trajectoryBuilder(goToBasketTowerLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelMiddle = drive.trajectoryBuilder(SlideBackFromTowerLevelMiddle1.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelMiddle = drive.trajectoryBuilder(GoBackToStartLevelMiddle.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelMiddleTwo = drive.trajectoryBuilder(GoToWearhouseLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelMiddleTwo = drive.trajectoryBuilder(GoBackToStartLevelMiddleTwo.end())
                .lineToLinearHeading(new Pose2d(-8, -50 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelMiddle2 = drive.trajectoryBuilder(goToBasketTowerLevelMiddleTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();

        Trajectory GoBackToStartLevelMiddle3 = drive.trajectoryBuilder(SlideBackFromTowerLevelMiddle2.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelMiddle3 = drive.trajectoryBuilder(GoBackToStartLevelMiddle3.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(53, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelMiddleTwo3 = drive.trajectoryBuilder(GoToWearhouseLevelMiddle3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelMiddle3 = drive.trajectoryBuilder(GoBackToStartLevelMiddleTwo3.end())
                .lineToLinearHeading(new Pose2d(-8, -49 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelMiddle3 = drive.trajectoryBuilder(goToBasketTowerLevelMiddle3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelMiddleFinal = drive.trajectoryBuilder(SlideBackFromTowerLevelMiddle3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelMiddleFinal = drive.trajectoryBuilder(GoBackToStartLevelMiddleFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToMiddleEndPositionOne = drive.trajectoryBuilder(GoToWearhouseLevelMiddleFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -40*teamColor, Math.toRadians(0*teamColor)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(goToBasketTowerLevelMiddle.end())
                .lineTo(new Vector2d(45, 0))
                .build();
        //endregion TRAJECTORIES FOR LEVEL MIDDLE


        //region TRAJECTORIES FOR LEVEL High
        Trajectory goToBasketTowerLevelHigh = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-8, -51 * teamColor))
                .build();
        Trajectory SlideBackFromTowerLevelHigh1 = drive.trajectoryBuilder(goToBasketTowerLevelHigh.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelHigh = drive.trajectoryBuilder(SlideBackFromTowerLevelHigh1.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelHigh = drive.trajectoryBuilder(GoBackToStartLevelHigh.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelHighTwo = drive.trajectoryBuilder(GoToWearhouseLevelHigh.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelHighTwo = drive.trajectoryBuilder(GoBackToStartLevelHighTwo.end())
                .lineToLinearHeading(new Pose2d(-8, -50 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelHigh2 = drive.trajectoryBuilder(goToBasketTowerLevelHighTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();

        Trajectory GoBackToStartLevelHigh3 = drive.trajectoryBuilder(SlideBackFromTowerLevelHigh2.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelHigh3 = drive.trajectoryBuilder(GoBackToStartLevelHigh3.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(53, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelHighTwo3 = drive.trajectoryBuilder(GoToWearhouseLevelHigh3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelHigh3 = drive.trajectoryBuilder(GoBackToStartLevelHighTwo3.end())
                .lineToLinearHeading(new Pose2d(-8, -49 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelHigh3 = drive.trajectoryBuilder(goToBasketTowerLevelHigh3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelHighFinal = drive.trajectoryBuilder(SlideBackFromTowerLevelHigh3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelHighFinal = drive.trajectoryBuilder(GoBackToStartLevelHighFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToHighEndPositionOne = drive.trajectoryBuilder(GoToWearhouseLevelHighFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -40*teamColor, Math.toRadians(0*teamColor)))
                .build();

        //endregion TRAJECTORIES FOR LEVEL High 

        //region TRAJECTORIES FOR LEVEL Low
        Trajectory goToBasketTowerLevelLow = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-8, -55 * teamColor))
                .build();
        Trajectory SlideBackFromTowerLevelLow1 = drive.trajectoryBuilder(goToBasketTowerLevelLow.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelLow = drive.trajectoryBuilder(SlideBackFromTowerLevelLow1.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelLow = drive.trajectoryBuilder(GoBackToStartLevelLow.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelLowTwo = drive.trajectoryBuilder(GoToWearhouseLevelLow.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelLowTwo = drive.trajectoryBuilder(GoBackToStartLevelLowTwo.end())
                .lineToLinearHeading(new Pose2d(-8, -50 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelLow2 = drive.trajectoryBuilder(goToBasketTowerLevelLowTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();

        Trajectory GoBackToStartLevelLow3 = drive.trajectoryBuilder(SlideBackFromTowerLevelLow2.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelLow3 = drive.trajectoryBuilder(GoBackToStartLevelLow3.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(53, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelLowTwo3 = drive.trajectoryBuilder(GoToWearhouseLevelLow3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelLow3 = drive.trajectoryBuilder(GoBackToStartLevelLowTwo3.end())
                .lineToLinearHeading(new Pose2d(-8, -49 *teamColor, Math.toRadians(90*teamColor)))
                .build();
        Trajectory SlideBackFromTowerLevelLow3 = drive.trajectoryBuilder(goToBasketTowerLevelLow3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineTo(new Vector2d(-8, -60*teamColor))
                .build();
        Trajectory GoBackToStartLevelLowFinal = drive.trajectoryBuilder(SlideBackFromTowerLevelLow3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -72*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelLowFinal = drive.trajectoryBuilder(GoBackToStartLevelLowFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -72*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToLowEndPositionOne = drive.trajectoryBuilder(GoToWearhouseLevelLowFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -40*teamColor, Math.toRadians(0*teamColor)))
                .build();

        //endregion TRAJECTORIES FOR LEVEL Low 

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
        VuforiaStuff.capElementPositionData posData = null;
        drive.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VuforiaStuff.capElementPos pos = null;

        waitForStart();
        drive.ArmLifter(0,4);
        posData = vuforiaStuff.vuforiascan(true, true);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        pos = posData.capElementPosition;


        if (opModeIsActive()) {
/*
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time() < 2.0) {
                telemetry.addData("Position", pos);
                telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
                telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
                telemetry.addData("RightYellowCount", posData.yellowCountRight);


                telemetry.update();
            }
  */
            telemetry.addData("Position", pos);
            telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
            telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
            telemetry.addData("RightYellowCount", posData.yellowCountRight);


            telemetry.update();
        }

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        int levelArmShouldGoTo = 0;

         if (pos == VuforiaStuff.capElementPos.CENTER) {
             currentState = State.GoToBasketTowerLevelMiddle;
             drive.followTrajectoryAsync(goToBasketTowerLevelMiddle);
             levelArmShouldGoTo = 2;
             drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
         }
         if(pos ==VuforiaStuff.capElementPos.RIGHT) {
             currentState = State.GoToBasketTowerLevelHigh;
             drive.followTrajectoryAsync(goToBasketTowerLevelHigh);
             levelArmShouldGoTo = 4;
             drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
         }
         if(pos == VuforiaStuff.capElementPos.LEFT) {
             currentState = State.GoToBasketTowerLevelLow;
             drive.followTrajectoryAsync(goToBasketTowerLevelLow);
             levelArmShouldGoTo = 1;
             drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
         }


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
//region States For Level Middle
                case GoToBasketTowerLevelMiddle:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksMiddleOne;

                       // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case OutakeBlocksMiddleOne:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelMiddle1;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelMiddle1);

                    break;
                case SlideBackFromTowerLevelMiddle1:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelMiddle;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelMiddle);
                    }
                    break;
                case GoBackToStartLevelMiddle:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelMiddle;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelMiddle);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelMiddle:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelMiddleTwo;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelMiddleTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelMiddleTwo:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelMiddleTwo;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelMiddleTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelMiddleTwo:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksMiddleTwo;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelMiddleTwo);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksMiddleTwo:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelMiddle2;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelMiddle2);

                    break;
                case SlideBackFromTowerLevelMiddle2:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelMiddle3;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelMiddle3);
                    }
                    break;

                //THIRD BLOCK CODE START
                case GoBackToStartLevelMiddle3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelMiddle3;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelMiddle3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelMiddle3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelMiddleTwo3;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelMiddleTwo3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelMiddleTwo3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelMiddle3;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelMiddle3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelMiddle3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksMiddle3;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelMiddle3Two);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksMiddle3:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelMiddle3;
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelMiddle3);

                    break;
                case SlideBackFromTowerLevelMiddle3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelMiddleFinal;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelMiddleFinal);
                    }
                    break;

                //THIRD BLOCK CODE END
                case GoBackToStartLevelMiddleFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelMiddleFinal;
                        drive.followTrajectoryAsync(GoToWearhouseLevelMiddleFinal);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelMiddleFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        if(slideToSide) {
                            currentState = State.GoToMiddleEndPositionOne;
                            drive.followTrajectoryAsync(GoToMiddleEndPositionOne);
                        }
                        else
                        {
                            currentState = State.IDLE;
                        }
                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToMiddleEndPositionOne:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
//endregion States For Level Middle                    

//region States For Level High
                case GoToBasketTowerLevelHigh:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksHighOne;

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case OutakeBlocksHighOne:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelHigh1;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelHigh1);

                    break;
                case SlideBackFromTowerLevelHigh1:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelHigh;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelHigh);
                    }
                    break;
                case GoBackToStartLevelHigh:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelHigh;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelHigh);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelHigh:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelHighTwo;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelHighTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelHighTwo:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelHighTwo;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelHighTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelHighTwo:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksHighTwo;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelHighTwo);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksHighTwo:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelHigh2;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelHigh2);

                    break;
                case SlideBackFromTowerLevelHigh2:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelHigh3;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelHigh3);
                    }
                    break;

                //THIRD BLOCK CODE START
                case GoBackToStartLevelHigh3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelHigh3;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelHigh3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelHigh3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelHighTwo3;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelHighTwo3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelHighTwo3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelHigh3;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelHigh3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelHigh3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksHigh3;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelHigh3Two);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksHigh3:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelHigh3;
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelHigh3);

                    break;
                case SlideBackFromTowerLevelHigh3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelHighFinal;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelHighFinal);
                    }
                    break;

                //THIRD BLOCK CODE END
                case GoBackToStartLevelHighFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelHighFinal;
                        drive.followTrajectoryAsync(GoToWearhouseLevelHighFinal);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelHighFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        if(slideToSide) {
                            currentState = State.GoToHighEndPositionOne;
                            drive.followTrajectoryAsync(GoToHighEndPositionOne);
                        }
                        else
                        {
                            currentState = State.IDLE;
                        }
                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToHighEndPositionOne:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
//endregion States For Level High

//region States For Level Low
                case GoToBasketTowerLevelLow:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksLowOne;

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case OutakeBlocksLowOne:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelLow1;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelLow1);

                    break;
                case SlideBackFromTowerLevelLow1:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelLow;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelLow);
                    }
                    break;
                case GoBackToStartLevelLow:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelLow;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelLow);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelLow:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelLowTwo;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelLowTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelLowTwo:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelLowTwo;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelLowTwo);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelLowTwo:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksLowTwo;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelLowTwo);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksLowTwo:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelLow2;
                    //levelArmShouldGoTo = -1;
                    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelLow2);

                    break;
                case SlideBackFromTowerLevelLow2:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelLow3;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelLow3);
                    }
                    break;

                //THIRD BLOCK CODE START
                case GoBackToStartLevelLow3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelLow3;
                        drive.inTakeblocks();
                        drive.followTrajectoryAsync(GoToWearhouseLevelLow3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelLow3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelLowTwo3;
                        drive.stopIntakeBlocks();
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelLowTwo3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoBackToStartLevelLowTwo3:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() ) {
                        currentState = State.goToBasketTowerLevelLow3;
                        //levelArmShouldGoTo = 4;
                        //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelLow3);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case goToBasketTowerLevelLow3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.OutakeBlocksLow3;
                       /*
                        levelArmShouldGoTo = 4;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(goToBasketTowerLevelLow3Two);

                        // drive.followTrajectoryAsync(trajectory2);

                        */
                    }
                    break;
                case OutakeBlocksLow3:
                    drive.outTakeblocks();
                    sleep(1000);
                    drive.stopIntakeBlocks();
                    currentState = State.SlideBackFromTowerLevelLow3;
                    drive.followTrajectoryAsync(SlideBackFromTowerLevelLow3);

                    break;
                case SlideBackFromTowerLevelLow3:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoBackToStartLevelLowFinal;
                        levelArmShouldGoTo = -1;
                        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        drive.followTrajectoryAsync(GoBackToStartLevelLowFinal);
                    }
                    break;

                //THIRD BLOCK CODE END
                case GoBackToStartLevelLowFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.GoToWearhouseLevelLowFinal;
                        drive.followTrajectoryAsync(GoToWearhouseLevelLowFinal);

                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToWearhouseLevelLowFinal:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        if(slideToSide) {
                            currentState = State.GoToLowEndPositionOne;
                            drive.followTrajectoryAsync(GoToLowEndPositionOne);
                        }
                        else
                        {
                            currentState = State.IDLE;
                        }
                        // drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case GoToLowEndPositionOne:
                    if (!drive.isBusy() && !drive.IsArmLifterBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
//endregion States For Level Low

            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("Position", pos);
            telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
            telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
            telemetry.addData("RightYellowCount", posData.yellowCountRight);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ArmPosition",drive.ArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
}
