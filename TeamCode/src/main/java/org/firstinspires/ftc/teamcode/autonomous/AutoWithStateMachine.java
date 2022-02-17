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
public class AutoWithStateMachine extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        GoToBasketTowerLevelMiddle,   // First, follow a splineTo() trajectory
        OutakeBlocksMiddleOne,
        GoBackToStartLevelMiddle,
        GoToWearhouseLevelMiddle,
        GoBackToStartLevelMiddleTwo,
        goToBasketTowerLevelMiddleTwo,
        OutakeBlocksMiddleTwo,
        GoBackToStartLevelMiddle3,
        GoToWearhouseLevelMiddle3,
        GoBackToStartLevelMiddleTwo3,
        goToBasketTowerLevelMiddle3,
        OutakeBlocksMiddle3,
        GoBackToStartLevelMiddleFinal,
        GoToWearhouseLevelMiddleFinal,
        GoToMiddleEndPositionOne,
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
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

    public AutoWithStateMachine(int TeamColor, boolean SlideToSide) {
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
        Trajectory goToBasketTowerLevelMiddle = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-8, -55 * teamColor))
                .build();
        Trajectory GoBackToStartLevelMiddle = drive.trajectoryBuilder(goToBasketTowerLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelMiddle = drive.trajectoryBuilder(GoBackToStartLevelMiddle.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelMiddleTwo = drive.trajectoryBuilder(GoToWearhouseLevelMiddle.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelMiddleTwo = drive.trajectoryBuilder(GoBackToStartLevelMiddleTwo.end())
                .lineToLinearHeading(new Pose2d(-8, -51 *teamColor, Math.toRadians(90*teamColor)))
                .build();

        Trajectory GoBackToStartLevelMiddle3 = drive.trajectoryBuilder(goToBasketTowerLevelMiddleTwo.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoToWearhouseLevelMiddle3 = drive.trajectoryBuilder(GoBackToStartLevelMiddle3.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(50, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory GoBackToStartLevelMiddleTwo3 = drive.trajectoryBuilder(GoToWearhouseLevelMiddle3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -71*teamColor, Math.toRadians(0)))
                .build();
        Trajectory goToBasketTowerLevelMiddle3 = drive.trajectoryBuilder(GoBackToStartLevelMiddleTwo3.end())
                .lineToLinearHeading(new Pose2d(-8, -51 *teamColor, Math.toRadians(90*teamColor)))
                .build();

        Trajectory GoBackToStartLevelMiddleFinal = drive.trajectoryBuilder(goToBasketTowerLevelMiddle3.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(9, -71*teamColor, Math.toRadians(0)))
                .addTemporalMarker(0.5, () ->{

                    //drive.ArmLifter(-1,4);

                })
                .build();
        Trajectory GoToWearhouseLevelMiddleFinal = drive.trajectoryBuilder(GoBackToStartLevelMiddleFinal.end())
                //.splineToConstantHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(40, -71*teamColor, Math.toRadians(0)))
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

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
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

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.GoToBasketTowerLevelMiddle;
        drive.followTrajectoryAsync(goToBasketTowerLevelMiddle);
        int levelArmShouldGoTo = 2;
        drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
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
                    currentState = State.GoBackToStartLevelMiddle;
                    levelArmShouldGoTo = -1;
                    drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(GoBackToStartLevelMiddle);

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
                    currentState = State.GoBackToStartLevelMiddle3;
                    levelArmShouldGoTo = -1;
                    drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(GoBackToStartLevelMiddle3);

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
                    currentState = State.GoBackToStartLevelMiddleFinal;
                    levelArmShouldGoTo = -1;
                    drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    drive.followTrajectoryAsync(GoBackToStartLevelMiddleFinal);

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
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
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
