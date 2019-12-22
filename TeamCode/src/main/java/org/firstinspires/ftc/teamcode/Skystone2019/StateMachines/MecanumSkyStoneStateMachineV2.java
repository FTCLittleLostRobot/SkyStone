/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import android.graphics.Bitmap;
import com.qualcomm.robotcore.robot.RobotState;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumSkyStoneStateMachineV2 {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    ColorFinder colorFinder;
    MecanumSkyStoneStateMachineV2.RobotState state;
    SetUpStateMachine setUpStateMachine;
    IConfiguration robotConfig;
    MecanumMotor motors;
    private CoreHexStateMachine coreHexStateMachineBlockGrabber;
    private CoreHexStateMachine coreHexStateMachineBlockLifter;
    private MecanumRotateStateMachine mecanumRotateStateMachine;
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;

    // THIS IS IF THE ROBOT IS FACING FORWARDS
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_SPEED = 0.25;
    static final double GO_FORWARD = -1;
    static final double GO_BACK = 1;
    static final double GO_RIGHT = -1;
    static final double GO_LEFT = 1;
    private int DistanceUnderBridge = 33;
    private Image vuforiaImageObject;
    private Bitmap bitmapFromVuforia;
    int foundColumn = -1;
    public int skyStonePosition = -1;
    private boolean SecondSkyStone = false;
    private int origSpeed = 50;
    private int robotHeading = 0;
    private boolean RedTeam;
    private boolean EndWall;
    private boolean TwoSkystones;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum RobotState
    {
        Start,
        MoveForwards,
        MovingForwards,

        CheckScreen,
        ConvertImageFromScreen,
        DetectColorFromImage,

        WaitForStrafe,


        MoveTowardsBlock1,
        MovingTowardsBlock1,
        FaceFront,
        CheckForSkystone,
        MovingTowardsBlock,
        GrabBlock,
        GrabbingBlock,
        LiftBlock,
        LiftingBlock,
        ReOrientate,
        ReOrienting,
        SpinAfterReOrientate,
        SpinAfterReOrienting,

        MoveUnderSkyBridge1,
        MovingUnderSkyBridge1,
        DropBlock,
        DroppingBlock,
        StraightenBeforeBackup,
        StraighteningBeforeBackup,
        BackUp1,
        BackingUp1,
        SquareAgainstWall,
        SquareingAgainstWall,
        MoveInfrontOfBlock,
        MovingInfrontOfBlock,
        FaceFront2,
        FacingFront2,

        ParkUnderBridge,
        ParkingUnderBridge,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ColorFinder colorFinder, GyroController gyro, boolean EndByWall, boolean isRed, boolean TwoSkystones, HardwareMecanumBase robot ) {

        this.telemetry = telemetry;
        this.colorFinder = colorFinder;

        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);
        this.motors = motors;
        this.coreHexStateMachineBlockGrabber = new CoreHexStateMachine();
        this.coreHexStateMachineBlockGrabber.init(telemetry, robot, CoreHex.CoreHexMotors.BlockGrabber);
        this.coreHexStateMachineBlockLifter = new CoreHexStateMachine();
        this.coreHexStateMachineBlockLifter.init(telemetry, robot, CoreHex.CoreHexMotors.BlockLifter);
        this.mecanumRotateStateMachine = new MecanumRotateStateMachine();
        this.mecanumRotateStateMachine.init(telemetry, motors, gyro);
        this.RedTeam = isRed;
        this.EndWall = EndByWall;
        this.robotHeading = gyro.GetCurrentRobotHeading();
        this.TwoSkystones = TwoSkystones;
        state = MecanumSkyStoneStateMachineV2.RobotState.Start;
    }

    public void Start()
    {
        this.origSpeed = this.motors.GetSpeedMultiplier();
        state = RobotState.MoveForwards;
    }

    private void CheckIfDone(RobotState nextState) {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
            this.motors.SetSpeedToValue(origSpeed);
            state = nextState;
        }
    }

    private void CheckIfDoneRotating(RobotState nextState) {
        if (this.mecanumRotateStateMachine.IsDone()) {
            state = nextState;
        }
        else
        {
            this.mecanumRotateStateMachine.ProcessState();
        }
    }
    public boolean IsDone()
    {
        return (state == MecanumSkyStoneStateMachineV2.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());
        telemetry.addData ("Found In", this.foundColumn);
        telemetry.addData("Column 0", colorFinder.column0);
        telemetry.addData("Column 1", colorFinder.column1);
        telemetry.addData("Column 2", colorFinder.column2);
        telemetry.addData("targetLeftFrontEncoderValue", moveRobot.targetLeftFrontEncoderValue);
        telemetry.addData("CurrentLeftFrontPosition", moveRobot.GetLeftMotorEncodePosition());
        telemetry.update();


        switch (state)
        {
            case MoveForwards:
                holdTimer.reset();
                this.moveRobot.StartMoveNoPid(40, 18, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingForwards;
                break;

            case MovingForwards:
                if (holdTimer.time() >= 1000) {
                    state = RobotState.CheckScreen;
                }
                break;

            case CheckScreen:
                try {
                    vuforiaImageObject = colorFinder.getVuforiaImagefromFrame();
                }
                catch(InterruptedException ex){
                    telemetry.addData("error", ex.getMessage());
                    break;
                }
                state = RobotState.ConvertImageFromScreen;
                break;

            case ConvertImageFromScreen:
                try {
                    bitmapFromVuforia = colorFinder.getBitmapToAnalyze(vuforiaImageObject);
                }
                catch(InterruptedException ex) {
                    telemetry.addData("error", ex.getMessage());
                    break;
                }
                state = RobotState.DetectColorFromImage;
                break;

            case DetectColorFromImage:
                foundColumn = colorFinder.FindColor(bitmapFromVuforia, ColorFinder.ColorTarget.Black, telemetry);
                telemetry.addData("column", foundColumn);
                this.CheckIfDone(RobotState.CheckForSkystone);
                break;

            case CheckForSkystone:
                if (foundColumn == 2 )
                {
                    this.moveRobot.StartMove(30, 9, -1 , -0.7, 0 );
                    state = RobotState.WaitForStrafe;
                    skyStonePosition = 2;
                    if (RedTeam == true){
                        this.DistanceUnderBridge = DistanceUnderBridge + 8; //DO NOT CHANGE
                    }
                    else {
                        this.DistanceUnderBridge = DistanceUnderBridge + 28; //DO NOT CHANGE
                    }
                }
                else if (foundColumn == 1 )
                {
                    skyStonePosition = 1;
                    this.moveRobot.StartMove(30, 10, 0 , GO_FORWARD, 0 );
                    this.DistanceUnderBridge = DistanceUnderBridge + 16;
                    state = RobotState.WaitForStrafe;

                }
                else if (foundColumn == 0)
                {
                    this.moveRobot.StartMove(40, 9, 1 , -0.75, 0 );
                    state = RobotState.WaitForStrafe;
                    skyStonePosition = 0;
                    if (RedTeam == true){
                        this.DistanceUnderBridge = DistanceUnderBridge + 28;
                    }
                    else {
                        this.DistanceUnderBridge = DistanceUnderBridge + 8;

                    }

                }
                break;

            case WaitForStrafe:
                this.CheckIfDone(RobotState.GrabBlock);
                break;

            case GrabBlock:
                holdTimer.reset();
                // coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Down);
                // coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.GrabbingBlock;
                break;

            case GrabbingBlock:
                if ((holdTimer.time() >= 1500)) {
                    state = RobotState.ReOrientate;
                }
                else {
                    //  coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;

            case ReOrientate:
                if (EndWall == true){
                    this.moveRobot.StartMove(30, 28, 0, GO_BACK, 0);
                    state = RobotState.ReOrienting;
                }
                else {
                    this.moveRobot.StartMove(30, 8, 0, GO_BACK, 0);
                    state = RobotState.ReOrienting;
                }
                break;

            case ReOrienting:
                this.CheckIfDone(RobotState.SpinAfterReOrientate);
                break;
            case SpinAfterReOrientate:
                if (RedTeam == true) {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 30);
                    state = RobotState.SpinAfterReOrienting;
                }
                else {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 30);
                    state = RobotState.SpinAfterReOrienting;
                }

                break;

            case SpinAfterReOrienting:
                this.CheckIfDoneRotating(RobotState.MoveUnderSkyBridge1);
                break;

            case MoveUnderSkyBridge1:
                moveRobot.StartMove(45, DistanceUnderBridge, 0, GO_FORWARD, 0);
                state = RobotState.MovingUnderSkyBridge1;
                break;

            case MovingUnderSkyBridge1:
                this.CheckIfDone(RobotState.StraightenBeforeBackup);
                break;

            case StraightenBeforeBackup:
                if (RedTeam == true){
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 20);
                }
                else {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 20);
                }
                state = RobotState.StraighteningBeforeBackup;
                break;
            case StraighteningBeforeBackup:
                this.CheckIfDoneRotating(RobotState.DropBlock);
                break;

///////////////////////////////
            case DropBlock:
                holdTimer.reset();
                // coreHexStateMachineBlockGrabber.ReleaseGrip();
                // coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.PositionZeroGrabber);
                // coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
                //coreHexStateMachineBlockLifter.ProcessState();
                // coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.DroppingBlock;
            case DroppingBlock:
                if ((holdTimer.time() >= 1000)) {
                    if (TwoSkystones && SecondSkyStone == false) {
                        state = RobotState.BackUp1;
                    }
                    else{
                        state = RobotState.ParkUnderBridge;
                    }
                }
                break;


            //For the 2nd competition we decide to just do one skystone so for now after it straightens up it will go to park under bridge
            case BackUp1:
                moveRobot.StartMove(45,70, 0, GO_BACK, 0);
                state = RobotState.BackingUp1;
                break;

            case BackingUp1:
                this.CheckIfDone(RobotState.SquareAgainstWall);
                break;
            case SquareAgainstWall:
                holdTimer.reset();
                moveRobot.StartMove(10,20, 0, GO_BACK, 0);
                state = RobotState.SquareingAgainstWall;
                break;
            case SquareingAgainstWall:
                if (holdTimer.time() >= 2000){
                    this.moveRobot.Complete();
                    state = RobotState.MoveInfrontOfBlock;
                }
                break;

            case MoveInfrontOfBlock:
                DistanceUnderBridge = DistanceUnderBridge + 24;
                SecondSkyStone = true;
                moveRobot.StartMove(10,6, 0, GO_FORWARD, 0);
                state = RobotState.MovingInfrontOfBlock;
                break;

            case MovingInfrontOfBlock:
                this.CheckIfDone(RobotState.FaceFront2);
                break;

            case FaceFront2:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading), 30);
                state = RobotState.FacingFront2;

                break;
            case FacingFront2:
                this.CheckIfDoneRotating(RobotState.MoveTowardsBlock1);
                break;

            case MoveTowardsBlock1:
                double distanceToSecondBlock = motors.DistanceSensor.getDistance(DistanceUnit.INCH);
                if (skyStonePosition == 2){
                    this.moveRobot.StartMove(30, distanceToSecondBlock , -1 , -0.7, 0 );
                    state = RobotState.MovingTowardsBlock1;
                }
                else if (skyStonePosition == 1){
                    this.moveRobot.StartMove(30, distanceToSecondBlock, 0 , GO_FORWARD, 0 );
                    state = RobotState.MovingTowardsBlock1;
                }
                else if (skyStonePosition == 0){
                    this.moveRobot.StartMove(40, distanceToSecondBlock, 1 , -0.75, 0 );
                    state = RobotState.MovingTowardsBlock1;
                }
                break;
            case MovingTowardsBlock1:
                this.CheckIfDone(RobotState.GrabBlock);
                break;
            case ParkUnderBridge:
                moveRobot.StartMove(35,14, 0, GO_BACK, 0);
                coreHexStateMachineBlockLifter.ProcessState();
                // coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.ParkingUnderBridge;
                break;
            case ParkingUnderBridge:
                this.CheckIfDone(RobotState.Done);
                break;
            //ThatOneBoi

        }

    }
}
