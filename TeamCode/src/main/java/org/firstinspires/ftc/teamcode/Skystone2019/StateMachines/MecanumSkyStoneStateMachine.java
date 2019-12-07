/* Little Lost Robots
   Core Devs: Danielle
*/
/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import android.graphics.Bitmap;
import com.qualcomm.robotcore.robot.RobotState;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumSkyStoneStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    ColorFinder colorFinder;
    MecanumSkyStoneStateMachine.RobotState state;
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
    private int DistanceUnderBridge = 57;
    private Image vuforiaImageObject;
    private Bitmap bitmapFromVuforia;
    int foundColumn = -1;
    public int skyStonePosition = -1;
    private boolean SecondSkyStone = true;
    private int InchesTowardsBlock = 25;
    private int origSpeed = 50;
    private int robotHeading = 0;
    private boolean RedTeam;
    private boolean EndWall;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum RobotState
    {
        Start,
        MoveForwards,
        MovingForwards,

        CheckScreen,
        ConvertImageFromScreen,
        DetectColorFromImage,

        Block0Position90Turn,
        Block0PositionGoLeft,
        Block0PositionGoingLeft,
        Block0Position90FaceFront,
        Block0Position90FaceingFront,
        Block1Position90Turn,
        Block1Position902ndturn,
        Block1Position902ndTurning,
        Block2Position90Turn,
        Block2PositionGoRight,
        Block2PositionGoingRight,
        Block2Position90FaceFront,
        Block2Position90FaceingFront,


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
        DropingBlock,
        ReleaseBlock,
        ReleasingBlock,
        StraightenBeforeBackup,
        StraighteningBeforeBackup,
        BackUp1,
        BackingUp1,
        MoveInfrontOfBlock,
        MovingInfrontOfBlock,
        FaceFront2,
        FacingFront2,

        ParkUnderBridge,
        ParkingUnderBridge,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ColorFinder colorFinder, GyroController gyro, boolean EndByWall, boolean isRed, HardwareMecanumBase robot ) {

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

        state = MecanumSkyStoneStateMachine.RobotState.Start;
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
        return (state == MecanumSkyStoneStateMachine.RobotState.Done);
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
                this.moveRobot.StartMove(30, 27, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.MovingForwards;
                break;

            case MovingForwards:
                this.CheckIfDone(RobotState.CheckScreen);
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
                foundColumn =  colorFinder.FindColor(bitmapFromVuforia, ColorFinder.ColorTarget.Black, telemetry);
                telemetry.addData("column", foundColumn);
                state = RobotState.CheckForSkystone;
                break;


            case CheckForSkystone:
                if (foundColumn == 2 )
                {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading +  90), 20);
                    state = RobotState.Block2Position90Turn;
                    skyStonePosition = 2;
                    this.DistanceUnderBridge = DistanceUnderBridge + 8; //DO NOT CHANGE
                }
                else if (foundColumn == 1 )
                {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                    skyStonePosition = 1;
                    this.DistanceUnderBridge = DistanceUnderBridge + 16;
                    this.InchesTowardsBlock = InchesTowardsBlock + 7;
                    state = RobotState.Block1Position90Turn;

                }
                else if (foundColumn == 0)
                {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 40);
                    state = RobotState.Block0Position90Turn;
                    skyStonePosition = 0;
                    this.DistanceUnderBridge = DistanceUnderBridge + 28;

                }
                break;

            case Block2Position90Turn:
                this.CheckIfDoneRotating(RobotState.Block2PositionGoRight);
                break;
            case Block2PositionGoRight:
                this.moveRobot.StartMove(40, 12, 0 , GO_FORWARD,0 );
                state = RobotState.Block2PositionGoingRight;
                break;
            case Block2PositionGoingRight:
                this.CheckIfDone(RobotState.Block2Position90FaceFront);
                break;
            case Block2Position90FaceFront:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 180), 40);
                state = RobotState.Block2Position90FaceingFront;
                break;
            case Block2Position90FaceingFront:
                this.CheckIfDoneRotating(RobotState.MoveTowardsBlock1);
                break;

            case Block1Position90Turn:
                this.CheckIfDoneRotating(RobotState.Block1Position902ndturn);
                break;
            case Block1Position902ndturn:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 180), 40);
                state = RobotState.Block1Position902ndTurning;
                break;
            case Block1Position902ndTurning:
                this.CheckIfDoneRotating(RobotState.MoveTowardsBlock1);
                break;

            case Block0Position90Turn:
                this.CheckIfDoneRotating(RobotState.Block0PositionGoLeft);
                break;

            case Block0PositionGoLeft:
                this.moveRobot.StartMove(40, 12, 0 , GO_FORWARD,0 );
                state = RobotState.Block0PositionGoingLeft;
                break;
            case Block0PositionGoingLeft:
                this.CheckIfDone(RobotState.Block0Position90FaceFront);
                break;
            case Block0Position90FaceFront:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 180), 40);
                state = RobotState.Block0Position90FaceingFront;
                break;
            case Block0Position90FaceingFront:
                this.CheckIfDoneRotating(RobotState.MoveTowardsBlock1);
                break;

            case MoveTowardsBlock1:
                this.moveRobot.StartMove(20, InchesTowardsBlock, 0 , GO_FORWARD,0 );
                state = RobotState.MovingTowardsBlock1;
                break;

            case MovingTowardsBlock1:
                this.CheckIfDone(RobotState.GrabBlock);
                break;

            case GrabBlock:
                holdTimer.reset();
                coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Down);
                coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.GrabbingBlock;
                break;

            case GrabbingBlock:
                if ((holdTimer.time() >= 1500)) {
                    state = RobotState.ReOrientate;
                }
                else {
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;
/*            case LiftBlock:
                holdTimer.reset();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.LiftingBlock;
                break;
            case LiftingBlock:
                if ((holdTimer.time() >= 1000)) {
                    state = RobotState.ReOrientate;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                }
                break;
*/
            case ReOrientate:
                if (EndWall == true){
                    this.moveRobot.StartMove(20, 40, 0, GO_BACK, 0);
                    state = RobotState.ReOrienting;
                }
                else {
                    this.moveRobot.StartMove(20, 16, 0, GO_BACK, 0);
                    state = RobotState.ReOrienting;
                }
                break;

            case ReOrienting:
                this.CheckIfDone(RobotState.SpinAfterReOrientate);
                break;

            case SpinAfterReOrientate:
                if (RedTeam == true) {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                    state = RobotState.SpinAfterReOrienting;
                }
                else {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 40);
                    state = RobotState.SpinAfterReOrienting;
                }

                break;

            case SpinAfterReOrienting:
                this.CheckIfDoneRotating(RobotState.MoveUnderSkyBridge1);
                break;

            case MoveUnderSkyBridge1:
                moveRobot.StartMove(35, DistanceUnderBridge, 0, GO_FORWARD, 0);
                state = RobotState.MovingUnderSkyBridge1;
                break;

            case MovingUnderSkyBridge1:
                this.CheckIfDone(RobotState.StraightenBeforeBackup);
                break;

            case StraightenBeforeBackup:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 20);
                state = RobotState.StraighteningBeforeBackup;
                break;
            case StraighteningBeforeBackup:
                this.CheckIfDoneRotating(RobotState.DropBlock);
                break;


            case DropBlock:
                holdTimer.reset();
                coreHexStateMachineBlockGrabber.ReleaseGrip();
                coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.PositionZeroGrabber);
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
                coreHexStateMachineBlockLifter.ProcessState();
                coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.DropingBlock;
                break;
            case DropingBlock:
                if ((holdTimer.time() >= 5000)) {
                    state = RobotState.ParkUnderBridge;
                }
                else {
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;

/*
            case DropBlock:
                holdTimer.reset();
                coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
                state = RobotState.DropingBlock;
                break;

            case DropingBlock:
                if ((holdTimer.time() >= 1000)) {
                    state = RobotState.StraightenBeforeBackup;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                    coreHexStateMachineBlockGrabber.ProcessState();
                }

                break;

            case ReleaseBlock:
                holdTimer.reset();
                coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.ReleasingBlock;
                break;

            case ReleasingBlock:
                if ((holdTimer.time() >= 1)) {
                    if (!this.SecondSkyStone) {
                        state = RobotState.ParkUnderBridge;
                    } else {
                        state = RobotState.StraightenBeforeBackup;
                    }
                }
                else {
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;
*/


            //For the 2nd competition we decide to just do one skystone so for now after it straightens up it will go to park under bridge
            case BackUp1:
                moveRobot.StartMove(45,110, 0, GO_BACK, 0);
                state = RobotState.BackingUp1;
                break;

            case BackingUp1:
                this.CheckIfDone(RobotState.MoveInfrontOfBlock);
                break;

            case MoveInfrontOfBlock:
                DistanceUnderBridge = DistanceUnderBridge + 30;
                InchesTowardsBlock = InchesTowardsBlock - 7;
                if (skyStonePosition == 2){
                    moveRobot.StartMove(40,14, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                else if (skyStonePosition == 1){
                    moveRobot.StartMove(40,5, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                else if (skyStonePosition == 0){
                    moveRobot.StartMove(40,1, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                break;

            case MovingInfrontOfBlock:
                this.CheckIfDone(RobotState.FaceFront2);
                break;

            case FaceFront2:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 180), 20);
                state = RobotState.FacingFront2;
                break;
            case FacingFront2:
                this.CheckIfDoneRotating(RobotState.MoveTowardsBlock1);
                break;
            case ParkUnderBridge:
                moveRobot.StartMove(20,25, 0, GO_BACK, 0);
                coreHexStateMachineBlockLifter.ProcessState();
                coreHexStateMachineBlockGrabber.ProcessState();
                state = RobotState.ParkingUnderBridge;
                break;
            case ParkingUnderBridge:
                this.CheckIfDone(RobotState.Done);
                break;
            //ThatOneBoi

        }

    }
}
