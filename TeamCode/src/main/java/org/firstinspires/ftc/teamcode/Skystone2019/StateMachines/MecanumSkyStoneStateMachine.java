/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import android.graphics.Bitmap;

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
    private GyroController gyro = new GyroController();
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;

    // THIS IS IF THE ROBOT IS FACING FORWARDS
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_SPEED = 0.25;
    static final double GO_FORWARD = -1;
    static final double GO_BACK = 1;
    static final double GO_RIGHT = -1;
    static final double GO_LEFT = 1;
    private int DistanceUnderBridge = 60;
    private Image vuforiaImageObject;
    private Bitmap bitmapFromVuforia;
    int foundColumn = -1;
    public int skyStonePosition = -1;
    private boolean SecondSkyStone = false;
    private int InchesTowardsBlock = 17;
    private int origSpeed = 50;
    private int robotHeading = 0;
    private boolean RedTeam;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

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
        Block1Position180Turn,
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

    public void init(Telemetry telemetry, MecanumMotor motors, ColorFinder colorFinder, boolean isCloseSquare, boolean isRed, HardwareMecanumBase robot ) {

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
        this.mecanumRotateStateMachine.init(telemetry, motors,gyro);
        this.RedTeam = isRed;
        this.gyro = gyro;

        state = MecanumSkyStoneStateMachine.RobotState.Start;
    }

    public void Start()
    {
        this.origSpeed = this.motors.GetSpeedMultiplier();
        state = RobotState.MoveForwards;
        this.robotHeading = this.gyro.GetCurrentRobotHeading();
    }

    private void CheckIfDone(RobotState nextState) {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
            this.motors.SetSpeedToValue(origSpeed);
            state = nextState;
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
                this.moveRobot.StartMove(20, 27, 0, MecanumEncoderMove.GO_BACK, 0);
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
                    this.moveRobot.StartRotate(telemetry, 20, 90.0, MecanumEncoderMove.RotationDirection.Right );
                    state = RobotState.Block2Position90Turn;
                    skyStonePosition = 2;
                    this.DistanceUnderBridge = DistanceUnderBridge + 8;
                }
                else if (foundColumn == 1 )
                {
                    this.moveRobot.StartRotate(telemetry, 20, 180.0, MecanumEncoderMove.RotationDirection.Right );
                    state = RobotState.Block1Position180Turn;
                    skyStonePosition = 1;

                }
                else if (foundColumn == 0)
                {
                    this.moveRobot.StartRotate(telemetry, 20, 90.0, MecanumEncoderMove.RotationDirection.Left );
                    state = RobotState.Block0Position90Turn;
                    skyStonePosition = 0;
                    this.DistanceUnderBridge = DistanceUnderBridge - 8;

                }
                break;

            case MovingTowardsBlock:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.GrabBlock;
                }
                break;
            case Block2Position90Turn:
                this.CheckIfDone(RobotState.Block2PositionGoRight);
                break;
            case Block2PositionGoRight:
                this.moveRobot.StartMove(20, 12, 0 , GO_BACK,0 );
                state = RobotState.Block2PositionGoingRight;
                break;
            case Block2PositionGoingRight:
                this.CheckIfDone(RobotState.Block2Position90FaceFront);
                break;

            case Block2Position90FaceFront:
                this.moveRobot.StartRotate(telemetry, 20, 90.0, MecanumEncoderMove.RotationDirection.Right );
                state = RobotState.Block2Position90FaceingFront;
                break;

            case Block2Position90FaceingFront:
                this.CheckIfDone(RobotState.MoveTowardsBlock1);
                break;



            case Block1Position180Turn:
                this.CheckIfDone(RobotState.MoveTowardsBlock1);
                break;



            case Block0Position90Turn:
                this.CheckIfDone(RobotState.Block0PositionGoLeft);
                break;
            case Block0PositionGoLeft:
                this.moveRobot.StartMove(20, 12, 0 , GO_BACK,0 );
                state = RobotState.Block0PositionGoingLeft;
                break;
            case Block0PositionGoingLeft:
                this.CheckIfDone(RobotState.Block0Position90FaceFront);
                break;

            case Block0Position90FaceFront:
                this.moveRobot.StartRotate(telemetry, 20, 90.0, MecanumEncoderMove.RotationDirection.Left );
                state = RobotState.Block0Position90FaceingFront;
                break;

            case Block0Position90FaceingFront:
                this.CheckIfDone(RobotState.MoveTowardsBlock1);
                break;

            case MoveTowardsBlock1:
                this.moveRobot.StartMove(30, InchesTowardsBlock, 0 , GO_FORWARD,0 );
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
                if ((holdTimer.time() >= 1)) {
                    state = RobotState.LiftBlock;
                }
                else {
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;
            case LiftBlock:
                holdTimer.reset();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.LiftingBlock;
                break;
            case LiftingBlock:
                if ((holdTimer.time() >= 1)) {
                    state = RobotState.ReOrientate;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                }
                break;

            case ReOrientate:
                this.moveRobot.StartMove(20, 5, 0, GO_BACK, 0);
                state = RobotState.ReOrienting;
                break;

            case ReOrienting:
                this.CheckIfDone(RobotState.SpinAfterReOrientate);
                break;

            case SpinAfterReOrientate:
                if (RedTeam == true) {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 20);
                    // this.moveRobot.StartRotate(telemetry, 50, 90.sac0, MecanumEncoderMove.RotationDirection.Right);
                    state = RobotState.SpinAfterReOrienting;
                }
                else {
                    this.moveRobot.StartRotate(telemetry, 50, 90.0, MecanumEncoderMove.RotationDirection.Left);
                    state = RobotState.SpinAfterReOrienting;
                }
                break;

            case SpinAfterReOrienting:
                if (mecanumRotateStateMachine.IsDone()) {
                    state = RobotState.LiftBlock;
                }
                else {
                    mecanumRotateStateMachine.ProcessState();
                }
//todo ADD Gyro for Blue team otherwise SpinAfterReOrientating wont work
                break;

            case MoveUnderSkyBridge1:
                moveRobot.StartMove(45, DistanceUnderBridge, 0, GO_FORWARD, 0);
                state = RobotState.MovingUnderSkyBridge1;

                break;

            case MovingUnderSkyBridge1:
                this.CheckIfDone(RobotState.DropBlock);
                break;

            case DropBlock:
                holdTimer.reset();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.DropingBlock;
                break;

            case DropingBlock:
                if ((holdTimer.time() >= 1)) {
                    state = RobotState.ReleaseBlock;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
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
                    if (this.SecondSkyStone == true) {
                        state = RobotState.ParkUnderBridge;
                    } else {
                        state = RobotState.BackUp1;
                    }
                }
                else {
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;

            case BackUp1:
                this.motors.SetSpeedToValue(10);
                moveRobot.StartMove(5,100, 0, GO_BACK, 0);
                state = RobotState.BackingUp1;
                break;

            case BackingUp1:
                this.CheckIfDone(RobotState.MoveInfrontOfBlock);
                break;

            case MoveInfrontOfBlock:
                this.SecondSkyStone = true;
                DistanceUnderBridge = DistanceUnderBridge + 30;
                InchesTowardsBlock = InchesTowardsBlock - 7;
                if (skyStonePosition == 2){
                    moveRobot.StartMove(20,14, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                else if (skyStonePosition == 1){
                    moveRobot.StartMove(20,5, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                else if (skyStonePosition == 0){
                    moveRobot.StartMove(20,1, 0, GO_FORWARD, 0);
                    state = RobotState.MovingInfrontOfBlock;
                }
                break;

            case MovingInfrontOfBlock:
                this.CheckIfDone(RobotState.FaceFront2);
                break;

            case FaceFront2:
                if (RedTeam == true) {
                    this.moveRobot.StartRotate(telemetry, 30, 90.0, MecanumEncoderMove.RotationDirection.Left);
                    state = RobotState.FacingFront2;
                }
                else {
                    this.moveRobot.StartRotate(telemetry, 30, 90.0, MecanumEncoderMove.RotationDirection.Right);
                    state = RobotState.FacingFront2;
                }
                break;
            case FacingFront2:
                this.CheckIfDone(RobotState.MoveTowardsBlock1);
                break;
            case ParkUnderBridge:
                moveRobot.StartMove(20,20, 0, GO_BACK, 0);
                state = RobotState.ParkingUnderBridge;
                break;
            case ParkingUnderBridge:
                this.CheckIfDone(RobotState.Done);
                break;

            case Done:
                state = MecanumSkyStoneStateMachine.RobotState.Done;
                break;

        }

    }
}
