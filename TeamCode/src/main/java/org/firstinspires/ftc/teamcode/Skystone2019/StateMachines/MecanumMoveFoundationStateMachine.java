/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import android.graphics.Bitmap;
import android.graphics.BitmapRegionDecoder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.ConfigFactory;

public class MecanumMoveFoundationStateMachine {
    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    MecanumMoveFoundationStateMachine.RobotState state;
    SetUpStateMachine setUpStateMachine;
    IConfiguration robotConfig;
    MecanumMotor motors;

    private CoreHexStateMachine coreHexStateMachineBlockGrabber;
    private CoreHexStateMachine coreHexStateMachineBlockLifter;
    private MecanumRotateStateMachine mecanumRotateStateMachine;
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;
    private HardwareMecanumBase robot;
    private int origSpeed = 50;
    private int robotHeading = 0;
    private boolean RedTeam;
    private boolean EndWall;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum RobotState
    {
        Start,
        MoveGrabbersMidLevel,
        MovingGrabberMidLevel,
        MoveStraight,
        MovingStraight,
        GrabFoundation,
        GrabbingFoundation,
        PullBack,
        PullingBack,
        ReleaseFoundation,
        ReleasingFoundation,
        TurnTowardsBridge,  // if Blue then turn left... if red then turn right
        TurningTowardsBridge,
        MoveTowardsBridge1,
        MovingTowardsBridge1,
        TurnTowardsOtherAlliance1,
        TurningTowardsOtherAlliance1,
        MoveParallelToFoundation, //Go Straight so can push foundation over next
        MovingParallelToFoundation,
        TurnTowardsFoundation, // if red turn different way then blue
        TurningTowardsFoundation,
        PushFoundation,
        PushingFoundation,
        BackUpALittle, //back up a bit so dont hit foundation
        BackingUpALittle,
        TurnTowardsOtherAlliance2,
        TurningTowardsOtherAlliance2, //depending on alliance //also add if statement for whether to go Alliance bridge to park or Wall

        MoveForwardsToAllianceBridge,
        MovingForwardsToAllianceBridge,
        //or
        MoveBackToWall,
        MovingBackToWall,

        FaceDepotSide,
        FacingDepotSide,
        ParkUnderBridge,
        ParkingUnderBridge,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motor, boolean EndByWall, boolean RedTeam, HardwareMecanumBase robot) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motor);
        this.RedTeam = RedTeam;
        this.EndWall = EndByWall;
        // Setup the configuration object
        robotConfig = new ConfigFactory().Get();
        this.coreHexStateMachineBlockGrabber = new CoreHexStateMachine();
        this.coreHexStateMachineBlockGrabber.init(telemetry, robot, CoreHex.CoreHexMotors.BlockGrabber);
        this.coreHexStateMachineBlockLifter = new CoreHexStateMachine();
        this.coreHexStateMachineBlockLifter.init(telemetry, robot, CoreHex.CoreHexMotors.BlockLifter);

        state = MecanumMoveFoundationStateMachine.RobotState.Start;
    }

    public void Start()
    {
        this.origSpeed = this.motors.GetSpeedMultiplier();
        state = RobotState.MoveGrabbersMidLevel;
    }

    private void CheckIfDone(MecanumMoveFoundationStateMachine.RobotState nextState) {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
            this.motors.SetSpeedToValue(origSpeed);
            state = nextState;
        }
    }
    private void CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState nextState) {
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
        return (state == MecanumMoveFoundationStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        switch (state)
        {
            case MoveGrabbersMidLevel:
                holdTimer.reset();
                coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockGrabber.ProcessState();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.MovingGrabberMidLevel;
                break;
            case MovingGrabberMidLevel:
                if ((holdTimer.time() >= 1500)) {
                    state = MecanumMoveFoundationStateMachine.RobotState.MoveStraight;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                    coreHexStateMachineBlockGrabber.ProcessState();
                }
                break;

            case MoveStraight:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(20, 40, 0, -1, 0);
                state = RobotState.MovingStraight;
                break;

            case MovingStraight:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.GrabFoundation);
                break;
            case GrabFoundation:
                holdTimer.reset();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.GrabbingFoundation;
                break;
            case GrabbingFoundation:
                if ((holdTimer.time() >= 1500)) {
                    state = MecanumMoveFoundationStateMachine.RobotState.PullBack;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                }
                break;
            case PullBack:
                this.moveRobot.StartMove(30, 27, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.PullingBack;
                break;
            case PullingBack:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.ReleaseFoundation);
                break;
            case ReleaseFoundation:
                holdTimer.reset();
                coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Up);
                coreHexStateMachineBlockLifter.ProcessState();
                state = RobotState.ReleasingFoundation;
                break;
            case ReleasingFoundation:
                if ((holdTimer.time() >= 1500)) {
                    state = RobotState.TurnTowardsBridge;
                }
                else {
                    coreHexStateMachineBlockLifter.ProcessState();
                }
                break;
            case TurnTowardsBridge:
                if (RedTeam == true){
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 40); //+90 goes right -90 goes left
                    state = RobotState.TurningTowardsBridge;

                }
                else{
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                    state = RobotState.TurningTowardsBridge;
                }
                break;
            case TurningTowardsBridge:
                this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.MoveTowardsBridge1);
                break;
            case MoveTowardsBridge1:
                this.moveRobot.StartMove(30, 10, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingTowardsBridge1;
                break;
            case MovingTowardsBridge1:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.TurnTowardsOtherAlliance1);
                break;
            case TurnTowardsOtherAlliance1:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                state = RobotState.TurningTowardsOtherAlliance1;
                break;
            case TurningTowardsOtherAlliance1:
                this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.MoveParallelToFoundation);
                break;
            case MoveParallelToFoundation:
                this.moveRobot.StartMove(30, 10, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingParallelToFoundation;
                break;
            case MovingParallelToFoundation:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.TurnTowardsFoundation);
                break;
            case TurnTowardsFoundation:
                if (RedTeam == true){
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                    state = RobotState.TurningTowardsFoundation;
                }
                else {
                    mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 40);
                    state = RobotState.TurningTowardsFoundation;
                }
                break;
            case TurningTowardsFoundation:
                this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.PushFoundation);
                break;
            case PushFoundation:
                this.moveRobot.StartMove(30, 10, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.PushingFoundation;
                break;
            case PushingFoundation:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.BackUpALittle);
                break;
            case BackUpALittle:
                this.moveRobot.StartMove(30, 3, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.BackingUpALittle;
                break;
            case BackingUpALittle:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.TurnTowardsOtherAlliance2);
                break;
            case TurnTowardsOtherAlliance2:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading - 90), 40);
                state = RobotState.TurningTowardsOtherAlliance2;
                break;
            case TurningTowardsOtherAlliance2:

                if (EndWall == true){
                    this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.MoveBackToWall);
                }
                else {
                    this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.MoveForwardsToAllianceBridge);
                }
                break;
            case MoveForwardsToAllianceBridge:
                this.moveRobot.StartMove(30, 10, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingForwardsToAllianceBridge;
                break;
            case MovingForwardsToAllianceBridge:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.FaceDepotSide);
                break;

            case MoveBackToWall:
                this.moveRobot.StartMove(30, 15, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.MovingBackToWall;
                break;
            case MovingBackToWall:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.FaceDepotSide);
                break;

            case FaceDepotSide:
                mecanumRotateStateMachine.StartWithGyro((double)(this.robotHeading + 90), 40);
                state = RobotState.FacingDepotSide;
                break;
            case FacingDepotSide:
                this.CheckIfDoneRotating(MecanumMoveFoundationStateMachine.RobotState.ParkUnderBridge);
                break;
            case ParkUnderBridge:
                this.moveRobot.StartMove(30, 15, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.ParkingUnderBridge;
                break;
            case ParkingUnderBridge:
                this.CheckIfDone(MecanumMoveFoundationStateMachine.RobotState.Done);
                break;

            case Done:
                state = MecanumMoveFoundationStateMachine.RobotState.Done;
                break;
        }
    }
}
