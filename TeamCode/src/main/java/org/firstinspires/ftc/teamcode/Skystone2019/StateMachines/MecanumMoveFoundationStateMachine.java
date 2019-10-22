/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.ConfigFactory;

public class MecanumMoveFoundationStateMachine {

    Telemetry telemetry;
    MecanumMove moveRobot;
    MecanumMoveFoundationStateMachine.RobotState state;
    IConfiguration robotConfig;

    boolean isCloseSquare;
    boolean isRed;
    enum RobotState
    {
        Start,
        MoveStraight,
        MovingStraight,
        PullBack,
        PullingBack,
        StrafeLeftRed,
        StrafingLeftRed,
        StrafeRightBlue,
        StrafingRightBlue,
        MoveFowards,
        MovingFowards,
        PushPlatformRed,
        PushingPlatformRed,
        PushPlatformBlue,
        PushingPlatformBlue,
        TowardsRedBridge,
        MovingTowardsRedBridge,
        UnderRedBridge,
        MovingUnderRedBridge,
        TowardBlueBridge,
        MovingTowardBlueBridge,
        UnderBlueBridge,
        MovingUnderBlueBridge,
        Done
    }

    public void init(Telemetry telemetry, HardwareMecanumBase robot, boolean isCloseSquare, boolean isRed) {

        this.telemetry = telemetry;
        this.moveRobot = new MecanumMove();
        this.moveRobot.init(robot);
        this.isCloseSquare = isCloseSquare;
        this.isRed = isRed;
        this.moveRobot.init(robot);

        // Setup the configuration object
        robotConfig = new ConfigFactory().Get();
        state = MecanumMoveFoundationStateMachine.RobotState.Start;
    }

    public void Start()
    {
        state = RobotState.MoveStraight;
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
            case MoveStraight:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesFromWall, 0, -1, 0);
                state = RobotState.MovingStraight;
                break;

            case MovingStraight:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.PullBack;
                }
                break;

            case PullBack:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedTowardsWall, 0, 1, 0);
                state = RobotState.PullingBack;
                break;

            case PullingBack:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();

                    if (isRed) {
                        state = RobotState.StrafeLeftRed;
                    }
                    else {
                        state = RobotState.StrafeRightBlue;
                    }
                }
                break;

            case StrafeLeftRed:
                //x = -1 makes it go right
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesSidewaysMoved, 1, 0, 0);
                state = RobotState.StrafingLeftRed;
                break;

            case StrafingLeftRed:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.MoveFowards;
                }
                break;

            case StrafeRightBlue:
                //x = -1 makes it go right
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesSidewaysMoved, -1, 0, 0);
                state = RobotState.StrafingRightBlue;
                break;

            case StrafingRightBlue:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.MoveFowards;
                }
                break;


            case MoveFowards:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesMoveNextToPlatForm, 0, -1, 0);
                state = RobotState.MovingFowards;
                break;

            case MovingFowards:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();

                    if (isRed) {
                        state = RobotState.PushPlatformRed;
                    }
                    else {
                        state = RobotState.PushPlatformBlue;
                    }
                }
                break;

            case PushPlatformRed:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesPlatFormPushed, -1, 0, 0);
                state = RobotState.PushingPlatformRed;
                break;

            case PushingPlatformRed:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.TowardsRedBridge;
                }
                break;

            case PushPlatformBlue:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesPlatFormPushed, 1, 0, 0);
                state = RobotState.PushingPlatformBlue;
                break;

            case PushingPlatformBlue:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();

                        state = RobotState.TowardBlueBridge;
                }
                break;

            case TowardsRedBridge:
                //y = 1 makes it go backwards
                if (isCloseSquare) {
                    this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedTowardsBridgeClose, 1, 1, 0);
                }
                else {
                    this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedTowardsBridgeFar, 1, -1, 0);
                }
                state = RobotState.MovingTowardsRedBridge;
                break;

            case MovingTowardsRedBridge:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.UnderRedBridge
                    ;
                }
                break;

            case UnderRedBridge:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedUnderBridge, 1, 0, 0);
                state = RobotState.MovingUnderRedBridge;
                break;

            case MovingUnderRedBridge:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done
                    ;
                }
                break;

            case TowardBlueBridge:
                //y = 1 makes it go backwards
                if (isCloseSquare) {
                    this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedTowardsBridgeClose, -1, 1, 0);
                }
                else {
                    this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedTowardsBridgeFar, -1, -1, 0);
                }
                state = RobotState.MovingTowardBlueBridge;
                break;

            case MovingTowardBlueBridge:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.UnderBlueBridge
                    ;
                }
                break;

            case UnderBlueBridge:
                //y = 1 makes it go backwards
                this.moveRobot.StartMove(50, robotConfig.FoundationInchesMovedUnderBridge, -1, 0, 0);
                state = RobotState.MovingUnderBlueBridge;
                break;

            case MovingUnderBlueBridge:
                if (this.moveRobot.IsDone()) {
                    this.moveRobot.Complete();
                    state = RobotState.Done
                    ;
                }
                break;

            case Done:
                state = MecanumMoveFoundationStateMachine.RobotState.Done;
                break;
        }
    }
}
