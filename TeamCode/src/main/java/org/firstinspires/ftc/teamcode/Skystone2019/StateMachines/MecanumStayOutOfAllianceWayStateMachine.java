/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.Config.IConfiguration;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class MecanumStayOutOfAllianceWayStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    MecanumStayOutOfAllianceWayStateMachine.RobotState state;
    SetUpStateMachine setUpStateMachine;
    IConfiguration robotConfig;
    MecanumMotor motors;
    private GyroInitStateMachine gyroInitStateMachine;
    private MecanumRotateStateMachine rotateStateMachine;
    private StayOutOfAlliancesWayCollectInfo stayOutOfAlliancesWay;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    boolean MoveForwards;
    boolean MoveBackwards;
    boolean ParkNeutralBridge;
    int WaitTimeAutonomous;

    // THIS IS IF THE ROBOT IS FACING FORWARDS
    static final double GO_FORWARD = -1;
    static final double GO_BACK = 1;

    enum RobotState
    {
        Start,
        WaitTimer,
        MoveForwards,
        MovingForwards,
        MoveBackwards,
        MovingBackwards,
        MoveForwardsNeutralBridge,
        MovingForwardsNeutralBridge,
        StraffeUnderBridge,
        StraffingUnderBridge,
        Done
    }


    public void init(Telemetry telemetry, MecanumMotor motors, boolean moveForwards, boolean moveBackwards, boolean parkNeutralBridge, int waitTimeAutonomous, Gamepad gamepad1) {

        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);
        this.motors = motors;

        this.telemetry = telemetry;
        this.stayOutOfAlliancesWay = new StayOutOfAlliancesWayCollectInfo();
        this.stayOutOfAlliancesWay.init(telemetry, gamepad1);
        stayOutOfAlliancesWay.Start();

        this.MoveForwards = moveForwards;
        this.MoveBackwards = moveBackwards;
        this.ParkNeutralBridge = parkNeutralBridge;
        this.WaitTimeAutonomous = waitTimeAutonomous;
        state = MecanumStayOutOfAllianceWayStateMachine.RobotState.Start;
    }

    public void Start()
    {
        holdTimer.reset();
        state = RobotState.WaitTimer;
    }

    private void CheckIfDone(RobotState nextState) {
        if (this.moveRobot.IsDone()) {
            this.moveRobot.Complete();
            state = nextState;
        }
    }
    public boolean IsDone()
    {
        return (state == MecanumStayOutOfAllianceWayStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        telemetry.addData("Current State", state.toString());

        telemetry.addData("targetLeftFrontEncoderValue", moveRobot.targetLeftFrontEncoderValue);
        telemetry.addData("CurrentLeftFrontPosition", moveRobot.GetLeftMotorEncodePosition());
        telemetry.update();


        switch (state) {
            case WaitTimer:
                if (holdTimer.time() >= WaitTimeAutonomous) {
                    if (MoveForwards == true) {
                        state = RobotState.MoveForwards;
                    } else if (MoveBackwards == true) {
                        state = RobotState.MoveBackwards;
                    } else if (ParkNeutralBridge == true) {
                        state = RobotState.MoveForwardsNeutralBridge;
                    }
                }

                break;
            case MoveForwards:
                this.moveRobot.StartMove(30, 20, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingForwards;
                break;
            case MovingForwards:
                this.CheckIfDone(RobotState.Done);
                break;

            case MoveBackwards:
                this.moveRobot.StartMove(30, 20, 0, MecanumEncoderMove.GO_BACK, 0);
                state = RobotState.MovingBackwards;
                break;
            case MovingBackwards:
                this.CheckIfDone(RobotState.Done);
                break;

            case MoveForwardsNeutralBridge:
                this.moveRobot.StartMove(30, 38, 0, MecanumEncoderMove.GO_FORWARD, 0);
                state = RobotState.MovingForwardsNeutralBridge;
                break;
            case MovingForwardsNeutralBridge:
                this.CheckIfDone(RobotState.StraffeUnderBridge);
                break;
            case StraffeUnderBridge:
                this.moveRobot.StartMove(30, 23, 1, 0, 0);
                state = RobotState.StraffingUnderBridge;
                break;
            case StraffingUnderBridge:
                this.CheckIfDone(RobotState.Done);
        }

    }
}
