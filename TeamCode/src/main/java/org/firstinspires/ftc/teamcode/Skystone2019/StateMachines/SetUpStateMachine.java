/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SetUpStateMachine {

    Telemetry telemetry;
    SetUpStateMachine.RobotState state;
    HardwareMecanumBase robot;
    Gamepad gamepad1;
    public boolean BlueTeam = false;
    public boolean RedTeam = false;
    public boolean StartDepot = false;
    public boolean StartFoundation = false;
    public boolean EndNeutralBridge = false;
    public boolean EndWall = false;
    public boolean JustGoingUnderBridge;
    public String configData = "";

    enum RobotState
    {
        Start,
        RedOrBlueQ,
        FoundationOrDepotQ,
        EndingPositionFartherOrCloserQ,
        JustGoingUnderBridge,
        Done
    }

    public void init(Telemetry telemetry, Gamepad gamepad1) {

        this.telemetry = telemetry;
        state = SetUpStateMachine.RobotState.Start;
        this.gamepad1 = gamepad1;
    }

    public void Start()
    {
        state = RobotState.RedOrBlueQ;
    }

    public boolean IsDone()
    {
        return (state == SetUpStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {
        boolean X;
        boolean Y;
        boolean B;
        boolean A;

        X = gamepad1.x;
        Y = gamepad1.y;
        B = gamepad1.b;
        A = gamepad1.a;
        // telemetry.addData("targetLeftFrontEncoderValue", this.moveRobot.targetLeftFrontEncoderValue);

        switch (state)
        {

            case RedOrBlueQ:
                telemetry.addData("What Alliance are we on?", "(X: Blue || B: red)");
                if (X){
                    BlueTeam = true;
                    configData = configData + "Alliance: BlueTeam || ";
                    state = RobotState.EndingPositionFartherOrCloserQ;
                }
                else if (B){
                    RedTeam = true;
                    configData = configData + "Alliance: RedTeam || ";
                    state = RobotState.EndingPositionFartherOrCloserQ;

                }
                else {
                    state = RobotState.RedOrBlueQ;
                }

                break;

          /*  case FoundationOrDepotQ:
                telemetry.addData("What side of the field are we on?", "A: Depot || Y: Foundation");

                if (A){
                    StartDepot = true;
                    configData = configData + "StartingPosition: Depot || ";
                    state = RobotState.EndingPositionFartherOrCloserQ;
                }
                else if (Y){
                    StartFoundation = true;
                    configData = configData + "StartingPosition: Foundation || ";
                    state = RobotState.EndingPositionFartherOrCloserQ;
                }
                else {
                    state = RobotState.FoundationOrDepotQ;
                }
                break;

           */

            case EndingPositionFartherOrCloserQ:
                telemetry.addData("Where are we ending in Autonomous?","B: closer to the wall || X: towards the neutral bridge");
                if (B){
                    EndWall = true;
                    configData = configData + "EndingPosition: Closer to Wall ";
                    state = RobotState.JustGoingUnderBridge;
                }
                else if (X){
                    EndNeutralBridge = true;
                    configData = configData + "EndingPosition: Closer to Neutral Bridge ";
                    state = RobotState.JustGoingUnderBridge;
                }
                else {
                    state = RobotState.EndingPositionFartherOrCloserQ;
                }
                break;
            case JustGoingUnderBridge:
                telemetry.addData("Are we just going under the bridge?", "Y: yes || A: no");
                if (Y){
                    JustGoingUnderBridge = true;
                    configData = configData + "JustGoingUnderBridge";
                    state = RobotState.Done;
                }
                else if (A){
                    JustGoingUnderBridge = false;
                    configData = configData + "Doing Autonomous program";
                    state = RobotState.Done;
                }
                else {
                    state = RobotState.JustGoingUnderBridge;
                }
                break;

            case Done:
                state = SetUpStateMachine.RobotState.Done;
                break;
        }
        telemetry.addData("GamePlan", configData);

    }
}
