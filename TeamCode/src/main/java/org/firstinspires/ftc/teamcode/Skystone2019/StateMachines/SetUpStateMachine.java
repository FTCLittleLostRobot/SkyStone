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
    public boolean TwoSkystones = false;
    public boolean JustGoingUnderBridge;
    public String configData = "";

    enum RobotState
    {
        Start,
        RedOrBlueQ,
        EndingPositionFartherOrCloserQ,
        Skystones,
        Foundation,
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


            case EndingPositionFartherOrCloserQ:
                telemetry.addData("Where are we ending in Autonomous?","A: closer to the wall || Y: towards the neutral bridge");
                if (A){
                    EndWall = true;
                    configData = configData + "EndingPosition: Closer to Wall ";
                    state = RobotState.Skystones;
                }
                else if (Y){
                    EndNeutralBridge = true;
                    configData = configData + "EndingPosition: Closer to Neutral Bridge ";
                    state = RobotState.Skystones;
                }
                else {
                    state = RobotState.EndingPositionFartherOrCloserQ;
                }
                break;
            case Skystones:
                telemetry.addData("How many Skystones are we getting?", "X: 1 || B: 2");
                if (X){
                    TwoSkystones = false;
                    configData = configData + "Skystones: 1 ";
                    state = RobotState.Skystones;

                }
                else if (B){
                    TwoSkystones = true;
                    configData = configData + "Skystones: 2 ";
                    state = RobotState.Skystones;

                }
                break;

            case Done:
                state = SetUpStateMachine.RobotState.Done;
                break;
        }
        telemetry.addLine();
        telemetry.addData("GamePlan", configData);

    }
}
