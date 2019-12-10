/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class StayOutOfAlliancesWay {

    Telemetry telemetry;
    StayOutOfAlliancesWay.RobotState state;
    HardwareMecanumBase robot;
    Gamepad gamepad1;
    public boolean MoveForwards;
    public boolean MoveBackwards;
    public boolean ParkNeutralBridge;
    public int WaitTimeAutonomous;

    public String gamePlan = "";

    enum RobotState
    {
        Start,
        MoveForwards,
        MoveBackwards,
        ParkNeutralBridge,
        WaitTimeAutonomous,
        Done
    }

    public void init(Telemetry telemetry, Gamepad gamepad1) {

        this.telemetry = telemetry;
        state = StayOutOfAlliancesWay.RobotState.Start;
        this.gamepad1 = gamepad1;
    }

    public void Start()
    {
        state = RobotState.MoveForwards;
    }

    public boolean IsDone()
    {
        return (state == StayOutOfAlliancesWay.RobotState.Done);
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

            case MoveForwards:
                telemetry.addData("Move Forwards to get under bridge?", "(X: Yes || B: No)");
                if (X){
                    MoveForwards = true;
                    gamePlan = gamePlan + "MovingForwards || ";
                    X = false;
                    state = RobotState.MoveBackwards;
                }
                else if (B){
                    MoveForwards = false;
                    B = false;
                    state = RobotState.MoveBackwards;
                }
                else {
                    state = RobotState.MoveForwards;
                }

                break;

            case MoveBackwards:
                telemetry.addData("Move Backwards to get under bridge?", "X: Yes || B: No ");
                telemetry.update();
                if (X){
                    MoveBackwards = true;
                    gamePlan = gamePlan + "MoveBackwards || ";
                    state = RobotState.ParkNeutralBridge;
                }
                else if (B){
                    MoveBackwards = false;
                    state = RobotState.ParkNeutralBridge;
                }
                else {
                    state = RobotState.MoveBackwards;
                }
                break;



            case ParkNeutralBridge:
                telemetry.addData("Park close to the NeutralBridge?","A: closer to the wall || y: towards the neutral bridge");
                if (Y){
                    ParkNeutralBridge = true;
                    gamePlan = gamePlan + "NeutralBridge || ";
                    Y = false;
                    state = RobotState.WaitTimeAutonomous;
                }
                else if (A){
                    A = false;
                    ParkNeutralBridge = false;
                    state = RobotState.WaitTimeAutonomous;
                }
                else {
                    state = RobotState.ParkNeutralBridge;
                }
                break;

            case WaitTimeAutonomous:
                telemetry.addData("How much time should we wait?", "Y: +2 || A: -2 || rightBumper: Done");
                if (Y){
                    WaitTimeAutonomous = WaitTimeAutonomous + 2;
                    gamePlan = gamePlan + WaitTimeAutonomous;
                }
                else if (A){
                    WaitTimeAutonomous = WaitTimeAutonomous - 2;
                    gamePlan = gamePlan + WaitTimeAutonomous;
                }
                else if (gamepad1.right_bumper){
                    state = RobotState.Done;

                }
                else {
                    state = RobotState.WaitTimeAutonomous;
                }
                break;

            case Done:
                state = StayOutOfAlliancesWay.RobotState.Done;
                break;
        }
        telemetry.addData("GamePlan", gamePlan);

    }
}
