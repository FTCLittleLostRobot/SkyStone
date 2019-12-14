/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class StayOutOfAlliancesWayCollectInfo {

    Telemetry telemetry;
    StayOutOfAlliancesWayCollectInfo.RobotState state;
    HardwareMecanumBase robot;
    Gamepad gamepad1;
    public boolean MoveForwards;
    public boolean MoveBackwards;
    public boolean ParkNeutralBridge;
    public int WaitTimeAutonomous;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public String gamePlan = "";
    public boolean YButtonClicked = false;
    public boolean AButtonClicked = false;

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
        state = StayOutOfAlliancesWayCollectInfo.RobotState.Start;
        this.gamepad1 = gamepad1;
    }

    public void Start()
    {
        state = RobotState.MoveForwards;
        holdTimer.reset();

    }

    public boolean IsDone()
    {
        return (state == StayOutOfAlliancesWayCollectInfo.RobotState.Done);
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
                if ((holdTimer.time() >= 500)) {

                    if (X) {
                        MoveForwards = true;
                        gamePlan = gamePlan + "MovingForwards || ";
                        X = false;
                        holdTimer.reset();
                        state = RobotState.MoveBackwards;
                    } else if (B) {
                        MoveForwards = false;
                        B = false;
                        holdTimer.reset();
                        state = RobotState.MoveBackwards;
                    } else {
                        state = RobotState.MoveForwards;
                    }
                }
                break;

            case MoveBackwards:
                telemetry.addData("Move Backwards Under bridge", "X: yes || B: no ");
                telemetry.update();
                if ((holdTimer.time() >= 500)) {

                    if (X) {
                        MoveBackwards = true;
                        gamePlan = gamePlan + "MoveBackwards || ";
                        holdTimer.reset();
                        state = RobotState.ParkNeutralBridge;
                    } else if (B) {
                        MoveBackwards = false;
                        holdTimer.reset();
                        state = RobotState.ParkNeutralBridge;
                    } else {
                        state = RobotState.MoveBackwards;
                    }
                }
                break;



            case ParkNeutralBridge:
                telemetry.addData("End Of Autonomous Parking","A: wall || y: neutral bridge");
                if ((holdTimer.time() >= 500)) {
                    if (Y) {
                        ParkNeutralBridge = true;
                        gamePlan = gamePlan + "NeutralBridge || ";
                        Y = false;
                        state = RobotState.WaitTimeAutonomous;
                    } else if (A) {
                        A = false;
                        ParkNeutralBridge = false;
                        state = RobotState.WaitTimeAutonomous;
                    } else {
                        state = RobotState.ParkNeutralBridge;
                    }
                }
                break;

            case WaitTimeAutonomous:
                telemetry.addData("Wait Time", "Y: +2 || A: -2 || RightBumper: Done");
                if (!Y) {
                    YButtonClicked = false;
                }
                if (!A) {
                    AButtonClicked = false;
                }
                if ((holdTimer.time() >= 500)) {
                    if (Y && !YButtonClicked) {
                        WaitTimeAutonomous = WaitTimeAutonomous + 2;
                        YButtonClicked = true;
                        state = RobotState.WaitTimeAutonomous;

                    } else if (A && !AButtonClicked) {
                        WaitTimeAutonomous = WaitTimeAutonomous - 2;
                        AButtonClicked = true;
                        state = RobotState.WaitTimeAutonomous;

                    } else if (gamepad1.right_bumper) {
                        state = RobotState.Done;
                        gamePlan = gamePlan + WaitTimeAutonomous;

                    } else {
                        state = RobotState.WaitTimeAutonomous;
                    }
                }
                break;

            case Done:
                state = StayOutOfAlliancesWayCollectInfo.RobotState.Done;
                break;
        }
        telemetry.addLine();
        telemetry.addData("WaitTimeAutonomous", WaitTimeAutonomous);
        telemetry.addData("GamePlan", gamePlan);
        telemetry.update();

    }
}
