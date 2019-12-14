/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.StayOutOfAlliancesWayCollectInfo;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumStayOutOfAllianceWayStateMachine;


@Autonomous(name="Mecanum: Autonomous AllianceMoving", group="Mecanum")
public class MecanumBridgeAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private StayOutOfAlliancesWayCollectInfo stayOutOfAlliancesWay;
    private MecanumStayOutOfAllianceWayStateMachine mecanumStayOutOfAllianceWayStateMachine;


    boolean MoveForwards;
    boolean MoveBackwards;
    boolean ParkNeutralBridge;
    int WaitTimeAutonomous;


    @Override
    public void init() {
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);
        this.moveRobot.init(this.mecanumRobot);

        this.stayOutOfAlliancesWay = new StayOutOfAlliancesWayCollectInfo();
        this.stayOutOfAlliancesWay.init(telemetry, gamepad1);
        stayOutOfAlliancesWay.Start();

        this.mecanumStayOutOfAllianceWayStateMachine = new MecanumStayOutOfAllianceWayStateMachine();
    }

    @Override
    public void init_loop()
    {
        stayOutOfAlliancesWay.ProcessState();
    }

    @Override
    public void start() {
        MoveForwards = stayOutOfAlliancesWay.MoveForwards;
        MoveBackwards = stayOutOfAlliancesWay.MoveBackwards;
        ParkNeutralBridge = stayOutOfAlliancesWay.ParkNeutralBridge;
        WaitTimeAutonomous = stayOutOfAlliancesWay.WaitTimeAutonomous;


        this.mecanumStayOutOfAllianceWayStateMachine.init(telemetry, mecanumRobot, MoveForwards, MoveBackwards, ParkNeutralBridge, WaitTimeAutonomous, gamepad1);
        this.mecanumStayOutOfAllianceWayStateMachine.Start();

    }

    @Override
    public void loop()
    {
        mecanumStayOutOfAllianceWayStateMachine.ProcessState();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}