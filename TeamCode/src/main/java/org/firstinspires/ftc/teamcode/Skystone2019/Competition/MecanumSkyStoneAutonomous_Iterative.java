/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumSkyStoneStateMachine;

@Autonomous(name="Mecanum: Skystone", group="Mecanum")
public class MecanumSkyStoneAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumSkyStoneStateMachine MecanumSkyStoneStateMachine;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private ColorFinder colorFinder;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.colorFinder = new ColorFinder();
        this.MecanumSkyStoneStateMachine = new MecanumSkyStoneStateMachine();

        /* Step 2: Setup of hardware  */
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);

        /* Step 3: Setup of controllers  */
        this.moveRobot.init(this.mecanumRobot);
        this.colorFinder.init(hardwareMap);

        /* Step 4: Setup of state machines  */
        this.MecanumSkyStoneStateMachine.init(telemetry, mecanumRobot, colorFinder);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void start() {
        MecanumSkyStoneStateMachine.Start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        //this goes into the StateMachine, "SamplingStateMachine" and then goes through all the states it needs
        MecanumSkyStoneStateMachine.ProcessState();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}