/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.ColorFinder;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.GyroInitStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumSkyStoneStateMachine;

@Autonomous(name="Mecanum: Skystone", group="Mecanum")
public class MecanumSkyStoneAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumSkyStoneStateMachine MecanumSkyStoneStateMachine;
    private GyroInitStateMachine gyroInitStateMachine ;
    private MecanumEncoderMove moveRobot;
    private MecanumMotor mecanumRobot;
    private GyroController gyro;
    private ColorFinder colorFinder;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.mecanumRobot = new MecanumMotor();
        this.moveRobot = new MecanumEncoderMove();
        this.colorFinder = new ColorFinder();
        this.gyro = new GyroController();
        this.MecanumSkyStoneStateMachine = new MecanumSkyStoneStateMachine();
        this.gyroInitStateMachine = new GyroInitStateMachine();

        /* Step 2: Setup of hardware  */
        this.robot.init(hardwareMap);
        this.mecanumRobot.init(robot);

        /* Step 3: Setup of controllers  */
        this.moveRobot.init(this.mecanumRobot);
        this.colorFinder.init(hardwareMap);
        this.gyro.init(robot);

        /* Step 4: Setup of state machines  */
        this.gyroInitStateMachine.init(telemetry, this.gyro);
        this.MecanumSkyStoneStateMachine.init(telemetry, mecanumRobot, colorFinder, false, false, robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void init_loop() {
        this.gyroInitStateMachine.ProcessState();
        telemetry.addData("Gyro Init State", gyroInitStateMachine.IsDone());    //
        telemetry.update();
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