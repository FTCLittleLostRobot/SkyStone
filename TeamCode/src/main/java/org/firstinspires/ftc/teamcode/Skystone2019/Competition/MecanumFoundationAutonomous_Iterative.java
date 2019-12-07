/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.GyroController;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.GyroInitStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveFoundationStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.SetUpStateMachine;

@Autonomous(name="Mecanum:Foundation", group="Mecanum")
public class MecanumFoundationAutonomous_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private MecanumMoveFoundationStateMachine moveFoundationStateMachine;
    private SetUpStateMachine setUpStateMachine;
    private GyroInitStateMachine gyroInitStateMachine ;
    private GyroController gyro;
    private boolean RedTeam;
    private boolean EndByWall;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.moveFoundationStateMachine = new MecanumMoveFoundationStateMachine();
        this.setUpStateMachine = new SetUpStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */
        this.gyro = new GyroController();
        this.gyroInitStateMachine = new GyroInitStateMachine();
        this.gyro.init(robot);

        /* Step 4: Setup of state machines  */
        this.setUpStateMachine.init(telemetry, gamepad1);
        setUpStateMachine.Start();
        this.gyroInitStateMachine.init(telemetry, this.gyro);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Press Blue Or Red Button To Select Team");    //
    }


    @Override
    public void init_loop() {
        this.gyroInitStateMachine.ProcessState();
        setUpStateMachine.ProcessState();
        telemetry.addData("Gyro Init State", gyroInitStateMachine.IsDone());
        telemetry.update();

    }

    @Override
    public void start() {
        RedTeam = setUpStateMachine.RedTeam;
        EndByWall = setUpStateMachine.EndWall;

        this.moveFoundationStateMachine.init(telemetry, motors, EndByWall, RedTeam, robot, gyro);
        moveFoundationStateMachine.Start();
    }

    @Override
    public void loop()
    {
        moveFoundationStateMachine.ProcessState();
    }

    @Override
    public void stop() {
    }
}