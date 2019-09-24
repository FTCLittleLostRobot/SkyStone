/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMove;

@Autonomous(name="Mecanum:Rotation", group="Mecanum")
public class MecanumRotation_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumMove moveRobot;
    private MecanumRotateStateMachine rotateStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.moveRobot = new MecanumMove();
        this.rotateStateMachine = new MecanumRotateStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);

        /* Step 3: Setup of controllers  */
        this.moveRobot.init(robot);
        //float right_stick_x;
        //right_stick_x= 1;


        /* Step 4: Setup of state machines  */
        this.rotateStateMachine.init(telemetry, 1.0, moveRobot);
        //todo why won't right stick and left stick work?
        //robot.MoveMecanum(0, 0, right_stick_x);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void start() {
        rotateStateMachine.Start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        //this goes into the StateMachine, "SamplingStateMachine" and then goes through all the states it needs
        rotateStateMachine.ProcessState();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}