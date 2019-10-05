/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveFoundationStateMachine;

@Autonomous(name="Mecanum:practice", group="Mecanum")
public class MecanumPractice_Iterative extends OpMode {

    private HardwareMecanumBase robot;
    private MecanumMoveFoundationStateMachine moveFoundationStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.robot = new HardwareMecanumBase();
        this.moveFoundationStateMachine = new MecanumMoveFoundationStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */
        this.moveFoundationStateMachine.init(telemetry, robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void start() {
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