/* Little Lost Robots
   Core Devs: Danielle, Ryan
*/

package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveFoundationStateMachine;

@Autonomous(name="Mecanum:Foundation", group="Mecanum")
public class MecanumFoundation_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private MecanumMoveFoundationStateMachine moveFoundationStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.moveFoundationStateMachine = new MecanumMoveFoundationStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Press Blue Or Red Button To Select Team");    //
    }


    @Override
    public void init_loop() {
        if (gamepad1.x){
            this.moveFoundationStateMachine.init(telemetry, motors, true, false);
            telemetry.addData("Say", "Welcome to the blue team");
        }
        else if (gamepad1.b){
            this.moveFoundationStateMachine.init(telemetry, motors, true, true);
            telemetry.addData("Say", "Welcome to the Red Team");
        }
    }

    @Override
    public void start() {

        this.moveFoundationStateMachine.init(telemetry, motors, true, false);
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