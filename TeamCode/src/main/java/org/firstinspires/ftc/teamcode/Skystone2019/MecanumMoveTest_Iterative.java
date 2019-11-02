/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumMoveStateMachine;

@Autonomous(name="Mecanum:MoveTest", group="Mecanum")
public class MecanumMoveTest_Iterative extends OpMode {

    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private MecanumRotateStateMachine rotateStateMachine;
    private MecanumMoveStateMachine moveStateMachine;

    @Override
    public void init() {
        /* Step 1: Setup of variables  */
        this.rotateStateMachine = new MecanumRotateStateMachine();
        this.moveStateMachine = new MecanumMoveStateMachine();

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        /* Step 3: Setup of controllers  */

        /* Step 4: Setup of state machines  */
        //this.rotateStateMachine.init(telemetry, motors);
        this.moveStateMachine.init(telemetry, motors);

        //this.rotateStateMachine = new MecanumRotateStateMachine();
        //this.rotateStateMachine.init(telemetry, motors);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    @Override
    public void start() {
        //rotateStateMachine.Start(90.0);
        moveStateMachine.Start(2, -1, -1, 0);
    }

    @Override
    public void loop()
    {
        moveStateMachine.ProcessState();

//        if (moveStateMachine.IsDone()) {
//            rotateStateMachine.ProcessState();
//        }
    }

    @Override
    public void stop() {
    }
}