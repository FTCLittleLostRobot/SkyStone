/* Little Lost Robots
   Core Devs: Danielle
*/


package org.firstinspires.ftc.teamcode.Skystone2019.Competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.CoreHex;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.CoreHexStateMachine;
import org.firstinspires.ftc.teamcode.Skystone2019.StateMachines.MecanumRotateStateMachine;

/**
 * This file provides basic Telop driving for the testing Mencanum robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Mencanum hardware class to define the devices on the robot.
 * All device access is managed through the HardwareMecanumBase2018 class.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Teleop v2", group="Mecanum")
public class MecanumTeleop_Iterative extends OpMode{

    /* Declare OpMode members. */
    private HardwareMecanumBase robot = new HardwareMecanumBase();
    private MecanumMotor motors = new MecanumMotor();
    private CoreHexStateMachine coreHexStateMachineBlockGrabber;
    private CoreHexStateMachine coreHexStateMachineBlockLifter;
    private float starting_left_x = 0;  //this makes the robot strafe right and left
    private float starting_left_y = 0;  //this makes the robot go forwards and backwards
    private float starting_right_x = 0; // this makes the robot rotate
    private boolean ButtonCheck = false;    //left and right bumper; faster, slower
    MecanumEncoderMove moveRobot;
    ElapsedTime holdTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Step 1: Setup of variables  */

        /* Step 2: Setup of hardware  */
        robot.init(hardwareMap);
        motors.init(robot);

        this.coreHexStateMachineBlockGrabber = new CoreHexStateMachine();
        this.coreHexStateMachineBlockGrabber.init(telemetry, robot, CoreHex.CoreHexMotors.BlockGrabber);

        this.coreHexStateMachineBlockLifter = new CoreHexStateMachine();
        this.coreHexStateMachineBlockLifter.init(telemetry, robot, CoreHex.CoreHexMotors.BlockLifter);

        this.moveRobot = new MecanumEncoderMove();
        this.moveRobot.init(motors);
        /* Step 3: Setup of controllers  */
        /* Step 4: Setup of state machines  */
        // NONE
        starting_left_x = -gamepad1.left_stick_x;
        starting_left_y = gamepad1.left_stick_y;
        starting_right_x= -gamepad1.right_stick_x;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //this shows the robot is ready
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left_stick_x;
        double left_stick_y;
        double right_stick_x;
        boolean left_bumper;    //boolean means it can only be true or false
        boolean right_bumper;

        left_stick_x = -gamepad1.left_stick_x - starting_left_x;
        left_stick_y = gamepad1.left_stick_y - starting_left_y;
        right_stick_x= -gamepad1.right_stick_x - starting_right_x;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_bumper = gamepad1.left_bumper;
        right_bumper= gamepad1.right_bumper;

        // if you hit the right bumper it will go faster if you hit the left bumper it will go slower, both will set it to regular
        if (left_bumper && right_bumper) {
            if (ButtonCheck == false){
                motors.IncreaseSpeed();
                // motors.ResetSpeed();
                ButtonCheck = true;
            }
        }
        else if (left_bumper) {
            if (ButtonCheck == false){
                motors.ResetSpeed();
                // motors.DecreaseSpeed();
                ButtonCheck = true;
            }
        }
        else if (right_bumper) {
            if (ButtonCheck == false) {
                motors.HighestSpeed();
                //motors.IncreaseSpeed();
                ButtonCheck = true;
            }
        }
        else{
            ButtonCheck = false;
        }

        if (gamepad2.x){

            coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Up);
        }
        else if (gamepad2.b){
            coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.Down);

        }

        if (gamepad2.y){
            coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Up);
        }
        else if (gamepad2.a){
            coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.Down);
        }

        if (gamepad2.right_bumper){
            coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.ControlledUp);
        }
        else if (gamepad2.left_bumper){
            coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.ControlledDown);
        }

        if (gamepad2.dpad_down){
            coreHexStateMachineBlockLifter.Start(CoreHex.RotationDirection.DropBlockLifter);
            coreHexStateMachineBlockGrabber.Start(CoreHex.RotationDirection.DropBlockGrabber);
            this.moveRobot.StartMove(20, 1, 0, MecanumEncoderMove.GO_BACK, 0);
            holdTimer.reset();
           /*
            if ((holdTimer.time() <= 2)){
                this.motors.SetSpeedToValue(15);
            }
            else {
                this.motors.MoveMecanum(0, 0, 0);
                this.motors.ResetMotors();
            }

            */
        }

        if (robot.Block_Grabber != null && robot.Block_Lifter != null) {
            coreHexStateMachineBlockGrabber.ProcessState();
            coreHexStateMachineBlockLifter.ProcessState();
        }

        motors.MoveMecanum(left_stick_x, left_stick_y, right_stick_x);

        telemetry.addData("SpeedMultplier", motors.GetSpeedMultiplier());
        telemetry.addData("left stick y value", gamepad1.left_stick_y);
        telemetry.addData("left stick x value", gamepad1.left_stick_x);
        telemetry.addData("right stick x value", gamepad1.right_stick_x);


        telemetry.addData("target position Lifter", coreHexStateMachineBlockLifter.GetCoreHexNextPosition());
        telemetry.addData("target position Grabber", coreHexStateMachineBlockGrabber.GetCoreHexNextPosition());
        telemetry.update();
        if (robot.Block_Grabber != null && robot.Block_Lifter != null) {
            telemetry.addData("Lifter current position", robot.Block_Lifter.getCurrentPosition());
            telemetry.addData("grabber current position", robot.Block_Grabber.getCurrentPosition());
            telemetry.update();
        }
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
