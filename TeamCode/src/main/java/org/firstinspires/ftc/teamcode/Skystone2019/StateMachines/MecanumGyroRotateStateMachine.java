/* Little Lost Robots
   Core Devs: Danielle
*/

package org.firstinspires.ftc.teamcode.Skystone2019.StateMachines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumEncoderMove;
import org.firstinspires.ftc.teamcode.Skystone2019.Controllers.MecanumMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumGyroRotateStateMachine {

    Telemetry telemetry;
    MecanumEncoderMove moveRobot;
    MecanumGyroRotateStateMachine.RobotState state;
    MecanumMotor motors;
    ModernRoboticsI2cGyro gyro;
    double speed;
    double angle;
    ElapsedTime holdTimer = new ElapsedTime();

    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    enum RobotState
    {
        Calibrating,
        StartRotation,
        DoneRotating,
        StartHold,
        Holding,
        Done
    }

    public void init(Telemetry telemetry, MecanumMotor motors, ModernRoboticsI2cGyro gyro) {
        this.motors = motors;
        this.telemetry = telemetry;
        this.gyro = gyro;
        motors.SetMecanumBreak();
        gyro.calibrate();
    }

    public void Start(double speed, double angle)
    {
        this.speed = speed;
        this.angle = angle;
        state = MecanumGyroRotateStateMachine.RobotState.Calibrating;
    }

    public boolean IsDone()
    {
        return (state == MecanumGyroRotateStateMachine.RobotState.Done);
    }

    public void ProcessState()
    {

        switch (state)
        {
            case Calibrating:
                if(!gyro.isCalibrating()){
                    gyro.resetZAxisIntegrator();
                    state = MecanumGyroRotateStateMachine.RobotState.StartRotation;
                }
                break;

            case StartRotation:

                if (motors.MovingToHeadingGyro(speed, angle, P_TURN_COEFF, gyro)){
                    holdTimer.reset();
                    state = RobotState.StartHold;
                }


                break;

            case StartHold:
                if ((holdTimer.time() <= 2)) {
                    // Update telemetry & Allow time for other processes to run.
                    motors.MovingToHeadingGyro(speed, angle, P_TURN_COEFF, gyro);
                 }
                else {
                    state = MecanumGyroRotateStateMachine.RobotState.Done;
                }
                break;

            case Done:
                state = MecanumGyroRotateStateMachine.RobotState.Done;
                break;
        }

    }
}
