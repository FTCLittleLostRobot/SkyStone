package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

public class GyroController {

    private ModernRoboticsI2cGyro externalGyro = null;


    public void init(HardwareMecanumBase hwBase) {

        externalGyro = hwBase.ExternalGyro;
    }

    public void StartCalibration()
    {
        externalGyro.calibrate();
    }

    public Boolean IsDoneCalibrating()
    {
        return externalGyro.isCalibrating();
    }

    public int GetCurrentRobotHeading()
    {
        return externalGyro.getIntegratedZValue();
    }

    public void ResetZIndexIntegrator()
    {
        externalGyro.resetZAxisIntegrator();
    }

    public double GetErrorToTarget(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - externalGyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double GetSteeringForce(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}