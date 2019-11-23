package org.firstinspires.ftc.teamcode.Skystone2019.Controllers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Skystone2019.HardwareMecanumBase;

import java.io.File;
import java.util.Locale;

public class GyroController {

    private ModernRoboticsI2cGyro externalGyro = null;
    private BNO055IMU imu = null;
    private Boolean useInternalGyro = true;


    public void init(HardwareMecanumBase hwBase) {

        if (useInternalGyro)
        {
            this.imu = hwBase.IMU;
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            this.imu.initialize(parameters);
        }
        else {
            externalGyro = hwBase.ExternalGyro;
        }
    }

    public void StartCalibration()
    {
        if (useInternalGyro)
        {
            // Calibration needs to be done with Teleop program.
        }
        else {
            externalGyro.calibrate();
        }
    }

    public Boolean IsDoneCalibrating()
    {
        if (useInternalGyro)
            return true;

        else
            return !externalGyro.isCalibrating();
    }

    public int GetCurrentRobotHeading()
    {
        if (useInternalGyro)
        {
            Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return (int)AngleUnit.DEGREES.normalize(angles.firstAngle);
        }
        else {
            return externalGyro.getIntegratedZValue();
        }
    }

    public void ResetZIndexIntegrator()
    {
        if (useInternalGyro)
        {
            // No way to reset hte zindex for imu
        }
        else {
            externalGyro.resetZAxisIntegrator();
        }
    }

    public double GetErrorToTarget(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - this.GetCurrentRobotHeading();
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
        // Takes an error, such as -90 degrees, multiplies it by the PCoeff (.1) to get a new value.
        // That would be -9.  -9 is below the min, so it will be a negative value to that direction.
        return Range.clip(error * PCoeff, -1, 1);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}