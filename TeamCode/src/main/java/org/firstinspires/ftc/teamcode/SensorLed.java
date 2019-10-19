
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


@TeleOp(name = "Sensor: LED", group = "Iterative OpMode")
public class SensorLed extends OpMode {

    public final double LED_RED = 0.6695;
    public final double LED_RED2 = 0.5145;
    public final double LED_BLUE = 0.7445;
    public final double LED_BLUE2 = 0.5695;
    private HashMap<String, Double> map = new HashMap<>();
    private List<String> mapValues;
    private final double SECOND_COLOR_ADD = .0550;

    public Servo servoLed = null;
    private int currentColor = 0;
    private boolean isLeftBumperPressed = false;
    private boolean isRigthBumperPressed = false;

    public void init() {
        servoLed = hardwareMap.servo.get("TeamColor");

        // Red = 4945 -> 5445
        // Blue = 5495 -> 5995
        // 550 Difference
        // First color = RED
        // Second color = BLUE (Red + ColorDifference (550))
        map.put("LarsonScanner", .4995);        // 0
        map.put("LightChase", .5045);           // 1
        map.put("HeartBeatSlow", .5095);        // 2
        map.put("HeartBeatMedium", .5145);      // 3
        map.put("HeartBeatFast", .5195);        // 4
        map.put("BreathSlow", .5245);           // 5
        map.put("BreathFast", .5295);           // 6
        currentColor = 0;
        mapValues = new ArrayList<String>(map.keySet());
    }

    @Override
    public void loop() {

        telemetry.addData("Servo is at", servoLed.getPosition());
        telemetry.addData("CurrentColor", currentColor);
        telemetry.addData("Color", mapValues.get(currentColor));

        if (gamepad1.left_bumper)
        {
            if (!isLeftBumperPressed) {
                currentColor = currentColor + 1;
                if (currentColor >= map.size()) {
                    currentColor = 0;
                }
                servoLed.setPosition(map.get(mapValues.get(currentColor)));
                isLeftBumperPressed = true;
            }
        }
        else
        {
            isLeftBumperPressed = false;
        }




        if (gamepad1.right_bumper)
        {
            if (!isRigthBumperPressed) {
                currentColor = currentColor + 1;
                if (currentColor >= map.size()) {
                    currentColor = 0;
                }
                servoLed.setPosition( map.get( mapValues.get( currentColor ) ) + SECOND_COLOR_ADD);
                isRigthBumperPressed = true;
            }
        }
        else
        {
            isRigthBumperPressed = false;
        }

        if (gamepad1.x) {
            telemetry.addData("Say", "Welcome to the blue team");

            servoLed.setPosition(LED_BLUE);


        } else if (gamepad1.b) {
            telemetry.addData("Say", "Welcome to the Red Team");

            servoLed.setPosition(LED_RED);


        } else if (gamepad1.a) {
            telemetry.addData("Say", "Welcome to the Blue2 Team");

            servoLed.setPosition(LED_BLUE2);

        } else if (gamepad1.y) {
            telemetry.addData("Say", "Welcome to the Red2 Team");

            servoLed.setPosition(LED_RED2);


        }

    }
}
