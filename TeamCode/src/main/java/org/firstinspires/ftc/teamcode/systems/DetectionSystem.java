package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class DetectionSystem {
    ColorSensor sleeveSensor;

    public enum Signal {
        ONE, TWO, THREE
    }

    public DetectionSystem(LinearOpMode opMode) {
        sleeveSensor = opMode.hardwareMap.get(ColorSensor.class, "colorSensorLeft");
    }

    public Signal scan() {
        if (sleeveSensor.red() > sleeveSensor.green() && sleeveSensor.red() > sleeveSensor.blue()) {
            return Signal.ONE;
        } else if (sleeveSensor.green() > sleeveSensor.red() && sleeveSensor.green() > sleeveSensor.blue()) {
            return Signal.TWO;
        } else if (sleeveSensor.blue() > sleeveSensor.green() && sleeveSensor.blue() > sleeveSensor.red()) {
            return Signal.THREE;
        }
        return scan();
    }
}
