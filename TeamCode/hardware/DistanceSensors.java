package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors {

    // Sensor objects for left and right distance readings
    public DistanceSensor RSensor = null;
    public DistanceSensor LSensor = null;

    // Hardware Mapping: Connects variables to physical sensor names on the hub
    public void init(HardwareMap hwMap) {
        RSensor = hwMap.get(DistanceSensor.class, "3distance");
        LSensor = hwMap.get(DistanceSensor.class, "4distance");
    }

    // Returns distance from right sensor in specified unit (CM or INCHES)
    public double rightDistance(DistanceUnit unit) {
        return RSensor.getDistance(unit);
    }

    // Returns distance from left sensor in specified unit (CM or INCHES)
    public double leftDistance(DistanceUnit unit) {
        return LSensor.getDistance(unit);
    }
}