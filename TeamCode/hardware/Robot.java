package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    // Declares all subsystem components managed by this container
    public Drivetrain drivetrain;
    public DistanceSensors sensors;

    // Constructor: Allocates memory for subsystems and passes OpMode reference down
    public Robot(LinearOpMode opMode) {
        drivetrain = new Drivetrain(opMode);
        sensors = new DistanceSensors();
    }

    // Central Init: Calls init on all subsystems, passing hardware map from REV Hub
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        sensors.init(hwMap);
    }
}