package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name="Auto Master", group="Autonomous")
public class AutoMaster extends LinearOpMode {

    // Holds the central robot container reference
    private Robot robot;

    @Override
    public void runOpMode() {
        // Creates Robot instance and link hardware
        robot = new Robot(this);
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Pauses program until driver presses START button
        waitForStart();

        // Main autonomous routine execution loop
        while (opModeIsActive()) {
            // Example routine call
            robot.drivetrain.driveForward(0.5);

            // Send diagnostic readouts to Driver Station phone
            telemetry.addData("Status", "Running");
            telemetry.addData("Heading", "%.1f", robot.drivetrain.getHeading());
            telemetry.addData("Right Distance (cm)", "%.1f", robot.sensors.rightDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance (cm)", "%.1f", robot.sensors.leftDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}