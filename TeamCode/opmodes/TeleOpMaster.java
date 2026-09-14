package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Auto Master", group="Autonomous")
public class TeleOpMaster extends LinearOpMode {

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
            double driveForward = -gamepad1.left_stick_y;
            double driveStrafe = gamepad1.left_stick_x;
            double driveTurn = gamepad1.right_stick_x;
            // Example routine call
            robot.drivetrain.driveMecanum(driveForward, driveStrafe,driveTurn);

            // Send diagnostic readouts to Driver Station phone
            telemetry.addData("Status", "Running");
            telemetry.addData("Heading", "%.1f", robot.drivetrain.getHeading());
            telemetry.addData("Right Distance (cm)", "%.1f", robot.sensors.rightDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance (cm)", "%.1f", robot.sensors.leftDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}