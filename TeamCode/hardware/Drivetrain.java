package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Drivetrain {

    // Motors representing the 4 drivetrain wheels

    public static double kP = 0.015;
    public static double kI = 0.0005;
    public static double kD = 0.001;
    public DcMotorEx FLDrive = null;
    public DcMotorEx FRDrive = null;
    public DcMotorEx BLDrive = null;
    public DcMotorEx BRDrive = null;

    private final PIDController turnPID = new PIDController(kP, kI, kD);
    // Inertial Measurement Unit (Gyroscope) for heading tracking
    public IMU imu = null;

    // Reference to the active running OpMode (used for telemetry and checking if match is active)
    private final LinearOpMode opMode;

    // --- ENCODER CONSTANTS ---
    // Physical hardware constants used to calculate distance from encoder ticks
    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 3.77953;
    public static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Constructor: Saves the reference to the running OpMode
    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Hardware Mapping: Binds physical REV Hub ports to code variables and sets initial states
    public void init(HardwareMap hwMap) {
        // Fetch motor references using the configuration names set on the Driver Station
        FLDrive = hwMap.get(DcMotorEx.class, "4FL");
        FRDrive = hwMap.get(DcMotorEx.class, "3FR");
        BLDrive = hwMap.get(DcMotorEx.class, "2BL");
        BRDrive = hwMap.get(DcMotorEx.class, "1BR");

        // Reverse right side motors so positive power moves the entire robot forward
        FLDrive.setDirection(DcMotorEx.Direction.FORWARD);
        FRDrive.setDirection(DcMotorEx.Direction.REVERSE);
        BLDrive.setDirection(DcMotorEx.Direction.FORWARD);
        BRDrive.setDirection(DcMotorEx.Direction.REVERSE);

        // Configure motors to actively brake (stop instantly) when power drops to 0
        FLDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // IMU Initialization and physical orientation definition on the robot
        imu = hwMap.get(IMU.class, "IMUfrancis");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Clear encoder readings on startup
        resetEncoders();
    }

    // --- BASIC MOVEMENT ---

    // Sets raw motor power levels (-1.0 to 1.0) to all 4 drive motors
    public void setDrivePowers(double fl, double fr, double bl, double br) {
        FLDrive.setPower(fl);
        FRDrive.setPower(fr);
        BLDrive.setPower(bl);
        BRDrive.setPower(br);
    }

    // Drives straight forward at the specified power level
    public void driveForward(double power) {
        setDrivePowers(power, power, power, power);
    }

    // Cuts power to all drive motors
    public void stopRobot() {
        setDrivePowers(0, 0, 0, 0);
    }

    // --- MECANUM ROBOT-CENTRIC DRIVE ---

    // Translates joystick inputs (strafe, forward/backward, turn) into Mecanum motor powers
    public void driveMecanum(double x, double y, double rx) {
        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        // Scale down motor values proportionally if any calculated power exceeds 1.0
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(blPower),
                Math.max(Math.abs(frPower), Math.abs(brPower))));

        if (max > 1.0) {
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        setDrivePowers(flPower, frPower, blPower, brPower);
    }

    // --- ENCODER LOGIC ---

    // Resets encoder tick counts back to 0 and puts motors in unencoded run mode
    public void resetEncoders() {
        FLDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Returns the average distance traveled in inches across all 4 encoders
    public double getAverageEncoderDistanceInches() {
        int avgTicks = getAverageEncoderPosition();
        return avgTicks / COUNTS_PER_INCH;
    }

    // Blocking encoder method: Drives a specific distance in inches before stopping
    public void driveDistance(double distanceInches, double power) {
        resetEncoders();
        double targetTicks = distanceInches * COUNTS_PER_INCH;
        double p = (distanceInches < 0) ? -Math.abs(power) : Math.abs(power);
        driveForward(p);

        // Keep driving until the average encoder position reaches the target ticks
        while (opMode.opModeIsActive() && Math.abs(getAverageEncoderPosition()) < Math.abs(targetTicks)) {
            opMode.telemetry.addData("Target Inches", distanceInches);
            opMode.telemetry.addData("Current Inches", "%.2f", getAverageEncoderDistanceInches());
            opMode.telemetry.update();
        }
        stopRobot();
    }

    // --- GYRO / HEADING LOGIC ---

    // Reads current heading (yaw) from the IMU in degrees
    public double getHeading() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Resets current orientation heading back to 0 degrees
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    // Proportional control loop to turn the robot in place to a specific target angle
    public double turnToHeadingStationary(double targetHeading) {
        turnPID.reset(); // Clear previous error accumulation and timing state

        double maxPower = 0.7;
        double minPower = 0.12; // Minimum threshold to overcome friction

        while (opMode.opModeIsActive()) {
            turnPID.setGains(kP, kI, kD);

            double currentHeading = getHeading();
            double error = normalizeError(targetHeading - currentHeading);

            // Exit loop when error is within tolerance
            if (Math.abs(error) < 0.5) {
                break;
            }

            // Pass (current + error) as target to handle -180/180 angle wrapping smoothly
            double turnPower = turnPID.calculate(currentHeading + error, currentHeading);

            // Clamp power limits
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            // Enforce minimum power threshold
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            // Apply turning power to drive motors
            setDrivePowers(-turnPower, turnPower, -turnPower, turnPower);

            opMode.telemetry.addData("Target", "%.1f", targetHeading);
            opMode.telemetry.addData("Heading", "%.1f", currentHeading);
            opMode.telemetry.addData("Error", "%.1f", error);
            opMode.telemetry.addData("PID Output", "%.2f", turnPower);
            opMode.telemetry.update();
        }

        stopRobot();
        return getHeading();
    }

    // Helper: Calculates average encoder position across all 4 drive motors
    private int getAverageEncoderPosition() {
        return (FLDrive.getCurrentPosition() + FRDrive.getCurrentPosition() +
                BLDrive.getCurrentPosition() + BRDrive.getCurrentPosition()) / 4;
    }

    // Helper: Keeps target angle errors wrapped between -180 and 180 degrees
    private double normalizeError(double error) {
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}