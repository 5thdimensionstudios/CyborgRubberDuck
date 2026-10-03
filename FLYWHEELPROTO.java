package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Flywheel RPM Prototype", group="Linear OpMode")
public class FlywheelRPMTest extends LinearOpMode {
    private static final String Left_Motor_Name = "Insert Actual Name";
    private static final String Right_Motor_Name = "Insert Actual Name";

    private static final double Motor_Ticks_Per_Rev = 0; //insert specific ticks per rev for the motor

    private static final boolean Reverse_Left = false;
    private static final boolean Reverse_Right = false;

    //Test Data Goes Here

    //List Distance and RPM in low to high order

    private static final double[] Test_Distance_M = {
            //Put distances here
    };
    private static final double[] Test_RPM = {
            //Put RPM here
    };

    //Distance Controlls
    private static final double Start_Distance_M = 1.0;

    private static final double Distance_Step_M = 0.5;

    private static final double Min_Distance_M = 0.5;
    private static final double Max_Distance_M = 10.0;

    //RPM Controlls

    private static final double Start_RPM = 1000;

    private static final double RPM_Step = 250;

    private static final double Min_RPM = 0;

    private static final double Max_RPM = 6000;

    //Motor Declaring

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private double targetRPM = Start_RPM;
    private double targetDistance = Start_Distance_M;

    private boolean flywheelsRunning = false;

    //set automaticMode to false if you are manual testing and true if your ding automatic distance
    private boolean automaticMode = false;

    //Button state tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    public void runOpMode() {
        //Hardware

        leftFlywheel = hardwareMap.get(DcMotorEx.class, Left_Motor_Name);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, Right_Motor_Name);

        leftFlywheel.setDirection(
                Reverse_Left
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD
        );

        rightFlywheel.setDirection(
                Reverse_Right
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD
        );

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);

        telemetry.addLine("FLYWHEEL TEST SYSTEM");
        telemetry.addLine("--------------------");
        telemetry.addLine("A = Start / Stop");
        telemetry.addLine("B = Manual / Automatic");
        telemetry.addLine("X = Emergency Stop");
        telemetry.addLine("Y = Reset");
        telemetry.addLine("");
        telemetry.addLine("MANUAL MODE:");
        telemetry.addLine("UP/DOWN = Change RPM");
        telemetry.addLine("");
        telemetry.addLine("AUTO MODE:");
        telemetry.addLine("LEFT/RIGHT = Change distance");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //Mode Switch

            if (gamepad1.b && !lastB) {
                automaticMode = !automaticMode;

                if (automaticMode) {
                    targetRPM = getRPMForDistance(targetDistance);
                }
            }

            if (!automaticMode) {
                if (gamepad1.dpad_up && !lastUp) {
                    targetRPM += RPM_Step;

                    targetRPM = Math.min(targetRPM, Max_RPM);
                }
                if (gamepad1.dpad_down && !lastDown) {
                    targetRPM -= RPM_Step;

                    targetRPM = Math.max(targetRPM, Min_RPM);
                }
            }

            if (automaticMode) {
                if (gamepad1.dpad_right && !lastRight) {
                    targetDistance += Distance_Step_M;

                    targetDistance = Math.min(targetDistance, Max_Distance_M);

                    targetRPM = getRPMForDistance(targetDistance);
                }
                if (gamepad1.dpad_left && !lastLeft) {
                    targetDistance -= Distance_Step_M;

                    targetDistance = Math.max(targetDistance, Min_Distance_M);

                    targetRPM = getRPMForDistance(targetDistance);
                }
            }

            if (gamepad1.a && !lastA) {
                flywheelsRunning = !flywheelsRunning;
            }

            if (gamepad1.x && !lastX) {
                flywheelsRunning = false;
                targetRPM = 0;
            }

            if (gamepad1.y && !lastY) {
                targetRPM = Start_RPM;
                targetDistance = Start_Distance_M;
                automaticMode = false;
                flywheelsRunning = false;
            }

            if (flywheelsRunning && targetRPM > 0) {
                double targetTicksPerSecond = (targetRPM / 60.0) * Motor_Ticks_Per_Rev;

                leftFlywheel.setVelocity(targetTicksPerSecond);
                rightFlywheel.setVelocity(targetTicksPerSecond);
            }
            else {
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
            }

            double leftTicksperSecond = Math.abs(leftFlywheel.getVelocity());
            double rightTicksPerSecond = Math.abs(rightFlywheel.getVelocity());

            double leftRPM = (leftTicksperSecond / Motor_Ticks_Per_Rev) * 60.0;
            double rightRPM = (rightTicksPerSecond / Motor_Ticks_Per_Rev) * 60.0;
            double averageRPM = (leftRPM + rightRPM) / 2.0;

            //Telemetry

            telemetry.addLine("FLYWHEEL TEST SYSTEM");
            telemetry.addLine("--------------------");

            telemetry.addData(
                    "Mode",
                    automaticMode
                            ? "AUTOMATIC"
                            : "MANUAL"
            );

            telemetry.addData(
                    "Status",
                    flywheelsRunning
                            ? "RUNNING"
                            : "STOPPED"
            );

            if (automaticMode) {

                telemetry.addData(
                        "Target Distance",
                        "%.2f m",
                        targetDistance
                );

                telemetry.addData(
                        "Calculated RPM",
                        "%.0f",
                        targetRPM
                );
            }

            telemetry.addData(
                    "Target RPM",
                    "%.0f",
                    targetRPM
            );

            telemetry.addData(
                    "Left RPM",
                    "%.0f",
                    leftRPM
            );

            telemetry.addData(
                    "Right RPM",
                    "%.0f",
                    rightRPM
            );

            telemetry.addData(
                    "Average RPM",
                    "%.0f",
                    averageRPM
            );

            if (Test_Distance_M.length == 0) {

                telemetry.addLine(
                        "NO EXPERIMENTAL DATA ENTERED"
                );

            } else {

                telemetry.addData(
                        "Data Points",
                        "%d",
                        Test_Distance_M.length
                );

                telemetry.addData(
                        "Measured Range",
                        "%.2f - %.2f m",
                        Test_Distance_M[0],
                        Test_Distance_M[
                                Test_Distance_M.length - 1
                                ]
                );
            }

            telemetry.update();

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

        }
    }

    //Calculations
    private double getRPMForDistance(double distance) {
        int numberOfPoints = Test_Distance_M.length;

        if (numberOfPoints == 0) {
            return 0;
        }

        if (Test_RPM.length != numberOfPoints) {
            return 0;
        }

        if (distance <= Test_Distance_M[0]) {
            return Test_RPM[0];
        }

        if (distance >= Test_Distance_M[numberOfPoints - 1]) {
            return Test_RPM[numberOfPoints - 1];
        }

        //Linear Interpolation
        for (int i = 0; i < numberOfPoints - 1; i++) {
            double distance1 = Test_Distance_M[i];
            double distance2 = Test_Distance_M[i + 1];
            double rpm1 = Test_RPM[i];
            double rpm2 = Test_RPM[i + 1];

            if (distance >= distance1 && distance <= distance2) {
                double fraction = (distance - distance1) / (distance2 - distance1);

                double calculatedRPM = rpm1 + fraction * (rpm2 - rpm1);

                return calculatedRPM;
            }
        }

        return 0;
    }
}
