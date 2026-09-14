package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        integralSum += error * dt;

        if (Math.abs(error) > 500) {
            integralSum = 0;
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;
        return (error * kP) + (integralSum * kI) + (derivative * kD);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}