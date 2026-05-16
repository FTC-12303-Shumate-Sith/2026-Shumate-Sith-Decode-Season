package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    private DcMotorEx shooter, twoShooter;
    private Servo hoodPitch;
    private ElapsedTime timer = new ElapsedTime();

    private boolean shooterActive = false;
    private double targetVelocity = 0;
    private double targetHoodPos = 1.0;

    // MATH CONSTANTS
    private static final double MOTOR_TPR = 28.0;
    private final double[] distanceTable = { -10,   -8,   -6,   -4,   -2,    0,    2,    6,   10,   12,   15};
    private final double[] velocityTable = {6000, 6000, 5900, 5600, 5300, 5200, 4900, 4300, 4300, 4000, 3800};
    private final double[] hoodTable     = { 0.4,  0.4,  0.4,  0.4,  0.4,  0.4, 0.47, 0.52, 0.58, 0.72,  0.8};

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        twoShooter = hardwareMap.get(DcMotorEx.class, "2shooter");
        hoodPitch = hardwareMap.get(Servo.class, "hoodPitch");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocityPIDFCoefficients(350, 0, 0, 15.6);
        twoShooter.setVelocityPIDFCoefficients(350, 0, 0, 15.6);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hoodPitch.setPosition(1.0);
    }

    public void update(double ty, boolean isValid) {
        if (isValid) {
            targetVelocity = interpolate(ty, distanceTable, velocityTable);
            targetHoodPos = interpolate(ty, distanceTable, hoodTable);
        } else {
            targetVelocity = 3900;
        }
    }

    public void run(boolean active) {
        if (active) {
            if (!shooterActive) {
                timer.reset();
                shooterActive = true;
            }
            double ticksPerSec = (targetVelocity * MOTOR_TPR) / 60.0;
            shooter.setVelocity(ticksPerSec);
            twoShooter.setVelocity(-ticksPerSec);
        } else {
            shooterActive = false;
            shooter.setVelocity(0);
            twoShooter.setVelocity(0);
        }
    }

    public boolean isReady() {
        if (!shooterActive) return false;
        double currentRPM = (Math.abs(shooter.getVelocity()) * 60.0) / MOTOR_TPR;
        // Ready if RPM is within 150 of target OR 1.1 seconds passed
        return (Math.abs(currentRPM - targetVelocity) < 150) || (timer.seconds() > 1.1);
    }

    public void setHood(double pos) { hoodPitch.setPosition(pos); }
    public double getTargetHood() { return targetHoodPos; }
    public double getTargetRPM() { return targetVelocity; }
    public double getCurrentRPM() { return (shooter.getVelocity() * 60.0) / MOTOR_TPR; }

    private double interpolate(double input, double[] xTable, double[] yTable) {
        if (input <= xTable[0]) return yTable[0];
        if (input >= xTable[xTable.length - 1]) return yTable[yTable.length - 1];
        for (int i = 0; i < xTable.length - 1; i++) {
            if (input <= xTable[i + 1]) {
                double percent = (input - xTable[i]) / (xTable[i + 1] - xTable[i]);
                return yTable[i] + percent * (yTable[i + 1] - yTable[i]);
            }
        }
        return yTable[0];
    }
}