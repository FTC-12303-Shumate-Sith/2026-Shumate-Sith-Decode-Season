package org.firstinspires.ftc.teamcode.TELEOPS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Disabled
public class FlywheelTunerTutorial extends OpMode {
    // Two motors like SITH RED TELE
    public DcMotorEx shooter;
    public DcMotorEx twoShooter;

    // --- Standardized Constants ---
    // 1 to 1 pulley ratio as requested
    public static final double GEAR_RATIO = 1.0;
    // GoBilda 6000 RPM motor encoder counts
    public static final double ENCODER_CPR = 28;

    // --- Target Velocities (in RPM) ---
    public static final double HIGH_SHOT_RPM = 4000;
    public static final double LOW_SHOT_RPM = 2500;

    // Required motor velocity in ticks per second
    public double highVelocity = (HIGH_SHOT_RPM * GEAR_RATIO / 60) * ENCODER_CPR;
    public double lowVelocity = (LOW_SHOT_RPM * GEAR_RATIO / 60) * ENCODER_CPR;

    double curTargetVelocity = highVelocity;

    // Initial PIDF coefficients for tuning.
    // Note: Since you use (15, 3, 0, 12) in your teleop, these are good starting points.
    double F = 12.0;
    double P = 15.0;
    double I = 0;

    // Array of step sizes for making adjustments.
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        // Hardware Map names matching SITH RED TELE
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        twoShooter = hardwareMap.get(DcMotorEx.class, "2shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Matching your Teleop Direction
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        twoShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Apply initial PIDF
        updatePIDF();

        telemetry.addLine("Tuning Init Complete - Two Motors Ready");
    }

    @Override
    public void loop() {
        // --- Gamepad Controls for Tuning ---

        // Toggle Velocity
        if (gamepad1.yWasPressed()) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // Cycle Step Size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Adjust F (Feedforward)
        if (gamepad1.dpadLeftWasPressed()) F -= stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F += stepSizes[stepIndex];

        // Adjust P (Proportional)
        if (gamepad1.dpadUpWasPressed()) P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed()) P -= stepSizes[stepIndex];

        // Apply updated coefficients to both motors
        updatePIDF();

        // Command both motors
        shooter.setVelocity(curTargetVelocity);
        twoShooter.setVelocity(-curTargetVelocity); // Negative because they face opposite directions

        // --- Telemetry Output ---
        double currentTicks = shooter.getVelocity();
        double currentRPM = (currentTicks / ENCODER_CPR) * 60.0;
        double targetRPM = (curTargetVelocity == highVelocity) ? HIGH_SHOT_RPM : LOW_SHOT_RPM;

        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Actual RPM", "%.2f", currentRPM);
        telemetry.addData("Error (RPM)", "%.2f", targetRPM - currentRPM);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P Gain", "%.4f (D-Pad U/D)", P);
        telemetry.addData("F Gain", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Motor 1 Ticks/s", "%.1f", shooter.getVelocity());
        telemetry.addData("Motor 2 Ticks/s", "%.1f", twoShooter.getVelocity());
    }

    private void updatePIDF() {
        PIDFCoefficients coefficients = new PIDFCoefficients(P, I, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        twoShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
}