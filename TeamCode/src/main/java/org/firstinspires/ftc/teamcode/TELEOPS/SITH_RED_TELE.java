package org.firstinspires.ftc.teamcode.TELEOPS;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// OP MODE & CONFIGURATION
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
@Configurable
@TeleOp
public class SITH_RED_TELE extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    //-------------------
    //--DECLARE SERVOS---
    //-------------------
    private Servo odometryLift = null;
    private Servo hoodPitch = null;
    private Servo headLight = null;
    private Servo rgbLight = null;

    //-------------------
    //--DECLARE MOTORS---
    //-------------------
    private DcMotorEx shooter = null; // CHANGED TO DcMotorEx
    private DcMotorEx twoShooter = null; // CHANGED TO DcMotorEx
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;

    //-----------------------
    //--DECLARE AUXILIARIES--
    //-----------------------
    private Limelight3A limelight = null;
    private GoBildaPrismDriver prism = null;
    private DigitalChannel stopSensor = null;

    //-----------------------
    //---DECLARE VARIABLES---
    //-----------------------

    //Limelight PID
    private boolean isAiming;
    private static final double ROTATIONAL_kP = -0.015;
    private static final double ROTATIONAL_kI = -0.005; // Start with a very small value
    private double integralSum = 0;                     // Accumulator for the I term

    //Driving
    private boolean slowMode = false;
    private boolean driveMode = false; //False = Field Orient
    private boolean isEndGame;
    private double slowModeMultiplier = 0.25;
    private boolean odometryState = true;
    private boolean ballInTransfer = false;

    //-----------------------
    //---NEW CODE VARIABLES---
    //-----------------------
    private double shooterStartTime = 0;
    private boolean shooterActive = false;
    private double targetVelocity = 0;

    // MATH CONSTANTS: GoBilda 5203 (6000 RPM) has 28 Ticks Per Revolution
    private static final double MOTOR_TPR = 28.0;

    // NEW: Interpolation Logic Variables
    private double targetHoodPos = 1.0;
    private final double[] distanceTable = {-15.0, -5.0, 5.0, 15.0}; // ty values
    private final double[] velocityTable = {2850, 2550, 2250, 2050}; // shooter speeds (RPM)
    private final double[] hoodTable     = {0.45, 0.6, 0.8, 1.0};    // hood positions
    //-----------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// INIT
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        //-------------------
        //----INIT SERVOS----
        //-------------------
        odometryLift = hardwareMap.get(Servo.class, "Odometry");
        odometryLift.setPosition(0.7);
        headLight = hardwareMap.get(Servo.class, "Headlight");
        rgbLight = hardwareMap.get(Servo.class, "RGBLight");
        headLight.setPosition(0.3);
        rgbLight.setPosition(0.279);
        hoodPitch = hardwareMap.get(Servo.class, "hoodPitch");
        hoodPitch.setPosition(1.0);

        //-------------------
        //----INIT MOTORS----
        //-------------------
        // NEW CODE: Setup for encoder based control
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        twoShooter = hardwareMap.get(DcMotorEx.class, "2shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // NEW: Velocity PIDF Tuning (Note: F adjusted based on Max Ticks/Sec ~2800)
        shooter.setVelocityPIDFCoefficients(15, 3, 0, 12);
        twoShooter.setVelocityPIDFCoefficients(15, 3, 0, 12);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        twoShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        transfer = hardwareMap.get(DcMotorSimple.class,"transfer");


        //--------------------
        //--INIT AUXILIARIES--
        //--------------------
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        stopSensor = hardwareMap.get(DigitalChannel.class, "stop");
        stopSensor.setMode(DigitalChannel.Mode.INPUT);
    }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Start
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void start() {
        follower.startTeleopDrive();
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        limelight.start();
        telemetry.update();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addLine("GOOD LUCK SITH!");
    }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Loop
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();


        // When TeleOP time is over 100 seconds, Rumble Gamepad1
        if ((getRuntime() >= 99) && !isEndGame) {
            gamepad1.rumbleBlips(5);
            isEndGame = true;
        }

        //Limelight results
        LLResult result = limelight.getLatestResult();
        double tx = result.isValid() ? result.getTx() : 0;
        double ty = result.isValid() ? result.getTy() : 0; // NEW CODE: vertical offset for distance

        // NEW: Interpolation Logic for Distance (calculated every loop)
        if (result.isValid()) {
            targetVelocity = interpolate(ty, distanceTable, velocityTable);
            targetHoodPos = interpolate(ty, distanceTable, hoodTable);
        } else {
            targetVelocity = 6000; // Default RPM
        }

        //Enable auto aiming
        if (gamepad1.left_trigger > 0.50 || gamepad2.left_trigger > 0.50) {
            isAiming = true;
            if (result.isValid()) {
                // Accumulate the error for the Integral term
                integralSum += tx;

                // Anti-windup: Keep the sum within a range to prevent runaway spinning
                if (integralSum > 10) integralSum = 10;
                if (integralSum < -10) integralSum = -10;
            }
        } else {
            isAiming = false;
            integralSum = 0; // Reset sum when not aiming to prevent "jumping" next time you trigger
        }




        //-------------------
        // --DRIVE CONTROLS--
        //-------------------

        //Reset Heading
        if (gamepad1.y) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        //Slow Mode
        slowMode = gamepad1.left_bumper;

        if (isAiming && result.isValid()) {
            // Calculate PID power: (Proportional) + (Integral)
            double rotatePower = (tx * ROTATIONAL_kP) + (integralSum * ROTATIONAL_kI);

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    rotatePower,
                    driveMode // Robot Centric
            );
        } else if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5,
                driveMode // Robot Centric
        );
            //This is how it looks with slowMode on
        else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    driveMode // Robot Centric
            );


        //-------------------
        //----SUB SYSTEMS----
        //-------------------

        ballInTransfer = stopSensor.getState();

        //-------------------
        // NEW CODE: SHOOTER LOGIC (Velocity & Delay)
        //-------------------
        boolean shooterSpunUp = false;

        if (gamepad2.right_trigger >= 0.5) {
            if (!shooterActive) {
                shooterStartTime = getRuntime();
                shooterActive = true;
            }

            // MATH: Convert RPM (targetVelocity) to Ticks Per Second
            double ticksPerSec = (targetVelocity * MOTOR_TPR) / 60.0;

            shooter.setVelocity(ticksPerSec);
            twoShooter.setVelocity(-ticksPerSec);

            // Check if 1 second has passed
            if (getRuntime() - shooterStartTime >= 1.0) {
                shooterSpunUp = true;
            }
        } else {
            shooterActive = false;
            shooter.setVelocity(0);
            twoShooter.setVelocity(0);
        }

        //-------------------
        // NEW CODE: UPDATED TRANSFER LOGIC
        //-------------------
        // Run transfer if:
        // 1. Shooter is fully spun up (feeds the balls)
        // 2. OR intake is requested AND no ball is currently blocking the sensor
        if (shooterSpunUp) {
            transfer.setPower(1.0);
        } else if ((!ballInTransfer) && (gamepad1.right_trigger >= 0.5 || gamepad2.a)) {
            transfer.setPower(0.75);
        } else if (gamepad2.b) {
            transfer.setPower(-1.0);
        } else {
            transfer.setPower(0.0);
        }


        //intake
        if (shooterSpunUp) {
            intake.setPower(1.0);
        } else if ((gamepad1.right_trigger >= 0.5) || (gamepad2.a)) {
            intake.setPower(1.0);
        } else if (gamepad2.b) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        //Hood (NEW: Auto-Interpolation when aiming)
        if (isAiming && result.isValid()) {
            hoodPitch.setPosition(targetHoodPos);
        } else if (gamepad2.dpad_up) {
            hoodPitch.setPosition(0.6);
        } else if (gamepad2.dpad_down) {
            hoodPitch.setPosition(1.0);
        } else if (gamepad2.dpad_right) {
            hoodPitch.setPosition(0.8);
        } else if (gamepad2.dpad_left) {
            hoodPitch.setPosition(0.4);
        }


        //Toggle odometry lift position
        if (gamepad1.xWasPressed()) {
            odometryState = !odometryState;
        }

        //Raise or lower Odometry
        if (odometryState) {
            odometryLift.setPosition(0.7);
        } else {
            odometryLift.setPosition(0.0);
        }

        // NEW CODE: Additional Telemetry (Converts back to RPM for readout)
        telemetry.addData("Shooter1 RPM", (shooter.getVelocity() * 60.0) / MOTOR_TPR);
        telemetry.addData("Shooter2 RPM", (twoShooter.getVelocity() * 60.0) / MOTOR_TPR);
        telemetry.addData("Target RPM", targetVelocity);
        telemetry.addData("Spun Up?", shooterSpunUp);
        telemetry.addData("Target Hood", targetHoodPos);

        telemetry.addData("isAiming", isAiming);
        telemetry.addData("tx", tx);
        telemetry.addData("integralSum", integralSum); // Track the build up
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Valid Result", result.isValid());
        telemetry.addData("Ball In Transfer?", ballInTransfer);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

    }

    // NEW: Interpolation Helper Method
    private double interpolate(double input, double[] xTable, double[] yTable) {
        if (input <= xTable[0]) return yTable[0];
        if (input >= xTable[xTable.length - 1]) return yTable[yTable.length - 1];
        for (int i = 0; i < xTable.length - 1; i++) {
            if (input <= xTable[i + 1]) {
                double x0 = xTable[i], x1 = xTable[i + 1];
                double y0 = yTable[i], y1 = yTable[i + 1];
                return y0 + (input - x0) * (y1 - y0) / (x1 - x0);
            }
        }
        return yTable[yTable.length - 1];
    }
}
