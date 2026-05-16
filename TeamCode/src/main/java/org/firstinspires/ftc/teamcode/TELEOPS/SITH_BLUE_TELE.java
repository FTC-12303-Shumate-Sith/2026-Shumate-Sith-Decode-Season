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
public class SITH_BLUE_TELE extends OpMode {

    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    //-------------------
    //--DECLARE SERVOS---
    //-------------------
    private Servo odometryLift = null;
    private Servo headLight = null;
    private Servo rgbLight = null;

    //-------------------
    //--DECLARE MOTORS---
    //-------------------
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;

    //-----------------------
    //--DECLARE SUBSYSTEMS---
    //-----------------------
    private org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem shooterSub;

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
    private static final double ROTATIONAL_kP = -0.011;
    private static final double ROTATIONAL_kI = -0.003;
    private double integralSum = 0;

    //Driving
    private boolean slowMode = false;
    private boolean driveMode = false;
    private boolean isEndGame;
    private double slowModeMultiplier = 0.25;
    private boolean odometryState = true;
    private boolean ballInTransfer = false;

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

        //-------------------
        //----INIT MOTORS----
        //-------------------
        shooterSub = new org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem(hardwareMap);
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        transfer = hardwareMap.get(DcMotorSimple.class,"transfer");

        //--------------------
        //--INIT AUXILIARIES--
        //--------------------
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        stopSensor = hardwareMap.get(DigitalChannel.class, "stop");
        stopSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        limelight.start();
        telemetry.update();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addLine("GOOD LUCK SITH!");
    }

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        if ((getRuntime() >= 99) && !isEndGame) {
            gamepad1.rumbleBlips(5);
            isEndGame = true;
        }

        //Limelight results
        LLResult result = limelight.getLatestResult();
        double tx = result.isValid() ? result.getTx() : 0;
        double ty = result.isValid() ? result.getTy() : 0;

        // Update Subsystem Logic
        shooterSub.update(ty, result.isValid());

        //Aiming Trigger
        if (gamepad1.left_trigger > 0.50 || gamepad2.left_trigger > 0.50) {
            isAiming = true;
            if (result.isValid()) {
                integralSum += tx;
                if (integralSum > 10) integralSum = 10;
                if (integralSum < -10) integralSum = -10;
            }
        } else {
            isAiming = false;
            integralSum = 0;
        }

        //-------------------
        // --DRIVE CONTROLS--
        //-------------------
        if (gamepad1.y) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
        }

        slowMode = gamepad1.left_bumper;

        if (isAiming && result.isValid()) {
            double rotatePower = (tx * ROTATIONAL_kP) + (integralSum * ROTATIONAL_kI);
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, rotatePower, driveMode);
        } else {
            double mult = slowMode ? slowModeMultiplier : 1.0;
            follower.setTeleOpDrive(-gamepad1.left_stick_y * mult, -gamepad1.left_stick_x * mult, -gamepad1.right_stick_x * 0.5, driveMode);
        }

        //-------------------
        //----SUB SYSTEMS----
        //-------------------
        ballInTransfer = stopSensor.getState();

        // Shooter subsystem execution
        boolean shootPressed = gamepad2.right_trigger >= 0.5;
        shooterSub.run(shootPressed);
        boolean ready = shooterSub.isReady();

        //-----------------------------------
        // FIXED TRANSFER/INTAKE LOGIC
        //-----------------------------------
        if (shootPressed) {
            if (ready) {
                transfer.setPower(1.0);
                intake.setPower(1.0);
            } else {
                transfer.setPower(0.0);
                intake.setPower(0.0);
            }
        } else if (gamepad1.right_trigger >= 0.5 || gamepad2.a) {
            // Intake button is held: Intake ALWAYS spins
            intake.setPower(1.0);

            // Transfer ONLY spins if the ball isn't already there
            if (!ballInTransfer) {
                transfer.setPower(0.6);
            } else {
                transfer.setPower(0.0);
            }
        } else if (gamepad2.b) {
            transfer.setPower(-1.0);
            intake.setPower(-1.0);
        } else {
            // Idle state
            transfer.setPower(0.0);
            intake.setPower(0.0);
        }

        // Hood Control
        if (isAiming && result.isValid()) {
            shooterSub.setHood(shooterSub.getTargetHood());
        } else if (gamepad2.dpad_up) {
            shooterSub.setHood(0.6);
        } else if (gamepad2.dpad_down) {
            shooterSub.setHood(1.0);
        } else if (gamepad2.dpad_right) {
            shooterSub.setHood(0.8);
        } else if (gamepad2.dpad_left) {
            shooterSub.setHood(0.4);
        } else {
            shooterSub.setHood(1.0);
        }

        // Odometry Lift
        if (gamepad1.xWasPressed()) odometryState = !odometryState;
        odometryLift.setPosition(odometryState ? 0.7 : 0.0);

        // Telemetry
        telemetry.addData("Shooter Ready", ready);
        telemetry.addData("RPM", (int)shooterSub.getCurrentRPM());
        telemetry.addData("Target", (int)shooterSub.getTargetRPM());
        telemetry.addData("Ball Detected", ballInTransfer);
        telemetry.update();
    }
}