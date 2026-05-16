package org.firstinspires.ftc.teamcode.RED_AUTOS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel; // Added
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "RED FAR 6 BALL", group = "RedAutos")
public class FAR_RED_6_BALL extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    // ------------ Hardware ---------------
    private Servo   encoderLift = null;
    private Servo   headLight = null;
    private Servo   rgbLight = null;
    private DigitalChannel stopSensor = null; // Added

    GoBildaPrismDriver prism;
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;


    // ------------ Shooter ----------------
    private ShooterSubsystem shooterSub;
    private boolean shotsTriggered = false;
    private boolean ballInTransfer = false; // Added


    // Autonomous State Machine
    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVING AROUND FIELD
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_pickUp1POS,
        drive_pickUp1POS_pickUp1EndPOS,
        drive_pickUp1EndPOS_goToShootPOS,
        drive_goToShootPosShootPosPath,
        drive_shootPOS_EndPOS
    }
    PathState pathState;


    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(90, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(72,72, Math.toRadians(46));
    private final Pose pickUp1Pose = new Pose(98,36, Math.toRadians(0));
    private final Pose pickUp1EndPose = new Pose(135,36, Math.toRadians(0));
    private final Pose goToShootPose = new Pose(98,36, Math.toRadians(111));
    private final Pose endPose = new Pose(90,30, Math.toRadians(0));


    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosPickUp1PosPath,
            drivePickUp1PosPickUp1EndPosPath,
            drivePickUp1EndPosGoToShootPosPath,
            driveGoToShootPosShootPosPath,
            driveShootPosEndPosPath;


    //------------- Build Paths ---------------
    public void buildPaths() {
        driveStartPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosPickUp1PosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickUp1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickUp1Pose.getHeading())
                .build();
        drivePickUp1PosPickUp1EndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1Pose, pickUp1EndPose))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), pickUp1EndPose.getHeading())
                .build();
        drivePickUp1EndPosGoToShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1EndPose, goToShootPose))
                .setLinearHeadingInterpolation(pickUp1EndPose.getHeading(), goToShootPose.getHeading())
                .build();
        driveGoToShootPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(goToShootPose, shootPose))
                .setLinearHeadingInterpolation(goToShootPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }


    //------------- State Machine ---------------
    public void statePathUpdate() {
        switch (pathState) {
            case drive_startPOS_shootPOS:
                follower.followPath(driveStartPosShootPosPath, true);
                setPathState(PathState.shoot_preload);
                break;
            case shoot_preload:
                if (!follower.isBusy()) {
                    shooterSub.run(true);

                    // Indexing Logic: Wait for RPM
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        if (!ballInTransfer) {
                            transfer.setPower(0.6); // Move ball to sensor
                        } else {
                            transfer.setPower(0.0); // Wait at sensor
                        }
                    } else {
                        // Firing Logic
                        transfer.setPower(1.0);
                        intake.setPower(1.0);

                        if (!shotsTriggered) {
                            pathTimer.resetTimer();
                            shotsTriggered = true;
                        }
                    }

                    if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 1.2) {
                        transfer.setPower(0);
                        intake.setPower(0);
                        shooterSub.run(false);
                        follower.followPath(driveShootPosPickUp1PosPath, true);
                        setPathState(PathState.drive_shootPOS_pickUp1POS);
                    }
                }
                break;
            case drive_shootPOS_pickUp1POS:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    transfer.setPower(-1.0); // Intake mode for transfer

                    follower.setMaxPower(0.5);

                    follower.followPath(drivePickUp1PosPickUp1EndPosPath, true);
                    setPathState(PathState.drive_pickUp1POS_pickUp1EndPOS);
                }
                break;
            case drive_pickUp1POS_pickUp1EndPOS:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 2) {
                        follower.setMaxPower(1.0);
                        follower.followPath(drivePickUp1EndPosGoToShootPosPath, true);
                        setPathState(PathState.drive_pickUp1EndPOS_goToShootPOS);
                    }
                }
                break;
            case drive_pickUp1EndPOS_goToShootPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveGoToShootPosShootPosPath, true);
                    setPathState(PathState.drive_goToShootPosShootPosPath);
                }
                break;
            case drive_goToShootPosShootPosPath:
                if (!follower.isBusy()) {
                    shooterSub.run(true);

                    // Indexing Logic
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        if (!ballInTransfer) {
                            transfer.setPower(0.6);
                        } else {
                            transfer.setPower(0.0);
                        }
                    } else {
                        // Firing Logic
                        transfer.setPower(1.0);
                        intake.setPower(1.0);

                        if (!shotsTriggered) {
                            pathTimer.resetTimer();
                            shotsTriggered = true;
                        }
                    }

                    if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 1.2) {
                        transfer.setPower(0);
                        intake.setPower(0);
                        shooterSub.run(false);
                        follower.followPath(driveShootPosEndPosPath, true);
                        setPathState(PathState.drive_shootPOS_EndPOS);
                    }
                }
                break;
            case drive_shootPOS_EndPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("ALl Done");
                }
                break;
            default:
                telemetry.addLine("No state Commanded");
                break;
        }
    }


    //------- State Machine Helper Functions -------
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }


    public void init() {
        pathState = PathState.drive_startPOS_shootPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // --- NEW ROBOT INIT ---
        shooterSub = new ShooterSubsystem(hardwareMap);
        intake = hardwareMap.get(DcMotorSimple.class,"Intake");
        transfer = hardwareMap.get(DcMotorSimple.class,"Transfer");

        // Sensor Init
        stopSensor = hardwareMap.get(DigitalChannel.class, "stop");
        stopSensor.setMode(DigitalChannel.Mode.INPUT);

        headLight = hardwareMap.get(Servo.class,"Headlight");
        rgbLight = hardwareMap.get(Servo.class,"RGBLight");
        buildPaths();
        follower.setPose(startPose);
        encoderLift = hardwareMap.get(Servo.class,"Odometry");
        encoderLift.setPosition(0.0);
        headLight.setPosition(0.35);
        rgbLight.setPosition(0.47);
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        // Ensure hood is up for autonomous
        shooterSub.setHood(1.0);
    }

    public void loop() {
        follower.update();
        ballInTransfer = stopSensor.getState(); // Read sensor state
        shooterSub.update(0, false); // Keep PID logic running
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Ball at Sensor", ballInTransfer);
        telemetry.addData("Shooter Ready", shooterSub.isReady());
        telemetry.addData("RPM", (int)shooterSub.getCurrentRPM());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time", opModeTimer.getElapsedTime());
        telemetry.update();
    }
}