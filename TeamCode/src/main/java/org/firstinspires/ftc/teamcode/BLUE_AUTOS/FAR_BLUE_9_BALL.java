package org.firstinspires.ftc.teamcode.BLUE_AUTOS;

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

@Autonomous(name = "BLUE FAR 9 BALL", group = "BlueAutos")
public class FAR_BLUE_9_BALL extends OpMode {
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
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_pickUp1POS,
        drive_pickUp1POS_pickUp1EndPOS,
        drive_pickUp1EndPOS_goToShootPOS,
        drive_goToShootPosShootPosPath,
        drive_shootPOS_pickUp2POS,
        drive_pickUp2POS_pickUp2EndPOS,
        drive_pickUp2EndPOS_shootPOS,
        drive_shootPOS_EndPOS
    }
    PathState pathState;


    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(54, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(60,82, Math.toRadians(135));
    private final Pose pickUp1Pose = new Pose(45,36, Math.toRadians(180));
    private final Pose pickUp1EndPose = new Pose(10,36, Math.toRadians(180));
    private final Pose goToShootPose = new Pose(45,36, Math.toRadians(90));
    private final Pose pickUp2Pose = new Pose(45,60, Math.toRadians(180));
    private final Pose pickUp2EndPose = new Pose(12,60, Math.toRadians(180));
    private final Pose endPose = new Pose(60,30, Math.toRadians(180));


    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosPickUp1PosPath,
            drivePickUp1PosPickUp1EndPosPath,
            drivePickUp1EndPosGoToShootPosPath,
            driveGoToShootPosShootPosPath,
            driveShootPosPickUp2PosPath,
            drivePickUp2PosPickUp2EndPosPath,
            drivePickUp2EndPosShootPosPath,
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
        driveShootPosPickUp2PosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickUp2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickUp2Pose.getHeading())
                .build();
        drivePickUp2PosPickUp2EndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp2Pose, pickUp2EndPose))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(), pickUp2EndPose.getHeading())
                .build();
        drivePickUp2EndPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp2EndPose, shootPose))
                .setLinearHeadingInterpolation(pickUp2EndPose.getHeading(), shootPose.getHeading())
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

                    // Indexing logic
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        if (!ballInTransfer) transfer.setPower(0.6);
                        else transfer.setPower(0.0);
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
                    transfer.setPower(-1.0); // Intake mode

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
                        if (!ballInTransfer) transfer.setPower(0.6);
                        else transfer.setPower(0.0);
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
                        follower.followPath(driveShootPosPickUp2PosPath, true);
                        setPathState(PathState.drive_shootPOS_pickUp2POS);
                    }
                }
                break;
            case drive_shootPOS_pickUp2POS:
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    transfer.setPower(-1.0); // Intake mode

                    follower.setMaxPower(0.5);

                    follower.followPath(drivePickUp2PosPickUp2EndPosPath, true);
                    setPathState(PathState.drive_pickUp2POS_pickUp2EndPOS);
                }
                break;
            case drive_pickUp2POS_pickUp2EndPOS:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 2) {
                        follower.setMaxPower(1.0);
                        follower.followPath(drivePickUp2EndPosShootPosPath, true);
                        setPathState(PathState.drive_pickUp2EndPOS_shootPOS);
                    }
                }
                break;
            case drive_pickUp2EndPOS_shootPOS:
                if (!follower.isBusy()) {
                    shooterSub.run(true);

                    // Indexing Logic
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        if (!ballInTransfer) transfer.setPower(0.6);
                        else transfer.setPower(0.0);
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
                        intake.setPower(0.0);
                        transfer.setPower(0.0);
                        shooterSub.run(false);
                        follower.followPath(driveShootPosEndPosPath);
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
        shooterSub.setHood(1.0); // Set default hood position for auto
    }

    public void loop() {
        follower.update();
        ballInTransfer = stopSensor.getState(); // Sensor Logic
        shooterSub.update(0, false); // Heartbeat for Shooter PID
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