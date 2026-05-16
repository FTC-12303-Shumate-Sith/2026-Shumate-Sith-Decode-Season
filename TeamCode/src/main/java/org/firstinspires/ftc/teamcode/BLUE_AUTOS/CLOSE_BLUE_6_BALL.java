package org.firstinspires.ftc.teamcode.BLUE_AUTOS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "BLUE CLOSE 6 BALL", group = "BlueAutos")
public class CLOSE_BLUE_6_BALL extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ------------ Hardware ---------------
    private Servo   encoderLift = null;
    private Servo   headLight = null;
    private Servo   rgbLight = null;
    private DigitalChannel stopSensor = null;

    GoBildaPrismDriver prism;
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;

    // ------------ Shooter ----------------
    private ShooterSubsystem shooterSub;
    private boolean shotsTriggered = false;
    private boolean ballInTransfer = false;

    // ----- Autonomous State Machine ------
    public enum PathState   {
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_pickUp1POS,
        drive_pickUp1POS_pickUp1EndPOS,
        drive_pickUp1EndPOS_shootPOS,
        drive_shootPOS_EndPOS,
    }
    PathState pathState;

    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(24,128, Math.toRadians(143));
    private final Pose shootPose = new Pose(33,120, Math.toRadians(143));
    private final Pose pickUp1Pose = new Pose(45,85, Math.toRadians(180));
    private final Pose pickUp1EndPose = new Pose(19,85, Math.toRadians(180));
    private final Pose endPose = new Pose(22,104,Math.toRadians(180));

    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosPickUp1PosPath,
            drivePickUp1PosPickUp1EndPosPath,
            drivePickUp1EndPosShootPosPath,
            driveShootPosEndPosPath;

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
        drivePickUp1EndPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(pickUp1EndPose, shootPose))
                .setLinearHeadingInterpolation(pickUp1EndPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPosPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case drive_startPOS_shootPOS:
                follower.followPath(driveStartPosShootPosPath, true);
                setPathState(PathState.shoot_preload);
                break;

            case shoot_preload:
                if (!follower.isBusy()) {
                    shooterSub.run(true);

                    // Indexing: Stop transfer if full while waiting for RPM
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        transfer.setPower(ballInTransfer ? 0.0 : 0.6);
                    } else {
                        // Firing: Push regardless of sensor
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
                    // Start intaking, but respect the sensor immediately
                    intake.setPower(1.0);
                    transfer.setPower(ballInTransfer ? 0.0 : 0.6);

                    follower.setMaxPower(0.5);
                    follower.followPath(drivePickUp1PosPickUp1EndPosPath, true);
                    setPathState(PathState.drive_pickUp1POS_pickUp1EndPOS);
                }
                break;

            case drive_pickUp1POS_pickUp1EndPOS:
                // IMPORTANT: Keep sensor logic active while the robot is driving over the ball
                intake.setPower(1.0);
                transfer.setPower(ballInTransfer ? 0.0 : 0.6);

                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 2) {
                        follower.setMaxPower(1.0);
                        follower.followPath(drivePickUp1EndPosShootPosPath, true);
                        setPathState(PathState.drive_pickUp1EndPOS_shootPOS);
                    }
                }
                break;

            case drive_pickUp1EndPOS_shootPOS:
                if (!follower.isBusy()) {
                    shooterSub.run(true);

                    // Indexing Logic
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0);
                        transfer.setPower(ballInTransfer ? 0.0 : 0.6);
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
                    telemetry.addLine("All Done");
                    intake.setPower(0);
                    transfer.setPower(0);
                }
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.drive_startPOS_shootPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        shooterSub = new ShooterSubsystem(hardwareMap);
        intake = hardwareMap.get(DcMotorSimple.class,"intake");
        transfer = hardwareMap.get(DcMotorSimple.class,"transfer");
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

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        shooterSub.setHood(1.0);
    }

    @Override
    public void loop() {
        follower.update();

        // Sensor returns TRUE when ball is detected
        ballInTransfer = stopSensor.getState();

        shooterSub.update(0, false);
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Ball Detected", ballInTransfer);
        telemetry.addData("Shooter Ready", shooterSub.isReady());
        telemetry.addData("RPM", (int)shooterSub.getCurrentRPM());
        telemetry.update();
    }
}