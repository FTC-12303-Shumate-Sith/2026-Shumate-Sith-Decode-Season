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

@Autonomous(name = "BLUE CLOSE 3 BALL", group = "BlueAutos")
public class CLOSE_BLUE_3_BALL extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ------------ Hardware ---------------
    private Servo   encoderLift = null;
    private Servo   headLight = null;
    private Servo   rgbLight = null;
    private DigitalChannel stopSensor = null; // Added sensor
    GoBildaPrismDriver prism;
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;

    // ------------ Shooter ----------------
    private ShooterSubsystem shooterSub;
    private boolean shotsTriggered = false;
    private boolean ballInTransfer = false; // Added variable

    // ----- Autonomous State Machine ------
    public enum PathState   {
        drive_startPOS_shootPOS,
        shoot_preload,
        drive_shootPOS_EndPOS
    }
    PathState pathState;

    //------------ Declare Poses ---------------
    private final Pose startPose = new Pose(24,128, Math.toRadians(143));
    private final Pose shootPose = new Pose(33,120, Math.toRadians(143));
    private final Pose endPose = new Pose(47,138, Math.toRadians(180));

    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath, driveShootPosEndPosPath;

    public void buildPaths() {
        driveStartPosShootPosPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
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
                    // 1. Always start spinning the shooter
                    shooterSub.run(true);

                    // 2. SENSOR LOGIC: Index the ball while waiting for RPM
                    if (!shooterSub.isReady()) {
                        // If shooter isn't ready yet, only run transfer if ball isn't at the sensor
                        if (!ballInTransfer) {
                            transfer.setPower(0.5); // Move ball to sensor
                        } else {
                            transfer.setPower(0.0); // Stop at sensor and wait
                        }
                        intake.setPower(1.0); // Intake can keep spinning as requested
                    }
                    // 3. FIRING LOGIC: Shooter is ready
                    else {
                        transfer.setPower(1.0); // Push ball into shooter
                        intake.setPower(1.0);

                        if (!shotsTriggered) {
                            pathTimer.resetTimer();
                            shotsTriggered = true;
                        }
                    }

                    // 4. After feeding for 1.2s (clearing the sensor), move on
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
                    telemetry.addLine("All Done");
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

        // Initialize Stop Sensor
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

        // Read the sensor every loop
        ballInTransfer = stopSensor.getState();

        statePathUpdate();
        shooterSub.update(0, false);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Ball at Sensor", ballInTransfer);
        telemetry.addData("Shooter Ready", shooterSub.isReady());
        telemetry.addData("RPM", (int)shooterSub.getCurrentRPM());
        telemetry.update();
    }
}