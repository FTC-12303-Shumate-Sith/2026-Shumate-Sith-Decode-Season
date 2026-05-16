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

@Autonomous(name = "RED CLOSE 3 BALL", group = "RedAutos")
public class CLOSE_RED_3_BALL extends OpMode {
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
    private final Pose startPose = new Pose(120,128, Math.toRadians(37));
    private final Pose shootPose = new Pose(111,122, Math.toRadians(37));
    private final Pose endPose = new Pose(96,138, Math.toRadians(180));


    //------------ Declare Paths ---------------
    private PathChain driveStartPosShootPosPath,
            driveShootPosEndPosPath;


    //------------- Build Paths ---------------
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
                    // 1. Spin up shooter
                    shooterSub.run(true);

                    // 2. Indexing Logic: Wait for RPM
                    if (!shooterSub.isReady()) {
                        intake.setPower(1.0); // Intake stays spinning
                        if (!ballInTransfer) {
                            transfer.setPower(0.6); // Move ball to sensor
                        } else {
                            transfer.setPower(0.0); // Stop at sensor
                        }
                    }
                    // 3. Firing Logic: Shooter is ready
                    else {
                        transfer.setPower(1.0);
                        intake.setPower(1.0);

                        if (!shotsTriggered) {
                            pathTimer.resetTimer();
                            shotsTriggered = true;
                        }
                    }

                    // 4. After feeding for 1.2 seconds, move to next path
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


    @Override
    public void init() {
        pathState = PathState.drive_startPOS_shootPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // Hardware Init
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

        // Ensure hood is set for preload shots
        shooterSub.setHood(1.0);
    }


    @Override
    public void loop() {
        follower.update();

        // Read the sensor every loop
        ballInTransfer = stopSensor.getState();

        // Shooter Heartbeat for PID
        shooterSub.update(0, false);

        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Ball at Sensor", ballInTransfer);
        telemetry.addData("Shooter Ready", shooterSub.isReady());
        telemetry.addData("RPM", (int)shooterSub.getCurrentRPM());
        telemetry.update();
    }
}