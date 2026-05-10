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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private boolean automatedDrive;
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
    private DcMotorSimple shooter = null;
    private DcMotorSimple twoShooter = null;
    private DcMotorSimple intake = null;
    private DcMotorSimple transfer = null;

    //-----------------------
    //--DECLARE AUXILIARIES--
    //-----------------------
    private Limelight3A limelight = null;
    private GoBildaPrismDriver prism = null;

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

        //hoodPitch = hardwareMap.get(Servo.class, "hoodPitch");
        //hoodPitch.setPosition(0.0);

        //-------------------
        //----INIT MOTORS----
        //-------------------
        shooter = hardwareMap.get(DcMotorSimple.class, "shooter");
        twoShooter = hardwareMap.get(DcMotorSimple.class, "2shooter");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        transfer = hardwareMap.get(DcMotorSimple.class,"transfer");


        //--------------------
        //--INIT AUXILIARIES--
        //--------------------
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
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
        if ((getRuntime() >= 99) && !isEndGame){
            gamepad1.rumbleBlips(5);
            isEndGame = true;
        }

        //Limelight results
        LLResult result = limelight.getLatestResult();
        double tx = result.isValid() ? result.getTx() : 0;

        //Enable auto aiming
        if (gamepad1.left_trigger > 0.50) {
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

        telemetry.addData("isAiming", isAiming);
        telemetry.addData("tx", tx);
        telemetry.addData("integralSum", integralSum); // Track the build up
        telemetry.addData("Slow Mode", slowMode);
        telemetry.addData("Valid Result", result.isValid());


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
        }

        else if (!slowMode) follower.setTeleOpDrive(
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

        //intake
        if ((gamepad1.right_trigger >= 0.5) || (gamepad2.a)) {
            intake.setPower(1.0);
        } else if (gamepad2.b) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        //Transfer
        if (gamepad2.x) {
            transfer.setPower(1.0);
        } else if (gamepad2.y) {
            transfer.setPower(-1.0);
        } else {
            transfer.setPower(0.0);
        }

        //Shooter
        if (gamepad2.right_trigger >= 0.5) {
            shooter.setPower(1.0);
            twoShooter.setPower(-1.0);
        } else if (gamepad2.left_trigger >= 0.5 ) {
            shooter.setPower(-1.0);
            twoShooter.setPower(1.0);
        } else {
            shooter.setPower(0.0);
            twoShooter.setPower(0.0);
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


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

    }
}
