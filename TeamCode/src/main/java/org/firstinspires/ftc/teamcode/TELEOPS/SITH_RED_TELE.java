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
    private boolean slowMode = false;
    private double slowModeMultiplier;

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
    private DcMotorEx turretYaw = null;

    //-----------------------
    //--DECLARE AUXILIARIES--
    //-----------------------
    private Limelight3A limelight = null;
    private GoBildaPrismDriver prism = null;


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
        //intake = hardwareMap.get(DcMotorSimple.class, "intake");
        //turretYaw = hardwareMap.get(DcMotorEx.class, "turretYaw");
        //turretYaw.setDirection(DcMotorSimple.Direction.REVERSE);
        //turretYaw.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //--------------------
        //--INIT AUXILIARIES--
        //--------------------
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);


    }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Start
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   @Override
   public void start() {
        follower.startTeleopDrive();
       prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        //limelight.start();
  }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Loop
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   @Override
   public void loop() {

       //Call this once per loop
       follower.update();
       telemetryM.update();
       if (!automatedDrive) {
           //Make the last parameter false for field-centric
           //In case the drivers want to use a "slowMode" you can scale the vectors
           //This is the normal version to use in the TeleOp
           if (!slowMode) follower.setTeleOpDrive(
                   -gamepad1.left_stick_y,
                   -gamepad1.left_stick_x,
                   -gamepad1.right_stick_x * 0.5,
                   false // Robot Centric
           );
               //This is how it looks with slowMode on
           else follower.setTeleOpDrive(
                   -gamepad1.left_stick_y * slowModeMultiplier,
                   -gamepad1.left_stick_x * slowModeMultiplier,
                   -gamepad1.right_stick_x * slowModeMultiplier,
                   false // Robot Centric
           );
       }

       //Slow Mode
       gamepad1.right_bumper = slowMode;

       //Optional way to change slow mode strength
       if (gamepad1.xWasPressed()) {
           slowModeMultiplier += 0.25;
       }
       //Optional way to change slow mode strength
       if (gamepad2.yWasPressed()) {
           slowModeMultiplier -= 0.25;
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


       if (gamepad2.a) {
           odometryLift.setPosition(0.7);
       } else if (gamepad2.b) {
           odometryLift.setPosition(0.0);
       }

       if (gamepad2.y) {
           prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
       } else if (gamepad2.x) {
           prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
       }


       telemetryM.debug("position", follower.getPose());
       telemetryM.debug("velocity", follower.getVelocity());
       telemetryM.debug("automatedDrive", automatedDrive);

    }
}
