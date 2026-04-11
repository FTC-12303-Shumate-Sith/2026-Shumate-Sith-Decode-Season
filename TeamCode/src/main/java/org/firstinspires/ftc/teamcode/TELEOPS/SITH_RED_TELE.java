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

    //-------------------
    //--DECLARE MOTORS---
    //-------------------
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
        odometryLift = hardwareMap.get(Servo.class, "Servo");
        odometryLift.setPosition(0.0);

       // hoodPitch = hardwareMap.get(Servo.class, "hoodPitch");
        //hoodPitch.setPosition(0.0);

        //-------------------
        //----INIT MOTORS----
        //-------------------
       // intake = hardwareMap.get(DcMotorSimple.class, "intake");

       // turretYaw = hardwareMap.get(DcMotorEx.class, "turretYaw");
       // turretYaw.setDirection(DcMotorSimple.Direction.REVERSE);
       // turretYaw.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
       // turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //--------------------
        //--INIT AUXILIARIES--
        //--------------------
      //  limelight = hardwareMap.get(Limelight3A.class, "limelight");
    //    limelight.pipelineSwitch(0);


        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");






    }

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Start
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   @Override
   public void start() {
        follower.startTeleopDrive();

     //   limelight.start();

        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

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
                   -gamepad1.right_stick_x,
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
       //Automated PathFollowing
       if (gamepad1.aWasPressed()) {
           follower.followPath(pathChain.get());
           automatedDrive = true;
       }
       //Stop automated following if the follower is done
       if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
           follower.startTeleopDrive();
           automatedDrive = false;
       }
       //Slow Mode
       if (gamepad1.rightBumperWasPressed()) {
           slowMode = !slowMode;
       }
       //Optional way to change slow mode strength
       if (gamepad1.xWasPressed()) {
           slowModeMultiplier += 0.25;
       }
       //Optional way to change slow mode strength
       if (gamepad2.yWasPressed()) {
           slowModeMultiplier -= 0.25;
       }


       if (gamepad2.a) {
           odometryLift.setPosition(0.7);
       }

       if (gamepad2.b) {
           odometryLift.setPosition(0.0);
       }


       if (gamepad2.y) {
           prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
       }

       if (gamepad2.x) {
           prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
       }


       telemetryM.debug("position", follower.getPose());
       telemetryM.debug("velocity", follower.getVelocity());
       telemetryM.debug("automatedDrive", automatedDrive);
    }
}
