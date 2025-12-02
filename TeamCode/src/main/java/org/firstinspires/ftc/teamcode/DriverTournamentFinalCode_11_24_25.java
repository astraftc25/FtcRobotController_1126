package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import android.util.Size;

@TeleOp(name = "DriverTournamentFinalCode_11_24_25 (Blocks to Java)")
public class DriverTournamentFinalCode_11_24_25 extends LinearOpMode {
    private DcMotor intake;
    private DcMotor launcher;
    private DcMotor intake2;
    private DcMotor rearleft;
    private DcMotor frontright;
    private DcMotor rearright;
    private DcMotor frontleft;
    private Servo   blocker;

    private AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private VisionPortal.Builder myVisionPortalBuilder;
    private AprilTagLibrary myAprilTagLibrary;
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    //Distance, Angles
    private boolean goalVisible;
    private double  range;
    private double  bearing;
    private double  elevation;

    private VoltageSensor myControlHubVoltageSensor;

    @Override
    public void runOpMode() {
        float forward_back;
        float strafe;
        float rotate;
        float frontLeftPower;
        float backLeftPower;
        float frontRightPower;
        float backRightPower;
        float max;


        //Initialize all sensors and actuators
        initMyHardwareSetup();

        //Initialize Camera
        initCamera();

        //To make sure that telemetry information is not cleared after each telemetry.update() call
        //telemetry.setAutoClear(false);
        //telemetry.setMsTransmissionInterval(5000); //5 sec

        //Wait for start button press on Driver Hub
        waitForStart();

        //Once OpMode is active run the loop
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("Voltage", myControlHubVoltageSensor.getVoltage());
                motifCode();

                // Put loop blocks here.
                forward_back = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;

                frontLeftPower  = (forward_back + strafe + rotate);
                frontRightPower = (forward_back - strafe - rotate);
                backLeftPower   = forward_back - strafe + rotate;
                backRightPower  = (forward_back + strafe - rotate);

                max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                max = Math.max(max, Math.abs(backRightPower));
                max = Math.max(max, Math.abs(backLeftPower));

                if (max > 1) {
                    frontLeftPower = (frontLeftPower / max);
                    frontRightPower =  (frontRightPower / max);
                    backLeftPower = (backLeftPower / max);
                    backRightPower = (backRightPower / max);
                }

                frontleft.setPower(frontLeftPower);
                frontright.setPower(frontRightPower);
                rearleft.setPower(backLeftPower);
                rearright.setPower(backRightPower);

                if (gamepad2.rightBumperWasPressed()) {
                    intake.setPower(1);
                }
                if (gamepad2.rightBumperWasReleased()) {
                    intake.setPower(0);
                }
                if (gamepad2.leftBumperWasPressed()) {
                    intake2.setPower(.7);
                }
                if (gamepad2.leftBumperWasReleased()) {
                    intake2.setPower(0);
                }
                if (gamepad2.dpadLeftWasPressed()) {
                    intake.setPower(-1);
                    intake2.setPower(-1);
                }
                if (gamepad2.dpadLeftWasReleased()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                }

                if(gamepad2.xWasPressed()) {
                    intake2.setPower(0);
                    blocker.setPosition(0.65); // Location to make blocker go down
                    sleep(500);
                    intake2.setPower(1);
                }
                if(gamepad2.xWasReleased())  {
                    blocker.setPosition(0); // Default location of the blocker blocks
                    intake2.setPower(0);
                }
                if (gamepad2.bWasPressed()) {
                    ((DcMotorEx) launcher).setVelocity(1200);
                }

                //Check if there is a Motif detected, decode it and display on driver hub
                motifCode();

                //Display RedAprilTag Locations
                findLocationFromGoal();

                if (gamepad2.aWasPressed()) {
                    ((DcMotorEx) launcher).setVelocity(1150);
                    //Empirical Formula based on experimentation
                    // velocity = 6.3112 * range + 772.64
/*
                    if(goalVisible) {
                        ((DcMotorEx) launcher).setVelocity(6.3112 * range + 772.64);
                    }
 */
                }
                if (gamepad2.dpadDownWasPressed()) {
                    launcher.setPower(0);
                }

                telemetry.update(); //No needed with telemetry update timer
            }
        }
    }

    public void initMyHardwareSetup() {
        //Initialize the variables
        goalVisible = false;
        range = 0;
        bearing = 0;
        elevation = 0;

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        rearright = hardwareMap.get(DcMotor.class, "rearright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");

        blocker = hardwareMap.get(Servo.class, "blocker");

        launcher = hardwareMap.get(DcMotor.class, "launcher");

        // Put initialization blocks here.
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);

        rearleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearright.setDirection(DcMotor.Direction.REVERSE);

        blocker.setPosition(0);

        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initCamera() {
        // Camera Initialization START
        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Get the AprilTagLibrary for the current season.
        myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        // Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        // Set whether or not to draw the tag ID on detections.
        myAprilTagProcessorBuilder.setDrawTagID(true);
        // Set whether or not to draw the tag outline on detections.
        myAprilTagProcessorBuilder.setDrawTagOutline(true);
        // Set whether or not to draw the cube projection on detections.
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);
        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Create Vision Portal Builder
        // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
        myVisionPortalBuilder = new VisionPortal.Builder();
        // Set the camera to the specified webcam name.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "eyes"));
        // Add the AprilTag processor.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        // Enable the live camera preview.
        myVisionPortalBuilder.enableLiveView(true);
        // Build the VisionPortal object and assign it to a variable.
        myVisionPortal = myVisionPortalBuilder.build();
        // Set the detector decimation.
        myAprilTagProcessor.setDecimation(2);
        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);
        //Camera initialization blocks: End
    }
    public void motifCode()
    {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;
        int myAprilTagIdCode;

        myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            if (myAprilTagDetection.metadata != null) {
                myAprilTagIdCode = myAprilTagDetection.id;

                if (myAprilTagIdCode == 21) {
                    telemetry.addLine("Green Purple Purple");
                    telemetry.addLine("");
                }
                else if (myAprilTagIdCode == 22) {
                    telemetry.addLine("Purple Green Purple");
                    telemetry.addLine("");
                }
                else if (myAprilTagIdCode == 23) {
                    telemetry.addLine("Purple Purple Green");
                    telemetry.addLine("");
                }
                else {
                    telemetry.addLine("No Motif Detected");
                    telemetry.addLine("");
                }
            }
        }
    }

    public void findLocationFromGoal()
    {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;
        int myAprilTagIdCode;

        myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            if (myAprilTagDetection.metadata != null) {
                myAprilTagIdCode = myAprilTagDetection.id;

                if (myAprilTagIdCode == 20)  {
                    range =  myAprilTagDetection.ftcPose.range;
                    bearing = myAprilTagDetection.ftcPose.bearing;
                    elevation = myAprilTagDetection.ftcPose.elevation;
                    goalVisible = true;

                    telemetry.addData("Blue Goal: Distance ", range);
                    telemetry.addLine("");
                    telemetry.addData("Blue Goal Bearing ", bearing);
                    telemetry.addLine("");
                    telemetry.addData("Blue Goal Elevation ", elevation);
                    telemetry.addLine("");
                }
                if (myAprilTagIdCode == 24)  {
                    range =  myAprilTagDetection.ftcPose.range;
                    bearing = myAprilTagDetection.ftcPose.bearing;
                    elevation = myAprilTagDetection.ftcPose.elevation;
                    goalVisible = true;

                    telemetry.addData("Red Goal Distance ", range);
                    telemetry.addLine("");
                    telemetry.addData("Red Goal Bearing ", bearing);
                    telemetry.addLine("");
                    telemetry.addData("Red Goal Elevation ", elevation);
                    telemetry.addLine("");
                }
                else { //Need to rotate the bot
                    goalVisible = false;
                    telemetry.addLine("Goal Not Visible");
                    telemetry.addLine("");
                }
            }
            else {
                //Need to rotate the bot
                goalVisible = false;
                telemetry.addLine("Goal Not Visible");
                telemetry.addLine("");
            }
        }
    }
}