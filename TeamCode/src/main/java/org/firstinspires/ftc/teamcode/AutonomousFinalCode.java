package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutonomousFinal", group="Auton")
public class AutonomousFinalCode extends LinearOpMode {

    private DcMotor frontleft, frontright, rearleft, rearright;
    private DcMotor intake, intake2;
    private DcMotor launcher;
    private Servo blocker;

    static final double COUNTS_PER_MOTOR_REV = 28 * 20;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 2.95;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double TRACK_WIDTH_INCHES = 14.3125;

    static final double DIAGONAL_FACTOR = 1.41;
    static final double TURN_CORRECTION = 1.8;
    static final double INTAKE_POWER = 1.0;
    static final double BLOCKER_DOWN = 0.65;
    static final double BLOCKER_UP = 0.0;

    @Override
    public void runOpMode() {

        frontleft  = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        rearleft   = hardwareMap.get(DcMotor.class, "rearleft");
        rearright  = hardwareMap.get(DcMotor.class, "rearright");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        blocker  = hardwareMap.get(Servo.class, "blocker");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearright.setDirection(DcMotor.Direction.REVERSE);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //sleep(4000);
            move("turnright", 24, 0.25, false); // shoot preloaded balls
            sleep(300);
            shootBalls(1.2, 4000);

            move("turnleft", 24, 0.25, false);
            sleep(300);

            move("forward", 28, 0.5, false); // intake balls during move
            sleep(300);

            move("turnright", 90, 0.25, false);
            sleep(300);

            move("forward", 18, 0.5, true);
            sleep(300);

            move("backward", 18, 0.5, false);
            sleep(300);

            move("turnleft", 90, 0.25, false);
            sleep(300);

            move("forward", 36, 0.5, false);
            sleep(300);

            move("turnright", 40, 0.25, false);
            sleep(300);
            shootBalls(1.2, 4000);

            move("turnleft", 40, 0.25, false);
            sleep(300);

            move("backward", 12, 0.5, false);
            sleep(300);

            move("turnright", 90, 0.25, false);
            sleep(300);

            move("forward", 18, 0.5, true);
            sleep(300);

            move("backward", 18, 0.5, false);
            sleep(300);

            move("turnleft", 50, 0.25, false);
            sleep(300);
            shootBalls(1.2, 4000);
        }
    }

    // ===== MOVE FUNCTION =====
    private void move(String direction, double value, double power, boolean runIntake) {
        double flInches = 0, frInches = 0, rlInches = 0, rrInches = 0;

        switch (direction.toLowerCase()) {
            case "forward":
                flInches = frInches = rlInches = rrInches = value;
                break;
            case "backward":
                flInches = frInches = rlInches = rrInches = -value;
                break;
            case "straferight":
                flInches = rrInches = value * DIAGONAL_FACTOR;
                frInches = rlInches = -value * DIAGONAL_FACTOR;
                break;
            case "strafeleft":
                flInches = rrInches = -value * DIAGONAL_FACTOR;
                frInches = rlInches = value * DIAGONAL_FACTOR;
                break;
            case "turnright":
                double inchesPerDegreeR = (Math.PI * TRACK_WIDTH_INCHES) / 360.0;
                flInches = rlInches = value * TURN_CORRECTION * inchesPerDegreeR;
                frInches = rrInches = -value * TURN_CORRECTION * inchesPerDegreeR;
                break;
            case "turnleft":
                double inchesPerDegreeL = (Math.PI * TRACK_WIDTH_INCHES) / 360.0;
                flInches = rlInches = -value * TURN_CORRECTION * inchesPerDegreeL;
                frInches = rrInches = value * TURN_CORRECTION * inchesPerDegreeL;
                break;
            case "forwardright":
                flInches = rrInches = value * DIAGONAL_FACTOR;
                frInches = rlInches = 0;
                break;
            case "forwardleft":
                frInches = rlInches = value * DIAGONAL_FACTOR;
                flInches = rrInches = 0;
                break;
            case "backwardright":
                frInches = rlInches = -value * DIAGONAL_FACTOR;
                flInches = rrInches = 0;
                break;
            case "backwardleft":
                flInches = rrInches = -value * DIAGONAL_FACTOR;
                frInches = rlInches = 0;
                break;
            default:
                telemetry.addData("Error", "Unknown direction: " + direction);
                telemetry.update();
                return;
        }

        moveAllWheels(flInches, frInches, rlInches, rrInches, power, runIntake);
    }

    private void moveAllWheels(double flInches, double frInches, double rlInches, double rrInches, double power, boolean runIntake) {
        int flTarget = frontleft.getCurrentPosition() + (int)(flInches * COUNTS_PER_INCH);
        int frTarget = frontright.getCurrentPosition() + (int)(frInches * COUNTS_PER_INCH);
        int rlTarget = rearleft.getCurrentPosition() + (int)(rlInches * COUNTS_PER_INCH);
        int rrTarget = rearright.getCurrentPosition() + (int)(rrInches * COUNTS_PER_INCH);

        frontleft.setTargetPosition(flTarget);
        frontright.setTargetPosition(frTarget);
        rearleft.setTargetPosition(rlTarget);
        rearright.setTargetPosition(rrTarget);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(power);
        frontright.setPower(power);
        rearleft.setPower(power);
        rearright.setPower(power);

        if(runIntake){
            intake.setPower(INTAKE_POWER);
            intake2.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        while (opModeIsActive() &&
                (frontleft.isBusy() || frontright.isBusy() || rearleft.isBusy() || rearright.isBusy())) {
            telemetry.addData("FL", frontleft.getCurrentPosition());
            telemetry.addData("FR", frontright.getCurrentPosition());
            telemetry.addData("RL", rearleft.getCurrentPosition());
            telemetry.addData("RR", rearright.getCurrentPosition());
            telemetry.update();
        }

        frontleft.setPower(0);
        frontright.setPower(0);
        rearleft.setPower(0);
        rearright.setPower(0);

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void shootBalls(double power, long timeMs) {
        blocker.setPosition(BLOCKER_DOWN);
        sleep(200);

        launcher.setPower(power);
        intake.setPower(INTAKE_POWER);
        intake2.setPower(INTAKE_POWER);

        sleep(timeMs);

        launcher.setPower(0);
        intake.setPower(0);
        intake2.setPower(0);

        blocker.setPosition(BLOCKER_UP);
    }
}
