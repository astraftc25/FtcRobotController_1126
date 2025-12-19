package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="IMU_PID_Mecanum", group="Auton")
public class TestIMUAuton extends LinearOpMode {

    private TestBenchIMU imu;
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // PID constants
    private final double kP = 0.05;
    private final double basePower = 0.4;

    // State machine
    private enum AutoState { INIT, DRIVE_FORWARD, STRAFE_LEFT, DRIVE_DIAGONAL, TURN1, STOP }
    private AutoState currentState = AutoState.INIT;

    private int driveTargetTicks;
    private double turnTargetHeading;

    @Override
    public void runOpMode() {

        // ---------- 1️⃣ Initialize IMU and motors ----------
        imu = new TestBenchIMU();
        imu.init(hardwareMap);

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("IMU and Motors Initialized");
        telemetry.update();

        waitForStart();

        // ---------- 2️⃣ Main state machine loop ----------
        while (opModeIsActive()) {

            switch(currentState) {

                case INIT:
                    // Sequence: turn first, drive forward, strafe, diagonal
                    turnTargetHeading = imu.getHeading(AngleUnit.DEGREES) + 90; // first turn
                    driveTargetTicks = 500; // first forward drive
                    currentState = AutoState.TURN1;
                    break;

                case TURN1:
                    if (turnToAngleWithIMU(turnTargetHeading)) {
                        currentState = AutoState.DRIVE_FORWARD;
                    }
                    break;

                case DRIVE_FORWARD:
                    if (driveForwardMecanum(driveTargetTicks, imu.getHeading(AngleUnit.DEGREES))) {
                        driveTargetTicks = 400; // next strafe distance
                        currentState = AutoState.STRAFE_LEFT;
                    }
                    break;

                case STRAFE_LEFT:
                    if (strafeMecanum(-driveTargetTicks, imu.getHeading(AngleUnit.DEGREES))) {
                        driveTargetTicks = 600; // next diagonal distance
                        currentState = AutoState.DRIVE_DIAGONAL;
                    }
                    break;

                case DRIVE_DIAGONAL:
                    if (driveDiagonalMecanum(driveTargetTicks, imu.getHeading(AngleUnit.DEGREES))) {
                        currentState = AutoState.STOP;
                    }
                    break;

                case STOP:
                    mecanumDrive(0,0,0);
                    return;
            }

            telemetry.addData("State", currentState);
            telemetry.update();
        }
    }

    // ---------- Mecanum drive ----------
    private void mecanumDrive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // Normalize if any power > 1
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }

    // ---------- IMU PID heading correction ----------
    private double getHeadingCorrection(double targetHeading) {
        double currentHeading = imu.getHeading(AngleUnit.DEGREES);
        double error = targetHeading - currentHeading;
        return error * kP;
    }

    // ---------- Drive forward ----------
    private boolean driveForwardMecanum(int targetTicks, double targetHeading) {
        int startPos = leftFront.getCurrentPosition();
        while (Math.abs(leftFront.getCurrentPosition() - startPos) < targetTicks && opModeIsActive()) {
            double correction = getHeadingCorrection(targetHeading);
            mecanumDrive(basePower, 0, correction);
            telemetry.addData("Drive Forward Heading", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        mecanumDrive(0,0,0);
        return true;
    }

    // ---------- Strafe ----------
    private boolean strafeMecanum(int targetTicks, double targetHeading) {
        int startPos = leftFront.getCurrentPosition();
        while (Math.abs(leftFront.getCurrentPosition() - startPos) < Math.abs(targetTicks) && opModeIsActive()) {
            double correction = getHeadingCorrection(targetHeading);
            double strafePower = (targetTicks > 0) ? basePower : -basePower;
            mecanumDrive(0, strafePower, correction);
            telemetry.addData("Strafe Heading", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        mecanumDrive(0,0,0);
        return true;
    }

    // ---------- Drive diagonal ----------
    private boolean driveDiagonalMecanum(int targetTicks, double targetHeading) {
        int startPos = leftFront.getCurrentPosition();
        while (Math.abs(leftFront.getCurrentPosition() - startPos) < targetTicks && opModeIsActive()) {
            double correction = getHeadingCorrection(targetHeading);
            // Example: forward + strafe (diagonal)
            mecanumDrive(basePower/1.4, basePower/1.4, correction);
            telemetry.addData("Diagonal Heading", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        mecanumDrive(0,0,0);
        return true;
    }

    // ---------- Turn to angle ----------
    private boolean turnToAngleWithIMU(double targetHeading) {
        final double tolerance = 2.0; // degrees
        double error = targetHeading - imu.getHeading(AngleUnit.DEGREES);
        while (Math.abs(error) > tolerance && opModeIsActive()) {
            error = targetHeading - imu.getHeading(AngleUnit.DEGREES);
            double correction = error * kP;
            mecanumDrive(0,0,correction);
            telemetry.addData("Turn Heading", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        mecanumDrive(0,0,0);
        return true;
    }
}
