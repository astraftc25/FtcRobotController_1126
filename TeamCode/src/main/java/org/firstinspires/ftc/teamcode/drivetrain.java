package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Drivetrain")
public class drivetrain extends LinearOpMode {

    // ===================== DRIVE HARDWARE =====================
    private DcMotorEx fl, fr, rl, rr;
    private IMU imu;

    // ===================== CONSTANTS =====================
    private final double WHEEL_DIAMETER_IN = 2.97;
    private final double TICKS_PER_REV = 560;
    private final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);

    private final double TRACK_WIDTH_IN = 14.3125;

    @Override
    public void runOpMode() {

        // ===================== HARDWARE MAP =====================
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        rl = hardwareMap.get(DcMotorEx.class, "rearleft");
        rr = hardwareMap.get(DcMotorEx.class, "rearright");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        rl.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Initialized - Drivetrain Full Geometry");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ===================== AUTON SEQUENCE =====================
        driveForward(24, 0.5, 5.0);
        turnLeft(90, 0.4, 2.0);
        strafeLeft(24, 0.5, 5.0);
        curveForwardLeftDeg(24, 90, 0.4);
        turnRight(180, 0.4, 3.0);
        driveForward(48, 0.6, 6.0);

        stopDrive();
    }

    // ===================== CORE MOVE =====================
    private void moveRobot(
            int flT, int frT, int rlT, int rrT,
            double p, double timeoutS) {

        setRunModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(flT);
        fr.setTargetPosition(frT);
        rl.setTargetPosition(rlT);
        rr.setTargetPosition(rrT);

        setRunModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        setMecanumPower(p, p, p, p);

        double start = getRuntime();
        while (opModeIsActive()
                && (fl.isBusy() || fr.isBusy())
                && getRuntime() - start < timeoutS) {
            idle();
        }

        stopDrive();
        setRunModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===================== BASIC MOVES =====================
    public void driveForward(double in, double p, double timeoutS) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(t, t, t, t, p, timeoutS);
    }

    public void strafeLeft(double in, double p, double timeoutS) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(-t, t, t, -t, p, timeoutS);
    }

    // ===================== FIXED IMU TURNS =====================
    public void turnLeft(double deg, double maxP, double timeoutS) {
        turnToHeading(getHeadingDeg() + deg, maxP, timeoutS);
    }

    public void turnRight(double deg, double maxP, double timeoutS) {
        turnToHeading(getHeadingDeg() - deg, maxP, timeoutS);
    }

    private void turnToHeading(double targetDeg, double maxP, double timeoutS) {
        targetDeg = wrap180(targetDeg);
        double start = getRuntime();

        while (opModeIsActive()
                && getRuntime() - start < timeoutS) {

            double current = wrap180(getHeadingDeg());
            double error = shortestErrorDeg(targetDeg, current);

            if (Math.abs(error) < 1.5) break;

            double pwr = error * 0.01;
            pwr = Math.max(-maxP, Math.min(maxP, pwr));

            if (Math.abs(pwr) < 0.08)
                pwr = Math.signum(pwr) * 0.08;

            setMecanumPower(-pwr, pwr, -pwr, pwr);
        }

        stopDrive();
    }

    // ===================== CURVES =====================
    public void curveForwardLeftDeg(double radiusIn, double angleDeg, double p) {
        double theta = Math.toRadians(angleDeg);

        double rLeft  = radiusIn - TRACK_WIDTH_IN / 2.0;
        double rRight = radiusIn + TRACK_WIDTH_IN / 2.0;

        int leftTicks  = (int) (rLeft  * theta * TICKS_PER_INCH);
        int rightTicks = (int) (rRight * theta * TICKS_PER_INCH);

        moveRobot(leftTicks, rightTicks, leftTicks, rightTicks, p, 6.0);
    }

    // ===================== HELPERS =====================
    private void setMecanumPower(double flp, double frp, double rlp, double rrp) {
        fl.setPower(flp);
        fr.setPower(frp);
        rl.setPower(rlp);
        rr.setPower(rrp);
    }

    private void stopDrive() {
        setMecanumPower(0, 0, 0, 0);
    }

    private void setRunModeAllDrive(DcMotor.RunMode m) {
        fl.setMode(m);
        fr.setMode(m);
        rl.setMode(m);
        rr.setMode(m);
    }

    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double wrap180(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    private double shortestErrorDeg(double t, double c) {
        double e = t - c;
        while (e > 180) e -= 360;
        while (e < -180) e += 360;
        return e;
    }
}
