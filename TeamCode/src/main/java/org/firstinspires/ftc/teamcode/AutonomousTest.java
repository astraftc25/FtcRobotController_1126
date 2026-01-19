package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="BlueFar_FullLibrary_Geometry")
public class AutonomousTest extends LinearOpMode {

    // ===================== DRIVE HARDWARE =====================
    private DcMotorEx fl, fr, rl, rr;
    private IMU imu;

    // ===================== MECHANISMS =====================
    private DcMotorEx rotater, shooter;
    private DcMotor intake1, intake2, roller;
    private Limelight3A limelight;

    // ===================== CONSTANTS =====================
    private final double WHEEL_DIAMETER_IN = 2.97;
    private final double TICKS_PER_REV = 560;
    private final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    private final double TRACK_WIDTH_IN = 14.3125;

    // Turret constants
    private final double TURRET_TICKS_PER_REV = 28.0;
    private final double TURRET_KP = 0.025;
    private final double TURRET_KD = 0.002;

    private double goalFieldHeadingDeg360 = 0.0;
    private double lastTurretErrorDeg = 0.0;
    private double lastTurretTime = 0.0;

    @Override
    public void runOpMode() {
        // ===================== HARDWARE MAP =====================
        fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        fr = hardwareMap.get(DcMotorEx.class, "frontright");
        rl = hardwareMap.get(DcMotorEx.class, "rearleft");
        rr = hardwareMap.get(DcMotorEx.class, "rearright");

        rotater = hardwareMap.get(DcMotorEx.class, "rotater");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        roller = hardwareMap.get(DcMotor.class, "roller");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        rl.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        rotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.start();

        telemetry.addLine("Initialized - Full Library Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        shooter.setPower(1.0);
        lastTurretTime = getRuntime();

        // ===================== AUTON SEQUENCE EXAMPLE =====================
        // ===================== AUTON SEQUENCE =====================

// 1) Shoot preloaded balls from starting launch line
        shootBalls(2000);

// -------------------- First cluster --------------------
// 2) Curve toward first set of balls (far side cluster)
        curveForwardRightDeg(24, 90, 0.4, true);  // quarter-circle arc to align

// 3) Drive forward while intaking first cluster
        driveForward(24, 0.5, true);  // intake 1 tile
        driveForward(24, 0.5, true);  // intake next tile
        intakeOff();

// 4) Return to launch zone to shoot first cluster
        turnLeft(180, 0.4, 2.0);      // face launch zone
        driveForward(48, 0.5, false); // 2 tiles back
        shootBalls(1500);

// -------------------- Middle cluster --------------------
// 5) Move to middle cluster
        turnRight(90, 0.4, 2.0);       // face middle cluster
        strafeRight(24, 0.5, false);   // adjust laterally if needed
        driveForward(24, 0.5, true);   // intake middle cluster
        intakeOff();

// 6) Return to launch zone to shoot middle cluster
        turnLeft(180, 0.4, 2.0);
        driveForward(48, 0.5, false);
        shootBalls(1500);

// -------------------- Last cluster --------------------
// 7) Move to last cluster
        turnRight(90, 0.4, 2.0);      // face last cluster
        driveForward(24, 0.5, true);  // intake last cluster
        intakeOff();

// 8) Return to launch zone to shoot last cluster
        turnLeft(180, 0.4, 2.0);
        driveForward(48, 0.5, false);
        shootBalls(1500);

// -------------------- End --------------------
        stopDrive();

    }

    // ===================== CORE MOVE =====================
    private void moveRobot(int flT, int frT, int rlT, int rrT, double p, boolean intake, double timeoutS) {
        if (intake) intakeOn(); else intakeOff();

        setRunModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(flT);
        fr.setTargetPosition(frT);
        rl.setTargetPosition(rlT);
        rr.setTargetPosition(rrT);

        setRunModeAllDrive(DcMotor.RunMode.RUN_TO_POSITION);
        setMecanumPower(Math.abs(p), Math.abs(p), Math.abs(p), Math.abs(p));

        double start = getRuntime();
        while (opModeIsActive() && (fl.isBusy() || fr.isBusy()) && getRuntime() - start < timeoutS) {
            updateTurretAndShooter();
        }

        stopDrive();
        setRunModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ===================== BASIC MOVES =====================
    public void driveForward(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(t, t, t, t, p, i, 5.0);
    }

    public void driveBackward(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(-t, -t, -t, -t, p, i, 5.0);
    }

    public void strafeRight(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(t, -t, -t, t, p, i, 5.0);
    }

    public void strafeLeft(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(-t, t, t, -t, p, i, 5.0);
    }

    public void diagonalForwardRight(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(t, 0, 0, t, p, i, 5.0);
    }

    public void diagonalForwardLeft(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(0, t, t, 0, p, i, 5.0);
    }

    public void diagonalBackwardRight(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(0, -t, -t, 0, p, i, 5.0);
    }

    public void diagonalBackwardLeft(double in, double p, boolean i) {
        int t = (int) (in * TICKS_PER_INCH);
        moveRobot(-t, 0, 0, -t, p, i, 5.0);
    }

    // ===================== GEOMETRY-BASED CURVES =====================
    public void curveForwardRightDeg(double radiusIn, double angleDeg, double p, boolean i) {
        double theta = Math.toRadians(angleDeg);

        double rLeft  = radiusIn + TRACK_WIDTH_IN / 2.0;
        double rRight = radiusIn - TRACK_WIDTH_IN / 2.0;

        int leftTicks  = (int) (rLeft  * theta * TICKS_PER_INCH);
        int rightTicks = (int) (rRight * theta * TICKS_PER_INCH);

        moveRobot(leftTicks, rightTicks, leftTicks, rightTicks, p, i, 6.0);
    }

    public void curveForwardLeftDeg(double radiusIn, double angleDeg, double p, boolean i) {
        double theta = Math.toRadians(angleDeg);

        double rLeft  = radiusIn - TRACK_WIDTH_IN / 2.0;
        double rRight = radiusIn + TRACK_WIDTH_IN / 2.0;

        int leftTicks  = (int) (rLeft  * theta * TICKS_PER_INCH);
        int rightTicks = (int) (rRight * theta * TICKS_PER_INCH);

        moveRobot(leftTicks, rightTicks, leftTicks, rightTicks, p, i, 6.0);
    }

    // ===================== TURN =====================
    public void turnLeft(double deg, double maxP, double timeoutS) {
        double target = getHeadingDeg() + deg;
        double start = getRuntime();

        while (opModeIsActive()
                && Math.abs(shortestErrorDeg(target, getHeadingDeg())) > 2
                && getRuntime() - start < timeoutS) {

            updateTurretAndShooter();
            double error = shortestErrorDeg(target, getHeadingDeg());
            double pwr = Math.max(-maxP, Math.min(maxP, error * 0.01));
            setMecanumPower(-pwr, pwr, -pwr, pwr);
        }
        stopDrive();
    }

    public void turnRight(double deg, double maxP, double timeoutS) {
        turnLeft(-deg, maxP, timeoutS);
    }

    // ===================== MECHANISMS =====================
    private void updateTurretAndShooter() {
        double now = getRuntime();
        double robotH = wrap360(getHeadingDeg());

        LLResult res = limelight.getLatestResult();
        if (res != null && res.isValid()) {
            goalFieldHeadingDeg360 = wrap360(robotH + res.getTx());
        }

        double turretDeg = wrap360((rotater.getCurrentPosition() / TURRET_TICKS_PER_REV) * 360.0);
        double targetTurret = wrap360(goalFieldHeadingDeg360 - robotH);
        double error = shortestErrorDeg(targetTurret, turretDeg);

        double dt = Math.max(now - lastTurretTime, 0.001);
        double dErr = (error - lastTurretErrorDeg) / dt;
        double pwr = (TURRET_KP * error) + (TURRET_KD * dErr);

        rotater.setPower(Math.max(-0.6, Math.min(0.6, pwr)));

        lastTurretErrorDeg = error;
        lastTurretTime = now;
    }

    private void shootBalls(long timeMs) {
        intakeOn();
        roller.setPower(1.0);
        double start = getRuntime();
        while (opModeIsActive() && getRuntime() - start < timeMs / 1000.0) {
            updateTurretAndShooter();
        }
        intakeOff();
        roller.setPower(0);
    }

    // ===================== HELPERS =====================
    private void intakeOn() { intake1.setPower(1); intake2.setPower(1); }
    private void intakeOff() { intake1.setPower(0); intake2.setPower(0); }

    private void setMecanumPower(double flp, double frp, double rlp, double rrp) {
        fl.setPower(flp);
        fr.setPower(frp);
        rl.setPower(rlp);
        rr.setPower(rrp);
    }

    private void stopDrive() { setMecanumPower(0, 0, 0, 0); }

    private void setRunModeAllDrive(DcMotor.RunMode m) {
        fl.setMode(m);
        fr.setMode(m);
        rl.setMode(m);
        rr.setMode(m);
    }

    private double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double wrap360(double d) {
        d %= 360;
        if (d < 0) d += 360;
        return d;
    }

    private double shortestErrorDeg(double t, double c) {
        double e = t - c;
        while (e > 180) e -= 360;
        while (e < -180) e += 360;
        return e;
    }
}
