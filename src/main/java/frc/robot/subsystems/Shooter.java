package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UnpaidIntern;
import frc.robot.subsystems.WebServer;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
    private final TalonFX belt;
    private final TalonFX kick;
    private final TalonFX shooter;

    double currentBeltRPM;
    double currentKickRPM;
    double currentShooterRPM;

    double appliedBeltSpeed = 1;
    double appliedKickSpeed = 1;
    double appliedShooter= 1;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;
    private static final double BELT_DELAY_SEC = 0.5;

    private final Timer beltTimer = new Timer();
    private boolean beltStarted = false;
    private double currentSpeed = 0;

    public static PIDController shooterPID = new PIDController(1.4, 0.5, 0);

    double MAX_SPEED_RPM = Constants.Trajectory.MAX_MOTOR_SPEED_SHOOTER;

    public Shooter(int beltCANId, int kickCANId, int shooterCANId) {
        belt = new TalonFX(beltCANId, "canivore");
        kick = new TalonFX(kickCANId, "canivore");
        shooter = new TalonFX(shooterCANId, "canivore");
    }

    public void runRotationsAsController(double speed) {
        speed = (speed*MAX_SPEED_RPM)*Constants.Trajectory.errorCorrectionMultiplier;
        matchRotations(speed);
    }

    public void matchRotations(double targetRPM) {

        double beltOutput = 0.5*(targetRPM/MAX_SPEED_RPM);
        double kickOutput = 0.6*(targetRPM/MAX_SPEED_RPM);
        double shooterOutput = shooterPID.calculate(currentShooterRPM, targetRPM) / MAX_SPEED_RPM;

        appliedBeltSpeed = -MathUtil.clamp(beltOutput, -1, 1);
        appliedKickSpeed = MathUtil.clamp(kickOutput, -1, 1);
        appliedShooter = MathUtil.clamp(shooterOutput, -1, 1);

        UnpaidIntern.setPercentWithDeadband(shooter, appliedShooter, DEADBAND);

        if (!beltTimer.isRunning()) {
            beltTimer.reset();
            beltTimer.start();
            beltStarted = false;
        }

        if (beltTimer.hasElapsed(BELT_DELAY_SEC)) {
            UnpaidIntern.setPercentWithDeadband(belt, appliedBeltSpeed, DEADBAND);
            UnpaidIntern.setPercentWithDeadband(kick, appliedKickSpeed, DEADBAND);
            beltStarted = true;
        }
    }

    @Override
    public void periodic() {

        currentBeltRPM = belt.getVelocity().getValueAsDouble() * 60;
        currentKickRPM = kick.getVelocity().getValueAsDouble() * 60;
        currentShooterRPM = shooter.getVelocity().getValueAsDouble() * 60;

        WebServer.putNumber("BeltSpeed", currentBeltRPM);
        WebServer.putNumber("KickSpeed", currentKickRPM);
        WebServer.putNumber("ShooterSpeed", currentShooterRPM);

        // if (beltStarted) {
        //     UnpaidIntern.setPercentWithDeadband(belt, appliedBeltSpeed, DEADBAND);
        //     UnpaidIntern.setPercentWithDeadband(kick, appliedKickSpeedL, DEADBAND);
        //     UnpaidIntern.setPercentWithDeadband(motorE, appliedKickSpeedR, DEADBAND);
        // }
    }

    public void stop() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.stop(belt);
        UnpaidIntern.stop(kick);
        UnpaidIntern.stop(shooter);
    }

    public void kill() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.killMotor(belt);
        UnpaidIntern.killMotor(kick);
        UnpaidIntern.killMotor(shooter);
    }
}