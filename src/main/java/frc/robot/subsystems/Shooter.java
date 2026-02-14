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
    private final TalonFX motorA;
    private final TalonFX motorB;
    private final TalonFX motorC;

    double appliedBeltSpeed = 1;
    double appliedShooterBSpeed = 1;
    double appliedShooterCSpeed = 1;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;
    private static final double BELT_DELAY_SEC = 1.0;

    private final Timer beltTimer = new Timer();
    private boolean beltStarted = false;
    private double currentSpeed = 0;

    // TUNE THIS FIRST (SMALLER FIRST) this is problably going to be wrong because units are in rpm now
    // TODO: TUNE PID FOR SHOOTER
    PIDController beltAPID = new PIDController(0.95, 0.2, 0);
    PIDController shooterBPID = new PIDController(0.95, 0.2, 0);
    PIDController shooterCPID = new PIDController(0.95, 0.2, 0);

    double MAX_SPEED_RPM = 6000;

    public Shooter(int shooterMotorBCANId, int shooterMotorCCANId, int beltMotorACANId) {
        motorA = new TalonFX(beltMotorACANId);
        motorB = new TalonFX(shooterMotorBCANId);
        motorC = new TalonFX(shooterMotorCCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
        UnpaidIntern.zeroEncoder(motorC);
    }

    private double clamp(double speed) {
        if (speed > MAX_SPEED) return MAX_SPEED;
        if (speed < MIN_SPEED) return MIN_SPEED;
        return speed;
    }

    public void set(double speed) {
        speed = clamp(speed);
        currentSpeed = speed;

        WebServer.putNumber("BeltSpeed", motorA.getVelocity().getValueAsDouble()*(180/Math.PI));
        WebServer.putNumber("ShooterBSpeed", motorB.getVelocity().getValueAsDouble()*(180/Math.PI));
        WebServer.putNumber("ShooterCSpeed", motorC.getVelocity().getValueAsDouble()*(180/Math.PI));

        UnpaidIntern.setPercentWithDeadband(motorB, speed*0.8, DEADBAND);
        UnpaidIntern.setPercentWithDeadband(motorC, -speed, DEADBAND);

        if (!beltTimer.isRunning()) {
            beltTimer.reset();
            beltTimer.start();
            beltStarted = false;
        }

        if (beltTimer.hasElapsed(BELT_DELAY_SEC)) {
            UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
            beltStarted = true;
        }
    }

    public void runRotationsAsController(double speed) {
        speed = (speed*MAX_SPEED_RPM)*Constants.Trajectory.errorCorrectionMultiplier;
        matchRotations(speed);
        System.out.println("RunRotationsAsControllerSpeed: "+speed);
    }

    public void matchRotations(double targetRPM) {

        double currentBeltRPM = motorA.getVelocity().getValueAsDouble() * 60;
        double currentBumperRPM = motorB.getVelocity().getValueAsDouble() * 60;
        double currentShooterCRPM = -motorC.getVelocity().getValueAsDouble() * 60;

        double beltTargetRPM = targetRPM * 0.2;

        double beltOutput = beltAPID.calculate(currentBeltRPM, beltTargetRPM) / MAX_SPEED_RPM * 0.2;
        double bOutput = shooterBPID.calculate(currentBumperRPM, targetRPM) / MAX_SPEED_RPM;
        double cOutput = shooterCPID.calculate(currentShooterCRPM, targetRPM) / MAX_SPEED_RPM;

        appliedBeltSpeed = MathUtil.clamp(beltOutput, -0.2, 0.2);
        appliedShooterBSpeed = MathUtil.clamp(bOutput, -1, 1);
        appliedShooterCSpeed = MathUtil.clamp(-cOutput, -1, 1);

        WebServer.putNumber("TargetSpeed", targetRPM);
        WebServer.putNumber("BeltSpeed", currentBeltRPM);
        WebServer.putNumber("ShooterBSpeed", currentBumperRPM);
        WebServer.putNumber("ShooterCSpeed", currentShooterCRPM);
        WebServer.putNumber("AppliedBeltSpeed", appliedBeltSpeed);
        WebServer.putNumber("AppliedShooterBSpeed", appliedShooterBSpeed);
        WebServer.putNumber("AppliedShooterCSpeed", appliedShooterCSpeed);

        UnpaidIntern.setPercentWithDeadband(motorB, appliedShooterBSpeed, DEADBAND);
        UnpaidIntern.setPercentWithDeadband(motorC, appliedShooterCSpeed, DEADBAND);

        if (!beltTimer.isRunning()) {
            beltTimer.reset();
            beltTimer.start();
            beltStarted = false;
        }

        if (beltTimer.hasElapsed(BELT_DELAY_SEC)) {
            UnpaidIntern.setPercentWithDeadband(motorA, appliedBeltSpeed, DEADBAND);
            beltStarted = true;
        }
    }

    @Override
    public void periodic() {
        if (beltStarted) {
            UnpaidIntern.setPercentWithDeadband(motorA, appliedBeltSpeed, DEADBAND);
        }
    }

    public void stop() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.stop(motorA);
        UnpaidIntern.stop(motorB);
        UnpaidIntern.stop(motorC);
        WebServer.putNumber("TargetSpeed", 0);
        WebServer.putNumber("BeltSpeed", 0);
        WebServer.putNumber("ShooterBSpeed", 0);
        WebServer.putNumber("ShooterCSpeed", 0);
        WebServer.putNumber("AppliedBeltSpeed", 0);
        WebServer.putNumber("AppliedShooterBSpeed", 0);
        WebServer.putNumber("AppliedShooterCSpeed", 0);
    }

    public void kill() {
        beltTimer.stop();
        beltTimer.reset();
        beltStarted = false;

        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
        UnpaidIntern.killMotor(motorC);
    }
}