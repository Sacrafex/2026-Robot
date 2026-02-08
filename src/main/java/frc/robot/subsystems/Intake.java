package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;
import frc.robot.subsystems.WebServer;

@SuppressWarnings("unused")
public class Intake extends SubsystemBase {
    private final TalonFX motorA;
    private final TalonFX motorB;
    // private final TalonFX motorC;

    private static final double DEADBAND = 0;

    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;

    //public Intake(int motorACANId, int motorBCANId, int motorCCANId) {
    public Intake(int motorACANId, int motorBCANId) {
        motorA = new TalonFX(motorACANId);
        motorB = new TalonFX(motorBCANId);
        // motorC = new TalonFX(motorBCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
        // UnpaidIntern.zeroEncoder(motorC);
    }

    public void set(double speed) {
        if (speed > MAX_SPEED) speed = 1.0;
        if (speed < MIN_SPEED) speed = -1.0;
        WebServer.putNumber("ShooterSpeed", speed*100);
        UnpaidIntern.setPercentWithDeadband(motorA, speed*0.7, DEADBAND);
        UnpaidIntern.setPercentWithDeadband(motorB, speed, DEADBAND);
        // UnpaidIntern.setPercentWithDeadband(motorC, speed, DEADBAND);
    }

    public void stop() {
        UnpaidIntern.stop(motorA);
        UnpaidIntern.stop(motorB);
        // UnpaidIntern.stop(motorC);
    }

    public void kill() {
        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
        // UnpaidIntern.killMotor(motorC);
    }

    public TalonFX getMotorA() {
        return motorA;
    }
    public TalonFX getMotorB() {
        return motorB;
    }
    // public TalonFX getMotorC() {
    //     return motorC;
    // }
}
