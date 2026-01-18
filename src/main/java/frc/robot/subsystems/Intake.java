package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class Intake extends SubsystemBase {
    private final TalonFX motorA;
    private final TalonFX motorB;

    private static final double DEADBAND = 0;

    public Intake(int motorACANId, int motorBCANId) {
        motorA = new TalonFX(motorACANId);
        motorB = new TalonFX(motorBCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
    }

    public void set(double speed) {
        UnpaidIntern.setPercentWithDeadband(motorA, speed, DEADBAND);
        UnpaidIntern.setPercentWithDeadband(motorB, speed, DEADBAND);
    }

    public void stop() {
        UnpaidIntern.stop(motorA);
        UnpaidIntern.stop(motorB);
    }

    public void kill() {
        UnpaidIntern.killMotor(motorA);
        UnpaidIntern.killMotor(motorB);
    }

    public TalonFX getMotorA() {
        return motorA;
    }

    public TalonFX getMotorB() {
        return motorB;
    }
}
