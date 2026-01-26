package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class Elevator extends SubsystemBase {
    private final TalonFX motorA;
    private final TalonFX motorB;

    private static final double DEADBAND = 0;

    private static final double MAX_SPEED = 1;
    private static final double MIN_SPEED = -1;

    public Elevator(int motorACANId, int motorBCANId) {
        motorA = new TalonFX(motorACANId);
        motorB = new TalonFX(motorBCANId);

        UnpaidIntern.zeroEncoder(motorA);
        UnpaidIntern.zeroEncoder(motorB);
    }

    public void set(double speed) {
        if (speed > MAX_SPEED) speed = 1.0;
        if (speed < MIN_SPEED) speed = -1.0;
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