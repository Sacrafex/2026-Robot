package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class OuttakeAngle extends SubsystemBase {

    private final TalonFX motor;
    private final double kP = 0.05;

    public OuttakeAngle(int motorCANId) {
        motor = new TalonFX(motorCANId);
        UnpaidIntern.zeroEncoder(motor);
    }

    public void setAngle(double targetDegrees) {
        double currentRotations = motor.getRotorPosition().getValueAsDouble();
        double currentDegrees = currentRotations * 360.0;
        double error = targetDegrees - currentDegrees;

        double output = error * kP;
        output = Math.max(Math.min(output, 1.0), -1.0);

        motor.set(output);

        SmartDashboard.putNumber("Outtake Target Angle", targetDegrees);
        SmartDashboard.putNumber("Outtake Current Angle", currentDegrees);
        SmartDashboard.putNumber("Outtake Motor Output", output);
    }

    public double getAngle() {
        return motor.getRotorPosition().getValueAsDouble() * 360.0;
    }

    public void stop() {
        motor.set(0.0);
    }
}