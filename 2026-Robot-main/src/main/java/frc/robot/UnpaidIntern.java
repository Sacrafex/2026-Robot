// Helper Script created by Killian Zabinsky (@Sacrafex) for Team 2839
// Please read INFORMATION.md for usage instructions and information.
// Uses REVLib 2025.0.3, CTRE Phoenix v6 25.4.0, CTRE Phoenix v5 5.35.1, WPILib-New-Commands 1.0.0

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

@SuppressWarnings("unused")
public final class UnpaidIntern {

    private UnpaidIntern() {}

    private static final int MAX_MOTORS = 32;
    private static final double[] talonFXZeroOffset = new double[MAX_MOTORS];
    private static final double[] sparkZeroOffset = new double[MAX_MOTORS];
    private static final double[] minLimits = new double[MAX_MOTORS];
    private static final double[] maxLimits = new double[MAX_MOTORS];
    private static final boolean[] hasLimits = new boolean[MAX_MOTORS];
    private static final boolean[] motorKilled = new boolean[MAX_MOTORS];

    private static int id(TalonFX motor) {
        return motor.getDeviceID();
    }
    private static int id(SparkMax motor) {
        return motor.getDeviceId();
    }

    public static double deadband(double value, double band) {
        return Math.abs(value) < band ? 0.0 : value;
    }

    public static void setPercentWithDeadband(TalonFX motor, double input, double band) {
        if (motorKilled[id(motor)]) return;
        motor.setControl(new DutyCycleOut(deadband(input, band)));
    }

    public static void setPercentWithDeadband(SparkMax motor, double input, double band) {
        if (motorKilled[id(motor)]) return;
        motor.setVoltage(deadband(input, band) * 12.0);
    }

    public static void zeroEncoder(TalonFX motor) {
        talonFXZeroOffset[id(motor)] = motor.getPosition().getValueAsDouble();
    }
    
    public static double getZeroedPosition(TalonFX motor) {
        return motor.getPosition().getValueAsDouble() - talonFXZeroOffset[id(motor)];
    }
    
    public static double getActualPosition(TalonFX motor) {
        return motor.getPosition().getValueAsDouble();
    }

    public static void zeroEncoder(SparkMax motor) {
        RelativeEncoder enc = motor.getEncoder();
        sparkZeroOffset[id(motor)] = enc.getPosition();
    }
    
    public static double getZeroedPosition(SparkMax motor) {
        RelativeEncoder enc = motor.getEncoder();
        return enc.getPosition() - sparkZeroOffset[id(motor)];
    }
    
    public static double getActualPosition(SparkMax motor) {
        return motor.getEncoder().getPosition();
    }

    public static void setLimits(TalonFX motor, double min, double max) {
        int i = id(motor);
        minLimits[i] = min;
        maxLimits[i] = max;
        hasLimits[i] = true;
    }
    
    public static void setLimits(SparkMax motor, double min, double max) {
        int i = id(motor);
        minLimits[i] = min;
        maxLimits[i] = max;
        hasLimits[i] = true;
    }

    public static void clearLimits(TalonFX motor) {
        hasLimits[id(motor)] = false;
    }
    
    public static void clearLimits(SparkMax motor) {
        hasLimits[id(motor)] = false;
    }

    private static double clamp(TalonFX motor, double val) {
        int i = id(motor);
        return (!hasLimits[i]) ? val : Math.max(minLimits[i], Math.min(maxLimits[i], val));
    }
    
    private static double clamp(SparkMax motor, double val) {
        int i = id(motor);
        return (!hasLimits[i]) ? val : Math.max(minLimits[i], Math.min(maxLimits[i], val));
    }

    public static void toPosition(TalonFX motor, double position) {
        if (motorKilled[id(motor)]) return;
        double target = clamp(motor, position);
        motor.setControl(new PositionVoltage(target));
    }

    public static boolean atPosition(TalonFX motor, double target, double tol) {
        return Math.abs(getZeroedPosition(motor) - target) <= tol;
    }

    public static void toPosition(SparkMax motor, double position) {
        if (motorKilled[id(motor)]) return;
        SparkClosedLoopController ctrl = motor.getClosedLoopController();
        double targetPos = clamp(motor, position);
        ctrl.setSetpoint(targetPos, ControlType.kPosition);
    }

    public static boolean atPosition(SparkMax motor, double target, double tol) {
        return Math.abs(getZeroedPosition(motor) - target) <= tol;
    }

    public static void toVelocity(TalonFX motor, double velocity) {
        if (motorKilled[id(motor)]) return;
        motor.setControl(new VelocityVoltage(velocity));
    }

    public static void toVelocity(SparkMax motor, double velocity) {
        if (motorKilled[id(motor)]) return;

        SparkClosedLoopController ctrl = motor.getClosedLoopController();
        double targetVel = clamp(motor, velocity);

        ctrl.setSetpoint(targetVel, ControlType.kVelocity);
    }

    public static void killMotor(TalonFX motor) {
        motorKilled[id(motor)] = true;
        motor.stopMotor();
    }
    
    public static void killMotor(SparkMax motor) {
        motorKilled[id(motor)] = true;
        motor.stopMotor();
    }

    public static void reviveMotor(TalonFX motor) {
        motorKilled[id(motor)] = false;
    }
    
    public static void reviveMotor(SparkMax motor) {
        motorKilled[id(motor)] = false;
    }

    public static boolean isMotorKilled(TalonFX motor) {
        return motorKilled[id(motor)];
    }
    
    public static boolean isMotorKilled(SparkMax motor) {
        return motorKilled[id(motor)];
    }

    public static void stop(TalonFX motor) {
        motor.stopMotor();
    }
    
    public static void stop(SparkMax motor) {
        motor.stopMotor();
    }
}