package frc.robot.subsystems;

import com.ctre.phoenix6.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Intake extends SubsystemBase {
    private SparkFlex intakeMotor = new SparkFlex(3, MotorType.kBrushless);
    private static final double DEADBAND = 0;

    public Intake(int motorCANId) {

    }

    public void set(double speed) {
        intakeMotor.set(0.5);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}