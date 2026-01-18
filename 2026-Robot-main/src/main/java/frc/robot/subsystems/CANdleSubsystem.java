package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle;

    public CANdleSubsystem(int deviceID) {
        candle = new CANdle(deviceID);
    }

    public void setColor(int r, int g, int b) {
        RGBWColor color = new RGBWColor(r, g, b, 0);

        candle.setControl(
            new SolidColor(0, 399)
                .withColor(color)
        );
    }

    public void off() {
        candle.setControl(new EmptyAnimation(0));
    }
}