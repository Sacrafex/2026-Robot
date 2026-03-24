package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle;
    private final Timer animationTimer;
    private int animationStep;
    private final int ledCount;

    public CANdleSubsystem(int deviceID, String canbus, int ledCount) {
        candle = new CANdle(deviceID, canbus);
        animationTimer = new Timer();
        animationTimer.start();
        animationStep = 0;
        this.ledCount = ledCount;
    }

    public void setColor(int r, int g, int b) {
        RGBWColor color = new RGBWColor(r, g, b, 0);
        candle.setControl(new SolidColor(0, ledCount - 1).withColor(color));
    }

    public void off() {
        candle.setControl(new EmptyAnimation(0));
    }

    public void shootingStarAnimation(int r, int g, int b) {
        if (animationTimer.hasElapsed(0.05)) {
            animationTimer.reset();
            for (int i = 0; i < ledCount; i++) {
                if (i == animationStep) {
                    candle.setControl(new SolidColor(i, i).withColor(new RGBWColor(r, g, b, 0)));
                } else if (i > (animationStep - 7 + ledCount) % ledCount && i < animationStep) {
                    int trailIndex = (animationStep - i + ledCount) % ledCount;
                    int brightness = 255 - (trailIndex * 36);
                    candle.setControl(new SolidColor(i, i).withColor(new RGBWColor(
                        (r * brightness) / 255, (g * brightness) / 255, (b * brightness) / 255, 0)));
                } else {
                    candle.setControl(new SolidColor(i, i).withColor(new RGBWColor(0, 0, 0, 0)));
                }
            }
            animationStep = (animationStep + 1) % ledCount;
        }
    }

    public void breathingAnimation(int r, int g, int b) {
        if (animationTimer.hasElapsed(0.05)) {
            animationTimer.reset();
            int brightness = (int) (128 + 127 * Math.sin(animationStep * Math.PI / 90));
            RGBWColor color = new RGBWColor((r * brightness) / 255, (g * brightness) / 255, (b * brightness) / 255, 0);
            candle.setControl(new SolidColor(0, ledCount - 1).withColor(color));
            animationStep = (animationStep + 1) % 180;
        }
    }

    public void fireAnimation() {
        if (animationTimer.hasElapsed(0.05)) {
            animationTimer.reset();
            int[][] fireColors = {
                {255, 69, 0},
                {255, 99, 71},
                {255, 215, 0},
                {255, 165, 0},
                {139, 0, 0}
            };
            for (int i = 0; i < ledCount; i++) {
                int flicker = (int) (Math.random() * 50) - 25;
                int colorIndex = i % fireColors.length;
                int[] color = fireColors[colorIndex];
                int r = Math.min(255, Math.max(0, color[0] + flicker));
                int g = Math.min(255, Math.max(0, color[1] + flicker));
                int b = Math.min(255, Math.max(0, color[2] + flicker));
                candle.setControl(new SolidColor(i, i).withColor(new RGBWColor(r, g, b, 0)));
            }
        }
    }
}