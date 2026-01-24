package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;

import java.util.function.BooleanSupplier;

public class CameraLightCycle extends Command {

    private final CANdleSubsystem lights;
    private final BooleanSupplier button;
    private final AimAndShoot.LimelightConfig[] cams;

    private int currentIndex = 0;
    private double lastTime = 0.0;
    private final double interval = 0.5;

    public CameraLightCycle(CANdleSubsystem lights, AimAndShoot.LimelightConfig[] cams, BooleanSupplier button) {
        this.lights = lights;
        this.cams = cams;
        this.button = button;
        addRequirements(lights);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        if (button.getAsBoolean()) {
            if (now - lastTime >= interval) {
                lights.setColor(0, 0, 255);
                for (AimAndShoot.LimelightConfig cam : cams) {
                    LimelightHelpers.setLEDMode_ForceOff(cam.name);
                }
                LimelightHelpers.setLEDMode_ForceOn(cams[currentIndex].name);
                System.out.println("Camera active: " + cams[currentIndex].name);
                currentIndex++;
                if (currentIndex >= cams.length) currentIndex = 0;

                if (currentIndex % 2 == 0) {
                    lights.setColor(255, 0, 0);
                } else {
                    lights.setColor(0, 0, 0);
                }

                lastTime = now;
            }
        } else {
            currentIndex = 0;
            lastTime = now;
            for (AimAndShoot.LimelightConfig cam : cams) {
                LimelightHelpers.setLEDMode_ForceOff(cam.name);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (AimAndShoot.LimelightConfig cam : cams) {
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        lights.setColor(0, 0, 0);
        currentIndex = 0;
        lastTime = 0.0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}