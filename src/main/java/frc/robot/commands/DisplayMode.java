package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class DisplayMode extends Command {
    private final CANdleSubsystem lights;

    public DisplayMode(CANdleSubsystem lights) {
        this.lights = lights;
        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.setColor(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}