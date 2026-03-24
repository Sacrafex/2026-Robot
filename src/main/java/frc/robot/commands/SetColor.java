package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class SetColor extends Command {
    private final CANdleSubsystem lights;
    private final int r, g, b;

    public SetColor(CANdleSubsystem lights, int r, int g, int b) {
        this.lights = lights;
        this.r = r;
        this.g = g;
        this.b = b;
        addRequirements(lights);
    }

    @Override
    public void initialize() {
        lights.setColor(r, g, b);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}