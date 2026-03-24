package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class EstimatePose extends Command {

    private final CANdleSubsystem lights;
    private final CommandSwerveDrivetrain drivetrain;
    public static final Constants.Limelight.LimelightConfig[] LIMELIGHTS = Constants.Limelight.LIMELIGHTS;
    private final boolean usingLights;

    private int currentIndex = 0;
    private double lastTime = 0.0;
    private final double interval = Constants.EstimatePose.interval;
    private boolean lightsOn = false;

    public EstimatePose(CANdleSubsystem lights,CommandSwerveDrivetrain drivetrain, boolean usingLights) {

        this.lights = lights;
        this.drivetrain = drivetrain;
        this.usingLights = usingLights;

        addRequirements(lights);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
            if (now - lastTime >= interval) {
                for (Constants.Limelight.LimelightConfig cam : LIMELIGHTS) {
                    LimelightHelpers.setLEDMode_ForceOff(cam.name);
                }
                if (Constants.Limelight.ENABLE_LIMELIGHT_LIGHTS && usingLights) {
                LimelightHelpers.setLEDMode_ForceOn(LIMELIGHTS[currentIndex].name);
                }
                LimelightHelpers.PoseEstimate ll = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHTS[currentIndex].name);

                if (ll != null && ll.pose != null && ll.tagCount > 0) {
                drivetrain.addVisionMeasurement(ll.pose, ll.timestampSeconds);
                }

                if (lightsOn && usingLights) {
                    lights.setColor(0, 0, 0);
                } else {
                    lights.setColor(0, 255, 0);
                }

                lightsOn = !lightsOn;

                currentIndex++;
                if (currentIndex >= LIMELIGHTS.length) {
                    currentIndex = 0;
                }

                lastTime = now;
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (Constants.Limelight.LimelightConfig cam : LIMELIGHTS) {
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        lights.setColor(0, 0, 0);
        currentIndex = 0;
        lastTime = 0.0;
        lightsOn = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}