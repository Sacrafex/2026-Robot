package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WebServer;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

public class GoAndShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    // TODO: Tune TurnPIDforGOANDSHOOT
    // We need to find an actual field to tune some of this, just a code and a hopper isn't enough
    private final PIDController omegaPID = new PIDController(0.1, 0.0, 0.05);

    public static final Constants.Limelight.LimelightConfig[] LIMELIGHTS = Constants.Limelight.LIMELIGHTS;

    public GoAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, CANdleSubsystem lights, BooleanSupplier autoButton, XboxController joystick) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;


        addRequirements(drivetrain, shooter, lights);

        for(Constants.Limelight.LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw
            );
        }
    }

    @Override
    public void execute() {

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        double targetX, targetY, presetRotation;

        if (alliance == DriverStation.Alliance.Blue) {
            targetX = 4.626;
            targetY = 8.069;
            presetRotation = 90.0;
        } else {
            targetX = 11.834;
            targetY = 8.069;
            presetRotation = -90.0;
        }

        Pose2d pose = drivetrain.getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        double distance = Math.sqrt(dx*dx + dy*dy);

        WebServer.putNumber("botPoseX", Math.round(pose.getX()*100)/100);
        WebServer.putNumber("botPoseY", Math.round(pose.getY()*100)/100);
        WebServer.putNumber("DistanceFromTarget", Math.round(distance*100)/100);

        double kP = 1.0;
        double maxSpeed = 3.0;
        double xSpeed = MathUtil.clamp(kP * dx, -maxSpeed, maxSpeed);
        double ySpeed = MathUtil.clamp(kP * dy, -maxSpeed, maxSpeed);

        if (distance < 1.0 && Math.abs(pose.getRotation().getDegrees() - presetRotation) < 5) {
            shooter.matchRotations(2000);
        } else {
            shooter.stop();
        }

        drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(omegaPID.calculate(pose.getRotation().getDegrees(), presetRotation)));
    }

    @Override
    public void end(boolean interrupted){
        for(Constants.Limelight.LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        drivetrain.setControl(drive.withRotationalRate(0));
        shooter.stop();
    }

    @Override
    public boolean isFinished(){ return false; }
}