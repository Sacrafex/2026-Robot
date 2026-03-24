package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WebServer;
import frc.robot.Constants;

public class GoAndShoot extends Command {

    
    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final CANdleSubsystem lights;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    
    public static PIDController omegaPID = Constants.GoAndShoot.omegaPID;
    public static PIDController drivePIDx = Constants.GoAndShoot.drivePIDx;
    public static PIDController drivePIDy = Constants.GoAndShoot.drivePIDy;

    public static final Constants.Limelight.LimelightConfig[] LIMELIGHTS = Constants.Limelight.LIMELIGHTS;

    public GoAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, CANdleSubsystem lights) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.lights = lights;

        addRequirements(drivetrain, shooter);

        omegaPID.enableContinuousInput(-180, 180);

        for (Constants.Limelight.LimelightConfig cam : LIMELIGHTS) {
            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw
            );
        }
    }

    @Override
    public void execute() {

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        double targetX;
        double targetY;
        double presetRotation;

        if (alliance == DriverStation.Alliance.Blue) {
            targetX = Constants.GoAndShoot.BLUE_ALLIANCE.targetX;
            targetY = Constants.GoAndShoot.BLUE_ALLIANCE.targetY;
            presetRotation = Constants.GoAndShoot.BLUE_ALLIANCE.presetRotation;
        } else {
            targetX = Constants.GoAndShoot.RED_ALLIANCE.targetX;
            targetY = Constants.GoAndShoot.RED_ALLIANCE.targetY;
            presetRotation = Constants.GoAndShoot.RED_ALLIANCE.presetRotation;
        }

        Pose2d pose = drivetrain.getPose();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        WebServer.putNumber("botPoseX", Math.round(pose.getX() * 100) / 100.0);
        WebServer.putNumber("botPoseY", Math.round(pose.getY() * 100) / 100.0);
        WebServer.putNumber("DistanceFromTarget", Math.round(distance * 100) / 100.0);

        double maxSpeed = 3.0;

        double xSpeed = MathUtil.clamp(drivePIDx.calculate(pose.getX(), targetX),-maxSpeed,maxSpeed);
        double ySpeed = MathUtil.clamp(drivePIDy.calculate(pose.getY(), targetY),-maxSpeed,maxSpeed);
        double rotRate = omegaPID.calculate(pose.getRotation().getDegrees(),presetRotation);
        double rotError = MathUtil.inputModulus(pose.getRotation().getDegrees() - presetRotation,-180,180);

        if (distance < 1.0 && Math.abs(rotError) < Constants.GoAndShoot.SHOOT_DEADBAND && Constants.GoAndShoot.autoShoot) {
            shooter.matchRotations(2000);
        } else {
            shooter.stop();
        }

        lights.setColor(255, 0, 0);

        drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotRate));
    }

    @Override
    public void end(boolean interrupted) {
        for (Constants.Limelight.LimelightConfig cam : LIMELIGHTS) {
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        drivetrain.setControl(
            drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        shooter.stop();
        lights.setColor(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}