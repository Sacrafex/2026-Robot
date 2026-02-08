package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TrackCode extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    double vx; double vy; double omega;
    
    private static final double kP_DIST = 1;
    private static final double kP_STRAFE = 0.2;
    private static final double kP_ROT = 4;

    private static final double MAX_VEL = 3.0;
    private static final double MAX_OMEGA = 10;

    private static final double GUESS_REDUCTION_MPL = 0.7;
    private static final double GUESS_REDUCTION_MPL_DEG = 0.9;

    private static final double vy_zero = 0.00001;
    private static final double omega_zero = 0.00001;

    private double lastTime = Timer.getFPGATimestamp();
    private final double interval = 0.05;

    private final XboxController joystick;

    public TrackCode(CommandSwerveDrivetrain drivetrain, XboxController joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        LimelightHelpers.setLEDMode_ForceOn("limelight-front");
        //LimelightHelpers.setLEDMode_ForceOff("limelight-front");

        if (!LimelightHelpers.getTV("limelight-front")) {
            double now = Timer.getFPGATimestamp();
            if (now - lastTime >= interval) {
                vy=vy*GUESS_REDUCTION_MPL; omega=omega*GUESS_REDUCTION_MPL_DEG;
                if(vy<vy_zero){vy=0;}if(omega<omega_zero){omega=0;}
                lastTime = now;
            }
            drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
            return;
        }

        double tx = LimelightHelpers.getTX("limelight-front");
        double strafeError = LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[1];
        double angleError = Math.toRadians(tx);

        vx = joystick.getLeftTriggerAxis() * kP_DIST;
        SmartDashboard.putNumber("vx", vx);
        vy = strafeError * kP_STRAFE;
        SmartDashboard.putNumber("vy", vy);
        omega = -angleError * kP_ROT;
        SmartDashboard.putNumber("omega", omega);

        vx = Math.max(-MAX_VEL, Math.min(MAX_VEL, vx));
        vy = Math.max(-MAX_VEL, Math.min(MAX_VEL, vy));
        omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, omega));

        drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}