package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.WebServer;
import frc.robot.subsystems.CANdleSubsystem;

public class TrackCode extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private final CANdleSubsystem lights;
    private final boolean usingLights;

    double vx; double vy; double omega;
    
    private static final double kP_DIST = 1;
    private static final double kP_STRAFE = 0.1;

    private static final double MAX_VEL = 8.0;
    private static final double MAX_OMEGA = 10;

    private static final double GUESS_REDUCTION_MPL = 0.7;
    private static final double GUESS_REDUCTION_MPL_DEG = 0.9;

    private static final double vy_zero = 0.00001;
    private static final double omega_zero = 0.00001;

    // TODO: Tune Code Tracking PID
    private final PIDController codePID = new PIDController(0.1, 0.0, 0.05);

    private double lastTime = Timer.getFPGATimestamp();
    private final double interval = 0.05;

    private final XboxController joystick;

    public TrackCode(CommandSwerveDrivetrain drivetrain, XboxController joystick, CANdleSubsystem lights, boolean usingLights) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.lights = lights;
        this.usingLights = usingLights;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (Constants.Limelight.ENABLE_LIMELIGHT_LIGHTS && usingLights) {
        LimelightHelpers.setLEDMode_ForceOn("limelight-front");
        }

        if (!LimelightHelpers.getTV("limelight-front")) {
            double now = Timer.getFPGATimestamp();
            if (now - lastTime >= interval) {
                vy=vy*GUESS_REDUCTION_MPL; omega=omega*GUESS_REDUCTION_MPL_DEG;
                if(vy<vy_zero){vy=0;}if(omega<omega_zero){omega=0;}
                lastTime = now;
            }
            drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
            lights.setColor(255, 0, 0);
            return;
        }

        double tx = LimelightHelpers.getTX("limelight-front");
        double strafeError = LimelightHelpers.getTargetPose_RobotSpace("limelight-front")[1];
        double angleError = Math.toRadians(tx);

        vx = joystick.getLeftTriggerAxis() * kP_DIST;
        WebServer.putNumber("TCvx", vx);
        vy = strafeError * kP_STRAFE;
        WebServer.putNumber("TCvy", vy);
        // TODO: IF DOESN'T WORK, TRY INVERSING angleError
        omega = codePID.calculate(-angleError, 0);
        WebServer.putNumber("TComega", omega);

        vx = Math.max(-MAX_VEL, Math.min(MAX_VEL, vx));
        vy = Math.max(-MAX_VEL, Math.min(MAX_VEL, vy));
        omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, omega));

        drivetrain.setControl(drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
        lights.setColor(0, 255, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        LimelightHelpers.setLEDMode_ForceOff("limelight-front");
        lights.setColor(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}