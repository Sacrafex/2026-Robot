package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OuttakeAngle;

import java.util.function.BooleanSupplier;

public class AimAndShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final OuttakeAngle outtakeAngle;
    private final BooleanSupplier autoButton;
    private final CANdleSubsystem lights;

    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();

    private static final double DEAD_BAND = 4;
    private static final double kP = 0.6;
    private static final double intakeSpeed = 0.5;

    // Meters
    private static final double BLUE_HOPPER_X = 4.626;
    private static final double BLUE_HOPPER_Y = 8.069;
    private static final double RED_HOPPER_X = 11.834;
    private static final double RED_HOPPER_Y = 8.069;

    private static double omega = 0.0;

    private static boolean autoIntakeLightsEnabled = true;
   
    private static class LimelightConfig {
        String name; double x, y, z, roll, pitch, yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw){
            this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;
        }
    }

    private static final LimelightConfig[] LIMELIGHTS = {
            new LimelightConfig("limelight-front",0.3556,0.0,0.13335,Math.toRadians(0),Math.toRadians(55),Math.toRadians(0)),
            new LimelightConfig("limelight-back",-0.3556,0.0,0.13335,Math.toRadians(0),Math.toRadians(55),Math.toRadians(180))
    };

    public AimAndShoot(CommandSwerveDrivetrain drivetrain, Intake intake,
                       OuttakeAngle outtakeAngle, CANdleSubsystem lights,
                       BooleanSupplier autoButton) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.outtakeAngle = outtakeAngle;
        this.autoButton = autoButton;
        this.lights = lights;

        addRequirements(drivetrain, intake, outtakeAngle, lights);

        // Apply Robot Offset
        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw);
        }
    }

    @Override
    public void execute() {

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        // Fetches pose relative to far right side of the driverstations on blue alliance
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

        if(botPose == null) {
            omega = 0.4;
            drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(omega));
            SmartDashboard.putBoolean("IsDetected", false);
            if (autoIntakeLightsEnabled) { lights.setColor(255, 0, 0); }
        } else {
            SmartDashboard.putBoolean("IsDetected", true);
            if (autoIntakeLightsEnabled) { lights.setColor(0, 255, 0); }

            // Set Hopper Pose
            double hopperX = 0; double hopperY = 0;
            if (alliance == DriverStation.Alliance.Blue) {
                hopperX = BLUE_HOPPER_X;
                hopperY = BLUE_HOPPER_Y;
                SmartDashboard.putString("Alliance", "BLUE");
            } else {
                hopperX = RED_HOPPER_X;
                hopperY = RED_HOPPER_Y;
                SmartDashboard.putString("Alliance", "RED");
            }   
            SmartDashboard.putNumber("HopperX", hopperX); SmartDashboard.putNumber("HopperY", hopperY);

            // Set Robot Pose
            double botPoseX = botPose.pose.getX();
            SmartDashboard.putNumber("botPoseX", botPoseX);
            double botPoseY = botPose.pose.getY();
            SmartDashboard.putNumber("botPoseY", botPoseY);

            double botRot = botPose.pose.getRotation().getDegrees();

            // Calculate Rectangular Difference Between Robot and Hopper
            double dx = hopperX - botPose.pose.getX();
            double dy = hopperY - botPose.pose.getY();

            double r = Math.sqrt( (dx)*(dx) + (dy)*(dy) );
            SmartDashboard.putNumber("Distance from Hopper", r);

            // Get Omega in Degrees
            omega = ((Math.atan2(dy, dx))*(180 / Math.PI));
            SmartDashboard.putNumber("Omega", omega);

            // kP Speed Control for Omega
            double appliedOmega = omega * kP;

            // Run Intake if within Deadband
            if(autoButton.getAsBoolean() && Math.abs(omega - botRot) < DEAD_BAND){
                intake.set(intakeSpeed);
            }

            // Apply Drivetrain Control
            drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(appliedOmega));
            SmartDashboard.putBoolean("isAutoActive", true);
        }
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(m_request.withRotationalRate(0));
        intake.stop();
        outtakeAngle.stop();
        SmartDashboard.putBoolean("isAutoActive", false);
    }

    @Override
    public boolean isFinished(){ return false; }
}
