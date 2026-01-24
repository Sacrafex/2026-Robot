package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
    private static final double kP = 0.009;

    private static final double FIRSTPOS = 0.4;
    private static final double MAXPOS = 0.9;

    // Meters
    private static final double BLUE_HOPPER_X = 4.626;
    private static final double BLUE_HOPPER_Y = 8.069;
    private static final double RED_HOPPER_X = 11.834;
    private static final double RED_HOPPER_Y = 8.069;

    private static boolean autoIntakeLightsEnabled = true;

    public static class LimelightConfig {
        String name; double x, y, z, roll, pitch, yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw){
            this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;
        }
    }

    public static final LimelightConfig[] LIMELIGHTS = {
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
            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw
            );
        }
    }

    @Override
    public void execute() {

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        boolean usingVision = false;

        LimelightHelpers.PoseEstimate ll;

        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOn(cam.name);

            ll = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam.name);

            if (ll != null && ll.tagCount >= 1) {
                drivetrain.addVisionMeasurement(ll.pose, ll.timestampSeconds);
                usingVision = true;
            }
        }

        SmartDashboard.putBoolean("UsingVision", usingVision);

        if (autoIntakeLightsEnabled) {
            if (usingVision) lights.setColor(0, 255, 0);
            else lights.setColor(255, 120, 0);
        }

        Pose2d poseToUse = drivetrain.getPose();

        double hopperX, hopperY;
        if (alliance == DriverStation.Alliance.Blue) {
            hopperX = BLUE_HOPPER_X;
            hopperY = BLUE_HOPPER_Y;
            SmartDashboard.putString("Alliance", "BLUE");
        } else {
            hopperX = RED_HOPPER_X;
            hopperY = RED_HOPPER_Y;
            SmartDashboard.putString("Alliance", "RED");
        }

        double botPoseX = poseToUse.getX();
        double botPoseY = poseToUse.getY();
        double botRot = poseToUse.getRotation().getDegrees();

        SmartDashboard.putNumber("botPoseX", botPoseX);
        SmartDashboard.putNumber("botPoseY", botPoseY);

        double dx = hopperX - botPoseX;
        double dy = hopperY - botPoseY;

        double r = Math.sqrt(dx*dx + dy*dy);
        SmartDashboard.putNumber("Distance from Hopper", r);

        double omega = Math.toDegrees(Math.atan2(dy, dx));
        SmartDashboard.putNumber("Omega", omega);

        double error = omega - botRot;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        double appliedOmega = error * kP;
        SmartDashboard.putNumber("AppliedOmega", appliedOmega);

        if (autoButton.getAsBoolean() && Math.abs(error) < DEAD_BAND) {
            double targetInputSpeed = lerp(FIRSTPOS, MAXPOS, r);
            intake.set(targetInputSpeed);
            SmartDashboard.putNumber("intakeSpeed", targetInputSpeed);
        } else {
            intake.stop();
        }

        // Apply Drivetrain Control
        drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(appliedOmega));
        SmartDashboard.putBoolean("isAutoActive", true);
    }

    @Override
    public void end(boolean interrupted){
        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        drivetrain.setControl(m_request.withRotationalRate(0));
        intake.stop();
        outtakeAngle.stop();
        SmartDashboard.putBoolean("isAutoActive", false);
    }

    @Override
    public boolean isFinished(){ return false; }

    // In the future I need to make a multiple point LERP
    public double lerp(double v0, double v1, double vn) {
        return v0 + vn * (v1 - v0);
    }
}