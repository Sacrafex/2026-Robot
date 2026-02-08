package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterLinearActuator;
import frc.robot.subsystems.WebServer;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.function.BooleanSupplier;

public class AimAndShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final ShooterLinearActuator ShooterLinearActuator;
    private final BooleanSupplier autoButton;
    private final CANdleSubsystem lights;
    private final XboxController joystick;
    private PolynomialSplineFunction intakeSpeedSpline;
    private PolynomialSplineFunction shooterAngleSpline;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private static final double SWERVE_ALIGN_DEADBAND = 4;
    private static final double INTAKE_ANGLE_DEADBAND = 0.005;
    private static final double kP = 0.1;

    // Meters
    private static final double BLUE_HOPPER_X = 4.626;
    private static final double BLUE_HOPPER_Y = 8.069;
    private static final double RED_HOPPER_X = 11.834;
    private static final double RED_HOPPER_Y = 8.069;

    private static boolean autoIntakeLightsEnabled = true;
    private static boolean allowAlignDriveControl = true;

    public static class LimelightConfig {
        String name; double x, y, z, roll, pitch, yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw){
            this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;
        }
    }

    public static final LimelightConfig[] LIMELIGHTS = {
        new LimelightConfig("limelight-front",0.01125,-0.005,0.095,Math.toRadians(0),Math.toRadians(58),Math.toRadians(0))
    };

    public AimAndShoot(CommandSwerveDrivetrain drivetrain, Intake intake,
                       ShooterLinearActuator ShooterLinearActuator, CANdleSubsystem lights,
                       BooleanSupplier autoButton, XboxController joystick) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.ShooterLinearActuator = ShooterLinearActuator;
        this.autoButton = autoButton;
        this.lights = lights;
        this.joystick = joystick;


        addRequirements(drivetrain, intake, ShooterLinearActuator, lights);

        double[] distances = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
        double[] speeds = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
        double[] angles = {5, 10, 18, 25, 32, 38};
        intakeSpeedSpline = new SplineInterpolator().interpolate(distances, speeds);
        shooterAngleSpline = new SplineInterpolator().interpolate(distances, angles);

        // Apply Robot Offset
        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw
            );
        }
    }

    @Override
    public void execute() {

        // For drivebase movement when aligning
        double xSpeed;
        if (allowAlignDriveControl) {
            xSpeed = joystick.getLeftX();
        } else {
            xSpeed = 0;
        }

        double ySpeed;
        if (allowAlignDriveControl) {
            ySpeed = joystick.getLeftY();
        } else {
            ySpeed = 0;
        }

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        boolean usingVision = false;

        LimelightHelpers.PoseEstimate ll;

        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOn(cam.name);

            ll = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam.name);

            if (ll != null && ll.pose != null && ll.tagCount >= 1) {
                drivetrain.addVisionMeasurement(ll.pose, ll.timestampSeconds);
                usingVision = true;
            }
        }

        SmartDashboard.putBoolean("UsingVision", usingVision);

        if (autoIntakeLightsEnabled) {
            if (usingVision) { lights.setColor(0, 255, 0); joystick.setRumble(RumbleType.kRightRumble, 0); }
            else { lights.setColor(255, 120, 0); joystick.setRumble(RumbleType.kRightRumble, 0.5); }
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

        WebServer.putNumber("botPoseX", botPoseX);
        WebServer.putNumber("botPoseY", botPoseY);

        double dx = hopperX - botPoseX;
        double dy = hopperY - botPoseY;

        double r = Math.sqrt(dx*dx + dy*dy);
        WebServer.putNumber("DistancefromHopper", r);

        double omega = Math.toDegrees(Math.atan2(dy, dx));
        WebServer.putNumber("Omega", omega);

        double error = omega - botRot;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        double appliedOmega = error * kP;
        WebServer.putNumber("AppliedOmega", appliedOmega);

        if (autoButton.getAsBoolean() && Math.abs(error) < SWERVE_ALIGN_DEADBAND) {
            double targetInputSpeed = intakeSpeedSpline.value(r);
            // May or may not work, I should problably make the value correspond to an angle and a speed but both as one would be simpler.
            double targetShooterAngle = shooterAngleSpline.value(r);

            if (ShooterLinearActuator.isAtTarget(targetShooterAngle, INTAKE_ANGLE_DEADBAND)) {
                intake.set(targetInputSpeed);
            } else {
                intake.set(0.0);
                ShooterLinearActuator.setPosition(targetShooterAngle);
            }

            WebServer.putNumber("intakeSpeed", targetInputSpeed);
            WebServer.putNumber("targetShooterAngle", targetShooterAngle);
        } else {
            intake.stop();
        }

        if (joystick.getRightTriggerAxis() >= 0.2) {
            intake.set(joystick.getRightTriggerAxis());
            ShooterLinearActuator.setPosition(joystick.getRightTriggerAxis());

            WebServer.putNumber("intakeSpeed", joystick.getRightTriggerAxis());
            WebServer.putNumber("targetShooterAngle", joystick.getRightTriggerAxis());
        }

        drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(appliedOmega));
        SmartDashboard.putBoolean("isAutoActive", true);
    }

    @Override
    public void end(boolean interrupted){
        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        drivetrain.setControl(drive.withRotationalRate(0));
        intake.stop();
        ShooterLinearActuator.stop();
        SmartDashboard.putBoolean("isAutoActive", false);
    }

    @Override
    public boolean isFinished(){ return false; }
}
