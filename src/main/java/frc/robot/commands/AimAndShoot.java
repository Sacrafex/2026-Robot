package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WebServer;
import frc.robot.Constants;

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.function.BooleanSupplier;

public class AimAndShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final BooleanSupplier autoButton;
    private final CANdleSubsystem lights;
    private final XboxController joystick;
    private PolynomialSplineFunction shooterSpeedSpline;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private static final double SWERVE_ALIGN_DEADBAND = 5;

    // Meters
    private static final double BLUE_HOPPER_X = 4.626;
    private static final double BLUE_HOPPER_Y = 8.069;
    private static final double RED_HOPPER_X = 11.834;
    private static final double RED_HOPPER_Y = 8.069;

    private static boolean autoIntakeLightsEnabled = true;
    private static boolean allowAlignDriveControl = true;

    // TODO: Tune AutoAlignPID for AIMANDSHOOT
    // TODO: Look into converting degrees to rads (might improve speed)
    private final PIDController omegaPID = new PIDController(0.1, 0.0, 0.05);

    public static final Constants.Limelight.LimelightConfig[] LIMELIGHTS = Constants.Limelight.LIMELIGHTS;

    public AimAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, CANdleSubsystem lights, BooleanSupplier autoButton, XboxController joystick) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.autoButton = autoButton;
        this.lights = lights;
        this.joystick = joystick;


        addRequirements(drivetrain, shooter, lights);

        double[] distances = Constants.Trajectory.distances;
        double[] speeds = Constants.Trajectory.speeds;

        shooterSpeedSpline = new AkimaSplineInterpolator().interpolate(distances, speeds);

        for(Constants.Limelight.LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(
                cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw
            );
        }
    }

    @Override
    public void execute() {

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

        for (Constants.Limelight.LimelightConfig cam : LIMELIGHTS){
            if (Constants.Limelight.ENABLE_LIMELIGHT_LIGHTS) {
            LimelightHelpers.setLEDMode_ForceOn(cam.name);
            }

            ll = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam.name);

            if (ll != null && ll.pose != null && ll.tagCount >= 1) {
                drivetrain.addVisionMeasurement(ll.pose, ll.timestampSeconds);
                joystick.setRumble(RumbleType.kRightRumble, 0.2); joystick.setRumble(RumbleType.kLeftRumble, 0.2);
                usingVision = true;
            }
        }

        try {
            if (autoIntakeLightsEnabled) {
                if (usingVision) { lights.setColor(0, 255, 0); }
                else { lights.setColor(255, 120, 0); }
            }
        } catch (Exception e) {
            System.out.println("CANdle Lights Not Connected. : " + e);
        }

        Pose2d poseToUse = drivetrain.getPose();

        double hopperX, hopperY;
        if (alliance == DriverStation.Alliance.Blue) {
            hopperX = BLUE_HOPPER_X;
            hopperY = BLUE_HOPPER_Y;
        } else {
            hopperX = RED_HOPPER_X;
            hopperY = RED_HOPPER_Y;
        }

        double botPoseX = poseToUse.getX();
        double botPoseY = poseToUse.getY();

        WebServer.putNumber("botPoseX", Math.round(botPoseX*100)/100);
        WebServer.putNumber("botPoseY", Math.round(botPoseY*100)/100);

        double dx = hopperX - botPoseX;
        double dy = hopperY - botPoseY;

        double r = Math.sqrt(dx*dx + dy*dy);
        WebServer.putNumber("DistancefromHopper", Math.round(r*100)/100);

        double omega = Math.toDegrees(Math.atan2(dy, dx));
        WebServer.putNumber("Omega", Math.round(omega*100)/100);

        double error = omega - drivetrain.getPose().getRotation().getDegrees();

        if (error > 180) error -= 360;
        else if (error < -180) error += 360;

        double appliedOmega = omegaPID.calculate(drivetrain.getPose().getRotation().getDegrees(), omega);
        WebServer.putNumber("AppliedOmega", appliedOmega);

        System.out.println(Math.abs(error));
        if (autoButton.getAsBoolean() && Math.abs(error) < SWERVE_ALIGN_DEADBAND) {
            try {

            double targetInputSpeed = shooterSpeedSpline.value(r);

            shooter.matchRotations(targetInputSpeed);

            } catch(Exception e) {
            System.out.println("Issue Calculating Trajectory: "+e.getMessage());
            }

        } else {
            shooter.stop();
        }

        drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(appliedOmega));
    }

    @Override
    public void end(boolean interrupted){
        for(Constants.Limelight.LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setLEDMode_ForceOff(cam.name);
        }
        lights.setColor(0, 0, 0);
        drivetrain.setControl(drive.withRotationalRate(0));
        joystick.setRumble(RumbleType.kRightRumble, 0);
        joystick.setRumble(RumbleType.kLeftRumble, 0);
        shooter.stop();
    }

    @Override
    public boolean isFinished(){ return false; }
}
