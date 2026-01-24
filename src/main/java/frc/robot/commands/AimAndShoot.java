package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final double deadbandDeg;
    private final CANdleSubsystem lights;

    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();
    
    private final PIDController turnPID = new PIDController(0.2, 0.0, 0.3);

    private static final double DEAD_BAND = 4;
    private static final double ANGLE_MIN = 20.0;
    private static final double ANGLE_MAX = 60.0;
    private static final double SPEED_MIN = 0.35; 
    private static final double SPEED_MAX = 0.8; 
    private static final double SHOOT_DISTANCE = 12.0;

    private static final double BLUE_HOPPER_X = 4.626;
    private static final double BLUE_HOPPER_Y = 8.069;
    private static final double RED_HOPPER_X = 11.834;
    private static final double RED_HOPPER_Y = 8.069;

    private static boolean autoIntakeLightsEnabled = true;

    private Pose2d lastKnownPose = null;

    private Double latchedTargetAngle = null;
    private int lastSeenTag = -1;
    private int lostCounter = 0;
    private static final int LOST_LIMIT = 10;
    
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
        this.deadbandDeg = DEAD_BAND;
        this.lights = lights;

        addRequirements(drivetrain, intake, outtakeAngle, lights);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw);
        }
    }

    @Override
    public void execute() {

        double omega = 0.0;
        Pose2d botPose = getAverageBotPose();

        if(botPose == null && lastKnownPose != null) botPose = lastKnownPose;

        if(botPose != null){
            lastKnownPose = botPose;

            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            double hopperX = (alliance==DriverStation.Alliance.Blue)? BLUE_HOPPER_X : RED_HOPPER_X;
            double hopperY = (alliance==DriverStation.Alliance.Blue)? BLUE_HOPPER_Y : RED_HOPPER_Y;
	    
            double dx = hopperX - botPose.getX();
            SmartDashboard.putNumber("Calculated DX", dx);
            System.out.println("dx: "+dx);
            double dy = hopperY - botPose.getY();
            SmartDashboard.putNumber("Calculated DY", dy);
            System.out.println("dy: "+dy);

            // Check which tag is currently being found
            int currentTag = (int) LimelightHelpers.getFiducialID("limelight-front");
            boolean seesTag = currentTag != -1;
            SmartDashboard.putNumber("currentFoundTag", currentTag);

            if (seesTag) {
                lostCounter = 0;
                if (latchedTargetAngle == null || currentTag != lastSeenTag) {
                    // Find Angle
                    latchedTargetAngle = Math.atan2(dy, dx);
                    lastSeenTag = currentTag;
                    turnPID.reset();
                }
            } else {
                lostCounter++;
                if (lostCounter > LOST_LIMIT) {
                    latchedTargetAngle = null;
                    lastSeenTag = -1;
                }
            }

            if (latchedTargetAngle != null) {
                double currentAngleRad = botPose.getRotation().getRadians();

                // Find quickest path
                double error = latchedTargetAngle - currentAngleRad;
                while(error > Math.PI) error -= 2*Math.PI;
                while(error < -Math.PI) error += 2*Math.PI;

                SmartDashboard.putNumber("AngleError", error);

                if(Math.abs(Math.toDegrees(error)) > deadbandDeg){
                    omega = turnPID.calculate(currentAngleRad, latchedTargetAngle);
                    omega = Math.max(Math.min(omega,3.0),-3.0);
                } else {
                    omega = 0;
                }
            } else {
                omega = 0;
            }

            SmartDashboard.putNumber("omega", omega);
            System.out.println("Found Omega: " + omega);

            if(autoButton.getAsBoolean() && latchedTargetAngle != null && Math.abs(omega) < 0.05){
                if (autoIntakeLightsEnabled) {lights.setColor(79, 52, 235);};

                double distanceFeet = Math.hypot(dx, dy)*3.28084;
                SmartDashboard.putNumber("Distance from Tag", distanceFeet);

                double distanceFactor = Math.min(distanceFeet/SHOOT_DISTANCE,1.0);
                double scaleFactor = Math.sqrt(distanceFactor);

                double targetOuttakeAngle = ANGLE_MIN + (ANGLE_MAX-ANGLE_MIN)*scaleFactor;
                targetOuttakeAngle = Math.min(Math.max(targetOuttakeAngle,ANGLE_MIN),ANGLE_MAX);

                double targetSpeed = SPEED_MIN + (SPEED_MAX-SPEED_MIN)*scaleFactor;
                targetSpeed = Math.min(Math.max(targetSpeed,SPEED_MIN),SPEED_MAX);

                intake.set(targetSpeed);
                outtakeAngle.setAngle(targetOuttakeAngle);
            } else {
                intake.stop();
                outtakeAngle.stop();
                if (autoIntakeLightsEnabled) {lights.setColor(11, 222, 0);};
            }
            SmartDashboard.putBoolean("IsDetected", true);

        } else {
            System.out.println("Limelight didn't return data.");
            SmartDashboard.putBoolean("IsDetected", false);
            if (autoIntakeLightsEnabled) {lights.setColor(250, 75, 43);};
        }

        drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(omega));
        SmartDashboard.putBoolean("isAutoActive", true);
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

    private Pose2d getAverageBotPose() {
        double sumX = 0, sumY = 0, sumCos = 0, sumSin = 0;
        int count = 0;

        for(LimelightConfig cam : LIMELIGHTS){
            if(LimelightHelpers.getTV(cam.name)){
                Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(cam.name);
                if(pose == null) continue;

                sumX += pose.getX();
                sumY += pose.getY();
                double rot = pose.getRotation().getRadians();
                sumCos += Math.cos(rot);
                sumSin += Math.sin(rot);
                count++;
            }
        }

        if(count == 0){
            if (autoIntakeLightsEnabled) { lights.setColor(235, 52, 52); }
            return null;
        }

        double avgX = sumX / count;
        SmartDashboard.putNumber("avgX", avgX);
        double avgY = sumY / count;
        SmartDashboard.putNumber("avgY", avgY);
        double avgRot = Math.atan2(sumSin, sumCos);
        SmartDashboard.putNumber("avgRot", avgRot);

        return new Pose2d(avgX, avgY, new Rotation2d(avgRot));
    }
}
