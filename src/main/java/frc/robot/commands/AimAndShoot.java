package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    // Allow moving relative to self
    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();
    
    private final PIDController turnPID = new PIDController(4.0, 0.0, 0.3);

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
    
    // Limelight Creation Definition, Do not edit
    private static class LimelightConfig {
        String name; double x, y, z, roll, pitch, yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw){
            this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;
        }
    }

    // https://upload.wikimedia.org/wikipedia/commons/c/c1/Yaw_Axis_Corrected.svg
    // Name , X offset, Y offset, Z offset, Roll offset, Pitch offset, Yaw offset
    // LimelightHelpers Measures distance in Meters - Degree in Radians
    private static final LimelightConfig[] LIMELIGHTS = {
            new LimelightConfig("limelight_front",0.25,0.0,0.8,0.0,Math.toRadians(10),0.0),
            new LimelightConfig("limelight_back",-0.25,0.0,0.8,0.0,Math.toRadians(5),Math.toRadians(180))
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

	// For each added limelight, set the offsets via limelight helpers
        for(LimelightConfig cam : LIMELIGHTS){
            LimelightHelpers.setCameraPose_RobotSpace(cam.name, cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw);
        }
    }

    @Override
    public void execute() {
        double omega = 0.0;
        Pose2d botPose = getAverageBotPose();
        // Use previous position if unknown (I don't actually know if this will work or not)
        if(botPose == null && lastKnownPose != null) botPose = lastKnownPose;

        if(botPose != null){
            lastKnownPose = botPose;

            // Get hopper location based on alliance side
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            double hopperX = (alliance==DriverStation.Alliance.Blue)? BLUE_HOPPER_X : RED_HOPPER_X;
            double hopperY = (alliance==DriverStation.Alliance.Blue)? BLUE_HOPPER_Y : RED_HOPPER_Y;
	    
	    // Find rectangular difference between the robot and the hopper
            double dx = hopperX - botPose.getX();
            double dy = hopperY - botPose.getY();

            // Fetch Target Heading
            double targetAngleRad = Math.atan2(dy, dx);
            double currentAngleRad = botPose.getRotation().getRadians();

            // Find quickest path
            double error = targetAngleRad - currentAngleRad;
            while(error > Math.PI) error -= 2*Math.PI;
            while(error < -Math.PI) error += 2*Math.PI;

            // Find omega based on a turnPID & target angle direction and dist
            if(Math.abs(Math.toDegrees(error)) > deadbandDeg){
                omega = turnPID.calculate(currentAngleRad, targetAngleRad);
                omega = Math.max(Math.min(omega,3.0),-3.0);
            } else {
                turnPID.reset();
            }

            if(autoButton.getAsBoolean() && Math.abs(Math.toDegrees(error)) <= deadbandDeg){
                if (autoIntakeLightsEnabled) {lights.setColor(79, 52, 235);};

                // Find distance using the hypotenuse of the difference
                double distanceFeet = Math.hypot(dx, dy)*3.28084;

                double distanceFactor = Math.min(distanceFeet/SHOOT_DISTANCE,1.0);
                // Might need to change this formula later
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

        } else {
            System.out.println("No limelight data available and no fallback. Doing nothing.");
            if (autoIntakeLightsEnabled) {lights.setColor(235, 52, 52);};
        }

	    // Apply Change to Drivetrain control
        drivetrain.setControl(m_request.withVelocityX(0).withVelocityY(0).withRotationalRate(omega));
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(m_request.withRotationalRate(0));
        intake.stop();
        outtakeAngle.stop();
    }

    @Override
    public boolean isFinished(){ return false; }

    private Pose2d getAverageBotPose(){
        double sumX=0, sumY=0, sumCos=0, sumSin=0;
        int count=0;

        try (ExecutorService executor = Executors.newVirtualThreadPerTaskExecutor()) {
            for(LimelightConfig cam : LIMELIGHTS){
                executor.submit(() -> {
                    if(LimelightHelpers.getTV(cam.name)){
                        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(cam.name);

                        // Don't do anything if it doesn't return a pose
                        if(pose==null) continue;

                        // Calculate Average Robot Pose
                        sumX += pose.getX(); sumY += pose.getY();
                        double rot = pose.getRotation().getRadians();
                        sumCos += Math.cos(rot); sumSin += Math.sin(rot);
                        // Add Camera as working Camera
                        count++;
                    }});
                }
            }

        if(count==0) if (autoIntakeLightsEnabled) {lights.setColor(235, 52, 52);}; return null;

        double avgX = sumX/count;
        double avgY = sumY/count;
        // Find average rotation of limelights
        double avgRot = Math.atan2(sumSin,sumCos);

	    // Return with type Pose2d (X, Y, Rotation)
        return new Pose2d(avgX,avgY,new Rotation2d(avgRot));
        }
    }
