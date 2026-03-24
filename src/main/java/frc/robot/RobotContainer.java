// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Uses REVLib 2026.0.0, CTRE Phoenix v6 26.1.0, WPILib-New-Commands 1.0.0, PathplannerLib 2026.1.2

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.EstimatePose;
import frc.robot.commands.GoAndShoot;
import frc.robot.commands.TrackCode;
import frc.robot.generated.TunerConstants_New;
import frc.robot.generated.TunerConstants_Vulcan;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WebServer;
@SuppressWarnings("unused")
public class RobotContainer {

    public static final String DRIVEBASE_TYPE = Constants.Drivebase.DRIVEBASE_TYPE;
    public final Intake intake = new Intake(24, 38, "*");
    // Update CAN ID's. This is after I made changes to shotoer subsystem
    public final Shooter shooter = new Shooter(48,20,55);
    public final CANdleSubsystem lights = new CANdleSubsystem(0, "*", 200);
    public final XboxController joystick0 = new XboxController(0);
    public final XboxController joystick1 = new XboxController(1);
    
    public static double SpeedReduction = Constants.Drivebase.SpeedReduction;
    private final double DEADBAND_VALUE = Constants.Drivebase.DEADBAND_VALUE;

    // NOT CHANGEABLE CONSTANTS
    public static double speedChange = 1;
    private double MaxSpeed = TunerConstants_New.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    public final CommandSwerveDrivetrain drivetrain = createDrivetrain();
    private final AimAndShoot aimAndShoot = new AimAndShoot(drivetrain, shooter, lights, () -> true, joystick0);
    private final WebServer webServer = new WebServer(drivetrain, shooter, aimAndShoot);

    private static CommandSwerveDrivetrain createDrivetrain() {
    switch (DRIVEBASE_TYPE) {
        case "2026":
            return TunerConstants_New.createDrivetrain();
        case "vulcan":
            return TunerConstants_Vulcan.createDrivetrain();
        default:
            throw new IllegalArgumentException("Unknown drivebase: " + DRIVEBASE_TYPE);
        }
    }

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("AutoAlignAndShoot",new AimAndShoot(drivetrain,shooter,lights,() -> true,joystick0));

        NamedCommands.registerCommand("ZeroRobotBase",drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        NamedCommands.registerCommand("LimelightSync",new EstimatePose(lights,drivetrain,false));

        NamedCommands.registerCommand("FinishedAuto", new InstantCommand(() -> {System.out.println("Finished Auto");}));

        NamedCommands.registerCommand("SetColorRed", new InstantCommand(() -> lights.setColor(255, 0, 0)));

        NamedCommands.registerCommand("SetColorGreen", new InstantCommand(() -> lights.setColor(0, 255, 0)));

        NamedCommands.registerCommand("SetColorBlue", new InstantCommand(() -> lights.setColor(0, 0, 255)));

        NamedCommands.registerCommand("SetColorModeStar", new InstantCommand(() -> lights.shootingStarAnimation(255, 255, 255)));

        NamedCommands.registerCommand("SetColorModeBreath", new InstantCommand(() -> lights.breathingAnimation(255, 255, 255)));

        NamedCommands.registerCommand("SetColorOff", new InstantCommand(() -> lights.setColor(0, 0, 0)));

        autoChooser = AutoBuilder.buildAutoChooser("Taxi");
        configureBindings();CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        
    }

    private void configureBindings() {

        // Joystick0 - Main Driver Controller
        
        new Trigger(() -> joystick0.getStartButton()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
            () -> new SwerveRequest.FieldCentric()
                .withVelocityX(-MathUtil.applyDeadband(joystick0.getLeftY(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
                .withVelocityY(-MathUtil.applyDeadband(joystick0.getLeftX(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
                .withRotationalRate(-MathUtil.applyDeadband(joystick0.getRightX(), DEADBAND_VALUE) * MaxAngularRate * SpeedReduction * speedChange)
        ));
                            
        // Joystick1 - Operator Controller

        new Trigger(() -> joystick1.getPOV() == 90).onTrue(new InstantCommand(() -> speedChange = 0.3)).onFalse(new InstantCommand(() -> speedChange = 1.0));
        
        new Trigger(() -> joystick1.getPOV() == 180).whileTrue(new TrackCode(drivetrain, joystick1, lights, true).alongWith(new InstantCommand(() -> lights.shootingStarAnimation(0, 255, 0)))).onFalse(new InstantCommand(() -> TeleopLights()));;
        
        new Trigger(() -> joystick1.getPOV() == 270).whileTrue(new GoAndShoot(drivetrain, shooter, lights));
        
        new Trigger(() -> joystick1.getPOV() == 360).whileTrue(new EstimatePose(lights,drivetrain,true).alongWith(new InstantCommand(() -> lights.shootingStarAnimation(0, 0, 255)))).onFalse(new InstantCommand(() -> TeleopLights()));;
        
        new Trigger(() -> joystick1.getLeftBumper()).whileTrue(new AimAndShoot(drivetrain,shooter,lights,() -> true,joystick1).alongWith(new InstantCommand(() -> lights.shootingStarAnimation(255, 0, 0)))).onFalse(new InstantCommand(() -> TeleopLights()));
        
        new Trigger(() -> joystick1.getRightBumper()).whileTrue(intake.run(() -> intake.timedArmDown())).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick1.getLeftTriggerAxis() > 0.2).whileTrue(intake.run(() -> intake.setIntake(joystick1.getLeftTriggerAxis()))).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick1.getRightTriggerAxis() > 0.05).whileTrue(shooter.run(() -> shooter.runRotationsAsController(joystick1.getRightTriggerAxis()*0.5))).onFalse(shooter.runOnce(shooter::stop));

        new Trigger(() -> joystick1.getBButton()).whileTrue(intake.run(() -> intake.setIntake(1))).onFalse(intake.runOnce(intake::stop));

        new Trigger(() -> joystick1.getAButton()).whileTrue(intake.run(() -> intake.setIntake(-1))).onFalse(intake.runOnce(intake::stop));

        // Idle & Telemetry
                    
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        String selected = WebServer.getSelectedAuto();

        Command zeroCommand = new InstantCommand(() -> {
            drivetrain.seedFieldCentric();
            System.out.println("Zeroed Robot.");
        });

        if (selected == null || selected.isEmpty()) {
            System.out.println("No auto selected, doing nothing.");
            return zeroCommand;
        } else {
            Command autoCommand = AutoBuilder.buildAuto(selected);
            return new SequentialCommandGroup(
                zeroCommand,
                autoCommand
            );
        }
    }

    public void initLights() {
        lights.breathingAnimation(255, 255, 255);
    }

    public void TeleopLights() {
        lights.fireAnimation();
    }

    public void disabledLights() {
        lights.breathingAnimation(255, 255, 255);
    }
}