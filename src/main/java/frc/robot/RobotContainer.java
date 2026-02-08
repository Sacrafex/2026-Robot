// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Uses REVLib 2026.0.0, CTRE Phoenix v6 26.1.0, WPILib-New-Commands 1.0.0, REVLib 2026.0.0

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.TrackCode;
import frc.robot.commands.EstimatePose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterLinearActuator;
import frc.robot.subsystems.WebServer;
import frc.robot.subsystems.Elevator;
@SuppressWarnings("unused")
public class RobotContainer {

    private final Intake intake = new Intake(27, 26);
    // When using limit switches, the channels are on the RIO directly on the left and I think the text to the right is the channel
    private final ShooterLinearActuator ShooterLinearActuator = new ShooterLinearActuator(27, 0, 1);
    private final CANdleSubsystem lights = new CANdleSubsystem(0);
    private final WebServer webServer = new WebServer();
    private final Elevator elevator = new Elevator(50, 30);
    private final XboxController joystick0 = new XboxController(0);
    private final XboxController joystick1 = new XboxController(1);

    private final double SpeedReduction = 0.5;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Autonomous Commands

        NamedCommands.registerCommand("AutoAlignAndShoot",new AimAndShoot(drivetrain,intake,ShooterLinearActuator,lights,() -> true,joystick0));

        NamedCommands.registerCommand("ZeroRobotBase",drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        NamedCommands.registerCommand("LimelightSync",new EstimatePose(lights,drivetrain,AimAndShoot.LIMELIGHTS).withTimeout(5.0));

        autoChooser = AutoBuilder.buildAutoChooser("Taxi");SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {

        // Joystick0 - Main Driver Controller

        new Trigger(() -> joystick0.getRightBumper()).whileTrue(new AimAndShoot(drivetrain,intake,ShooterLinearActuator,lights,() -> true,joystick0));

        new Trigger(() -> joystick0.getLeftTriggerAxis() > 0.25).whileTrue(new TrackCode(drivetrain, joystick0));

        //new Trigger(() -> joystick0.getLeftTriggerAxis() >= 0.2).whileTrue(new AimAndShoot(drivetrain,intake,ShooterLinearActuator,lights,() -> true,joystick0));

        //new Trigger(() -> joystick0.getRightTriggerAxis() >= 0.2).whileTrue(new AimAndShoot(drivetrain,intake,ShooterLinearActuator,lights,() -> true,joystick0));
        
        new Trigger(() -> joystick0.getStartButton()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        new Trigger(() -> joystick0.getLeftBumper()).whileTrue(new EstimatePose(lights,drivetrain,AimAndShoot.LIMELIGHTS));
        
        new Trigger(() -> joystick0.getRightTriggerAxis() > 0.05).whileTrue(intake.run(() -> intake.set(-joystick0.getRightTriggerAxis()))).onFalse(intake.runOnce(intake::stop));

        new Trigger(() -> joystick0.getPOV() == 0).whileTrue(elevator.run(() -> elevator.set(0.5))).onFalse(elevator.runOnce(elevator::stop));

        new Trigger(() -> joystick0.getPOV() == 180).whileTrue(elevator.run(() -> elevator.set(-0.5))).onFalse(elevator.runOnce(elevator::stop));

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick0.getLeftY() * MaxSpeed * SpeedReduction)
                    .withVelocityY(-joystick0.getLeftX() * MaxSpeed * SpeedReduction)
                    .withRotationalRate(-joystick0.getRightX() * MaxAngularRate * SpeedReduction)
                    )
                );
                    
        // Joystick1 - Operator Controller

        //new Trigger(() -> joystick1.getLeftTriggerAxis() > 0.25).whileTrue(new TrackCode(drivetrain, joystick1));

        new Trigger(() -> joystick1.getLeftBumper()).whileTrue(new EstimatePose(lights,drivetrain,AimAndShoot.LIMELIGHTS));

        // Idle & Telemetry
                    
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
