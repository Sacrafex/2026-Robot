# Libraries and Dependencies

To maintain stable version control we will NOT update Libraries within the WPILib Vendor Dependencies Tab. Instead,
we will retain the same version among robots including UnpaidIntern.java (A Helper File created by KJ). Our
current versions for required Dependencies are the following:

CTRE-Phoenix (v6) - 26.1.0
PathPlannerLib - 2026.1.2
REVLib - 2026.0.0
WPILib-New-Commands - 1.0.0

Additional Dependencies can be added without breaking the base. Again, do not update or change these versions as
the helpers require these versions. 

(For Lead Programmer) - You may update these libraries if need be (please make sure to migrate all code and build
when doing so.)

# UnpaidIntern.java (Helper Script)

COMMANDS:

1) Deadband
   - deadband(value, band)
     Returns 0 if input is inside the deadband range.
     Example: deadband(0.03, 0.05) -> 0.0

   - setPercentWithDeadband(motor, input, band)
     Sets motor output (percent or voltage) with deadband applied.
     It does ignore killed motors.

2) Zeroing
   - zeroEncoder(motor)
     Stores current encoder position as zero.

   - getZeroedPosition(motor)
     Returns encoder position relative to zero.

   - getActualPosition(motor)
     Returns raw encoder value.

3) Limits
   - setLimits(motor, min, max)
     Constrains future positions/velocities to a range.
     If you pass a zeroed position (getZeroedPosition) to toPosition, the limits are relative to zero.
     If you pass an actual encoder value (getActualPosition) to toPosition, the limits are relative to the raw encoder.

   - clearLimits(motor)
     Removes limits for the motor.

4) Position Control
   - toPosition(motor, position)
     Moves motor to a target position respecting limits and kill status.

   - atPosition(motor, target, tolerance)
     Returns true if motor is within tolerance of target.

5) Velocity Control
   - toVelocity(motor, velocity)
     Sets motor velocity, respecting limits and kill status.

6) Kill Switch
   - killMotor(motor)
     Immediately stops the motor and disables future commands until revived.

   - reviveMotor(motor)
     Re-enables a previously killed motor.

   - isMotorKilled(motor)
     Returns true if motor is currently killed.

7) Stop
   - stop(motor)
     Immediately stops the motor (without disabling future commands).

NOTES:
- Supports up to 32 motors (TalonFX or SparkMax) by default (can be changed *I used an array for simplicity)
- Automatically handles offsets and zeroing per motor
- All methods are safe to call on motors that may have been killed

# TunerConstants (Robot Specific Drivebase Information) - CTRE Swerve Generator

When creating a swerve drive, we will use the Swerve Generator provided by CTRE in Phoenix Tuner (APP *TOP LEFT HAMBURGER MENU -> MECHANISMS -> SWERVE GENERATOR).
We will only use the TunerConstants as the default configuration does NOT come with autos. We will simply copy the TunerConstants.java file and replace the old
version with the newly generated one. This allows other important code that handles AUTOs to stay present in the project.

NOTES:
Make sure the modules spin counter-clockwise in orientation to the floor (IF ROBOT IS ON IT'S SIDE, IT WILL BE OPPOSITE)

# PathPlanner

To use pathplanner, open the application and select the base folder of this project when opening. This should show a Tests.auto and a SimplePath.path. You
may edit these however you would like to test the robot. When calling commands in an auto, you need to create a NamedCommand within RobotContainer.java that
should look close to the following:

NamedCommands.registerCommand("examplePrintMSG", Commands.runOnce(()->{System.out.println("This is an example message");}));

(This example should be on line 53 of the true template within RobotContainer() {})

When the robot is ran, SmartDashboard should open with options for which Auto to use.

NOTES:

The files for the autos can be found in ./src/main/deploy/pathplanner/autos & ./src/main/deploy/pathplanner/paths relative to the base project directory.

# Creating a Basic Intake

A subsystem is simply a group of motors (or single motor) that represents a part of the robot. Subsystems are used to define
a motor and it's functions that can be called. For example, here is a simple Intake.java

"
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UnpaidIntern;

public class Intake extends SubsystemBase {
    // Motor ID (Find in Phoenix Tuner)
    private final TalonFX motor = new TalonFX(18);

    // Deadband threshold handled in command
    private static final double DEADBAND = 0;

    public Intake() {
        // zero encoder (not needed for simple intake, but included)
        UnpaidIntern.zeroEncoder(motor);
    }

    // Sets motor speed with deadband
    public void set(double speed) {
        UnpaidIntern.setPercentWithDeadband(motor, speed, DEADBAND);
    }

    // Stops the motor immediately
    public void stop() {
        UnpaidIntern.stop(motor);
    }

    // Kill motor
    public void kill() {
        UnpaidIntern.killMotor(motor);
    }

    // Getter for the motor (needed for TeleopIntake)
    public TalonFX getMotor() {
        return motor;
    }
}
"

So far we can set the speed of the motor to a percent when a deadband along with methods to stop the motor instantly
and kill the motor (so it cannot start after we killed it *We can optionally make a revive method using UnpaidIntern)

Next, let's create a basic command to link the controller value to the motor. Heres our /commands/TeleopIntake.java

"
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.UnpaidIntern;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.DoubleSupplier;

public class TeleopIntake extends Command {
    private final Intake intake;
    private final TalonFX motor;
    private final DoubleSupplier speedSupplier;
    private static final double DEADBAND = 0.05;

    // Constructor accepts a DoubleSupplier (e.g. joystick axis)
    public TeleopIntake(Intake intake, DoubleSupplier speedSupplier) {
        this.intake = intake;
        this.motor = intake.getMotor();
        this.speedSupplier = speedSupplier;
        addRequirements(intake);
        System.out.println("[INFO] TeleopIntake initialized");
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble(); // Gets current Value
        UnpaidIntern.setPercentWithDeadband(motor, speed, DEADBAND);
    }

    @Override
    public void end(boolean interrupted) {
        UnpaidIntern.stop(motor);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until canceled
    }
}
"

Finally, we can implement our RobotContainer control link to the command:

"Please check ./RobotContainer.java for the example as it is too large to fit here"

NOTE: Do not modify the imports as this is only an example (Do not just copy and paste as not everything was defined here and it would cut off other vital parts.)

TeleopIntake takes a DoubleSupplier to have variable speed.
Each loop, it reads joystick.getRawAxis(3) (R2) for the motor speed.
UnpaidIntern.setPercentWithDeadband() handles deadband automatically.