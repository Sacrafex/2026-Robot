package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.UnpaidIntern;

public class Intake extends SubsystemBase {

    private final TalonFX intake;
    private final TalonFX arm;

    private double lastTime = 0.0;
    private final double interval = 0.3;

    private boolean targetIsUp = false;
    private boolean enabledScheduler = false;

    private static final double DEADBAND = 0;
    private static final double MAX_SPEED = 0.85;
    private static final double MIN_SPEED = -0.85;

    private static final double ARM_DOWN_MOVE = -0.87;
    private static final double ARM_UP_MOVE = 0.20;
    private static final double ARM_HOLD_STRONG = 0.01;

    private static final double STALL_CURRENT = 150.0;

    private static final int CURRENT_SAMPLES = 12;
    private final double[] currentBuf = new double[CURRENT_SAMPLES];
    private int currentIdx = 0;

    private static final int REQUIRED_STALL_LOOPS = 5;
    private int stallLoops = 0;

    private boolean armStalled = false;

    public Intake(int intakeMotorCANId, int armMotorCANId, String canbus) {
        intake = new TalonFX(intakeMotorCANId, canbus);
        arm = new TalonFX(armMotorCANId, canbus);
        UnpaidIntern.zeroEncoder(intake);
        UnpaidIntern.zeroEncoder(arm);
    }

    public void setTargetState(boolean isUp) {
        if (isUp) {
            targetIsUp = true;
            UnpaidIntern.stop(intake);
        } else {
            targetIsUp = false;
        }
    }

    public void setSchedulerStatus(boolean isEnabled) {
        if (isEnabled) {
            enabledScheduler = true;
        }
    }

    public void schedulerChangesCall() {
        if (enabledScheduler && targetIsUp) {
            timedArmUp();
        }
    }

    public void bringIntakeDown() {
        targetIsUp = false;
        timedArmDown();
    }

    public void setIntake(double speed) {
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;
        UnpaidIntern.setPercentWithDeadband(intake, speed, DEADBAND);
        WebServer.putNumber("IntakeRoller", speed);
    }

    private double filteredCurrent() {
        currentBuf[currentIdx] = arm.getStatorCurrent().getValueAsDouble();
        currentIdx = (currentIdx + 1) % CURRENT_SAMPLES;
        double sum = 0;
        for (double v : currentBuf) sum += v;
        return sum / CURRENT_SAMPLES;
    }

    private boolean armIsStalledFiltered() {
        if (filteredCurrent() >= STALL_CURRENT) {
            stallLoops++;
        } else {
            stallLoops = 0;
        }
        return stallLoops >= REQUIRED_STALL_LOOPS;
    }

    public void checkStall() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTime >= interval) {
            if (armIsStalledFiltered() == true) {
                armStalled = true;
            } else {
                armStalled = false;
            }
        }
    }

    public void timedArmUp() {
        if (!armStalled) {
            UnpaidIntern.setPercentWithDeadband(arm, ARM_UP_MOVE, DEADBAND);
        } else {
            UnpaidIntern.setPercentWithDeadband(arm, ARM_HOLD_STRONG, DEADBAND);
        }
        checkStall();
    }


    public void timedArmDown() {
        if (!armStalled) {
            UnpaidIntern.setPercentWithDeadband(arm, -ARM_DOWN_MOVE, DEADBAND);
        } else {
            UnpaidIntern.setPercentWithDeadband(arm, -ARM_HOLD_STRONG, DEADBAND);
        }
        checkStall();
    }

    public void stop() {
        setIntake(0);
        UnpaidIntern.stop(intake);
        UnpaidIntern.stop(arm);
        armStalled = false;
        stallLoops = 0;
        RobotContainer.speedChange = 1;
    }

    public void kill() {
        UnpaidIntern.killMotor(intake);
        UnpaidIntern.killMotor(arm);
        armStalled = false;
        stallLoops = 0;
    }

    public TalonFX getMotorA() { return intake; }
    public TalonFX getMotorB() { return arm; }
}