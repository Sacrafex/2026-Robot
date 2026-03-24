package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class Constants {

    public class Drivebase {
        public static final String DRIVEBASE_TYPE = "2026"; // vulcan, 2026
        public static final double SpeedReduction = 0.5;
        public static final double DEADBAND_VALUE = 0.1;
    }
    
    public class Limelight {

        public static class LimelightConfig {
        public String name; public double x; public double y; public double z; public double roll; public double pitch; public double yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw)
            {this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;}
        }

        // Meters
        public static final LimelightConfig[] LIMELIGHTS = {
            new LimelightConfig("limelight-front",0.01125,-0.005,0.095,Math.toRadians(0),Math.toRadians(58),Math.toRadians(0))
        };

        public static final boolean ENABLE_LIMELIGHT_LIGHTS = true;

    }

    public class Trajectory {

        public static double[] distances = { 0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,  10.0, 20 }; // METERS
        public static double[] speeds =    { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 }; // RPM

        public static double MAX_MOTOR_SPEED_SHOOTER = 6000;

        public static double MOTOR_A_BELT_MULTIPLER = 1;
        public static double MOTOR_B_SPEED_MULTIPLER = 0.8;
        public static double MOTOR_C_SPEED_MULTIPLER = 1;

        public static double errorCorrectionMultiplier = 1;
    }

    public class AimAndShoot {

        public static final boolean autoShoot = true;
        public static final double SHOOT_DEADBAND = 5;

        public static final double kP = 0.1;

        public static final double BLUE_HOPPER_X = 4.626;
        public static final double BLUE_HOPPER_Y = 8.069;
        public static final double RED_HOPPER_X = 11.834;
        public static final double RED_HOPPER_Y = 8.069;

        public static final double SHOOTER_OFFSET_DEGREES = 90;

        public static boolean autoIntakeLightsEnabled = true;
        public static boolean allowAlignDriveControl = true;
    }

    public class EstimatePose {
        public static final double interval = 0.3;
    }

    public class GoAndShoot {
        public static PIDController omegaPID = new PIDController(0.1, 0.0, 0.05);
        public static PIDController drivePIDx = new PIDController(0.1, 0.0, 0.05);
        public static PIDController drivePIDy = new PIDController(0.1, 0.0, 0.05);

        public static final boolean autoShoot = true;
        public static final double SHOOT_DEADBAND = 1;

        public class BLUE_ALLIANCE {
            public static final double targetX = 4.626;
            public static final double targetY = 8.069;
            public static final double presetRotation = 90;
        }
        public class RED_ALLIANCE {
            public static final double targetX = 11.834;
            public static final double targetY = 8.069;
            public static final double presetRotation = -90;
        }
    }

    public class TrackCode {
        public static final double MAX_VEL = 8.0;
        public static final double MAX_OMEGA = 10;

        public static final double GUESS_REDUCTION_MPL_DEG = 0.9;

        public static final double omega_zero = 0.00001;

        public static PIDController codePID = new PIDController(2, 3, 0.000001);

        public static final double interval = 0.05;
    }

    public class WebServer {
        public static int portNumber = 5809;
    }
}
