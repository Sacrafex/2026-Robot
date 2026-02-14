package frc.robot;

public class Constants {
    
    public class Limelight {

        public static class LimelightConfig {
        public String name; public double x; public double y; public double z; public double roll; public double pitch; public double yaw;
        LimelightConfig(String name, double x, double y, double z, double roll, double pitch, double yaw)
            {this.name=name; this.x=x; this.y=y; this.z=z; this.roll=roll; this.pitch=pitch; this.yaw=yaw;}
        }

        public static final LimelightConfig[] LIMELIGHTS = {
            new LimelightConfig("limelight-front",0.01125,-0.005,0.095,Math.toRadians(0),Math.toRadians(58),Math.toRadians(0))
        };

        public static final boolean ENABLE_LIMELIGHT_LIGHTS = true;

    }

    public class Trajectory {

        public static double[] distances = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 20};
        public static double[] speeds = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

        public static double MAX_MOTOR_SPEED_SHOOTER = 6000;

        public static double MOTOR_A_BELT_MULTIPLER = 1;
        public static double MOTOR_B_SPEED_MULTIPLER = 0.8;
        public static double MOTOR_C_SPEED_MULTIPLER = 1;

        public static double errorCorrectionMultiplier = 1;
    }
}
