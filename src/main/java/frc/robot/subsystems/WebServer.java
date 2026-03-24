package frc.robot.subsystems;

import java.io.*;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import fi.iki.elonen.NanoHTTPD;
import frc.robot.Constants;
import frc.robot.commands.GoAndShoot;
import frc.robot.commands.TrackCode;
import frc.robot.commands.AimAndShoot;


public class WebServer extends SubsystemBase {
    
            private static CommandSwerveDrivetrain drivetrain;
            private static Shooter shooter;
            private static AimAndShoot AimAndShoot;
            private static final Map<String, Double> dashboardValues = new ConcurrentHashMap<>();
            private static final Map<String, String> autoChooser = new ConcurrentHashMap<>();
            private static volatile String selectedAuto = "Taxi";
        
            static {
                autoChooser.put("Taxi", "Taxi");
                autoChooser.put("Still", "Still");
                autoChooser.put("Right", "Right");
                autoChooser.put("Middle", "Middle");
                autoChooser.put("Left", "Left");
                autoChooser.put("MiddleSpinR", "MiddleSpinR");
                autoChooser.put("MiddleSpinL", "MiddleSpinL");
            }
        
            public static void putNumber(String key, double value) {
                dashboardValues.put(key, value);
            }
        
            public static double getNumber(String key, double defaultVal) {
                return dashboardValues.getOrDefault(key, defaultVal);
            }
        
            public static String getSelectedAuto() {
                return selectedAuto;
            }
        
            public static Map<String, String> getAutos() {
                return autoChooser;
            }
        
            private final DashboardServer server;
        
            public WebServer(CommandSwerveDrivetrain drivetrain, Shooter shooter, AimAndShoot AimAndShoot) {
            WebServer.drivetrain = drivetrain;
            WebServer.shooter = shooter;
            WebServer.AimAndShoot = AimAndShoot;
            server = new DashboardServer(Constants.WebServer.portNumber);
            try {
                server.start(5000, false);
                System.out.println("Dashboard: http://" + getLocalIp() + ":"+Constants.WebServer.portNumber+"/");
            } catch (IOException e) {
                stopServer();
            }
        }
    
        @Override
        public void periodic() {}
    
        public void stopServer() {
            if (server != null) server.stop();
        }
    
        private String getLocalIp() {
            try {
                return InetAddress.getLocalHost().getHostAddress();
            } catch (Exception e) {
                return "127.0.0.1";
            }
        }
    
        private static class DashboardServer extends NanoHTTPD {
    
            public DashboardServer(int port) {
                super(port);
            }
    
            @Override
            public Response serve(IHTTPSession session) {
                String uri = session.getUri();
    
            if ("/data".equals(uri)) {
                StringBuilder json = new StringBuilder("{");
                boolean first = true;
                for (var e : dashboardValues.entrySet()) {
                    if (!first) json.append(",");
                    json.append("\"").append(e.getKey()).append("\":").append(e.getValue());
                    first = false;
                }

                double distance = AimAndShoot.distance;
                json.append(",\"distance\":").append(distance);
                json.append(",\"speeds\":[");
                for (double d : Constants.Trajectory.speeds) {
                    json.append(d).append(",");
                }
                if (Constants.Trajectory.speeds.length > 0) {
                    json.setLength(json.length() - 1);
                }
                json.append("}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }
    
            if ("/autos".equals(uri)) {
                StringBuilder json = new StringBuilder("{");
                json.append("\"selected\":\"").append(selectedAuto).append("\",");
                json.append("\"options\":{");
                boolean first = true;
                for (var e : autoChooser.entrySet()) {
                    if (!first) json.append(",");
                    json.append("\"").append(e.getKey()).append("\":\"").append(e.getValue()).append("\"");
                    first = false;
                }
                json.append("}}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/rotations".equals(uri)) {
                StringBuilder json = new StringBuilder("{");
                double drivebaserot = drivetrain.getRotation3d().getAngle()*(180/Math.PI);
                json.append("\"degree\":").append(drivebaserot);
                json.append("}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/setAuto".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                String auto = p.getOrDefault("auto", List.of("DoNothing")).get(0);
                if (autoChooser.containsValue(auto)) selectedAuto = auto;
                return newFixedLengthResponse("OK");
            }

            if ("/setErrorCorrection".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                String eCValue = p.get("value").get(0);
                Constants.Trajectory.errorCorrectionMultiplier = Double.parseDouble(eCValue)/50;
                return newFixedLengthResponse("OK");
            }

            if ("/set".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                if (!p.containsKey("target") || !p.containsKey("value") || !p.containsKey("time")) {
                return newFixedLengthResponse("Missing parameters");
                }
                String setTarget = p.get("target").get(0);
                double setValue = Double.parseDouble(p.get("value").get(0));
                double setTime = Double.parseDouble(p.get("time").get(0));
                if ("shooterSpeedRotations".equals(setTarget)) {
                CommandScheduler.getInstance().schedule(Commands.run(() -> shooter.matchRotations(setValue), shooter).withTimeout(setTime).andThen(shooter::stop));
                }
                return newFixedLengthResponse("OK");
            }

            if ("/setPID".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                if (!p.containsKey("kPvar") || !p.containsKey("kIvar") || !p.containsKey("kDvar") || !p.containsKey("target")) {
                    return newFixedLengthResponse(Response.Status.BAD_REQUEST, NanoHTTPD.MIME_PLAINTEXT, "Missing kP, kI, or kD");
                }
                try {
                    double kP = Double.parseDouble(p.get("kPvar").get(0));
                    double kI = Double.parseDouble(p.get("kIvar").get(0));
                    double kD = Double.parseDouble(p.get("kDvar").get(0));
                    String target = p.get("target").get(0);

                    switch (target) {
                        case "ShooterPID":
                        Shooter.shooterPID.setPID(kP, kI, kD);
                        break;
                        case "GoAndShoot":
                        GoAndShoot.omegaPID.setPID(kP, kI, kD);
                        break;
                        case "TrackCodePID":
                        TrackCode.codePID.setPID(kP, kI, kD);
                        break;
                    }

                    return newFixedLengthResponse("OK");
                } catch (Exception e) {
                    return newFixedLengthResponse("Issue Occurred: " + e);
                }
            }

            if ("/requestpid".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                String setTarget = p.get("target").get(0);
                StringBuilder json = new StringBuilder("{");
                switch (setTarget) {
                    case "ShooterPID":
                    json.append("\"kP\":").append(Shooter.shooterPID.getP()).append(",");
                    json.append("\"kI\":").append(Shooter.shooterPID.getI()).append(",");
                    json.append("\"kD\":").append(Shooter.shooterPID.getD());
                    break;
                    case "TrackCodePID":
                    json.append("\"kP\":").append(TrackCode.codePID.getP()).append(",");
                    json.append("\"kI\":").append(TrackCode.codePID.getI()).append(",");
                    json.append("\"kD\":").append(TrackCode.codePID.getD());
                    break;
                    case "GoAndShoot":
                    json.append("\"kP\":").append(GoAndShoot.omegaPID.getP()).append(",");
                    json.append("\"kI\":").append(GoAndShoot.omegaPID.getI()).append(",");
                    json.append("\"kD\":").append(GoAndShoot.omegaPID.getD());
                    break;
                    default:
                    return newFixedLengthResponse("Error: Not a Valid Request");
                }
                json.append("}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/speeds".equals(uri)) {
                StringBuilder json = new StringBuilder("[");
                for (double speed : Constants.Trajectory.speeds) {
                    json.append(speed).append(",");
                }
                if (Constants.Trajectory.speeds.length > 0) {
                    json.setLength(json.length() - 1);
                }
                json.append("]");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/updateSpeeds".equals(uri)) {
                Map<String, List<String>> params = session.getParameters();
                try {
                    int index = Integer.parseInt(params.get("index").get(0));
                    double speed = Double.parseDouble(params.get("speed").get(0));
                    if (index >= 0 && index < Constants.Trajectory.speeds.length) {
                        Constants.Trajectory.speeds[index] = speed;
                        if (AimAndShoot != null) {
                            AimAndShoot.notifySpeedChange();
                        }
                        return newFixedLengthResponse("OK");
                    } else {
                        return newFixedLengthResponse(Response.Status.BAD_REQUEST, NanoHTTPD.MIME_PLAINTEXT, "Index out of bounds");
                    }
                } catch (Exception e) {
                    return newFixedLengthResponse(Response.Status.BAD_REQUEST, NanoHTTPD.MIME_PLAINTEXT, "Invalid parameters");
                }
            }

            try (InputStream is = getClass().getClassLoader().getResourceAsStream("dashboard.html")) {
                if (is == null) return newFixedLengthResponse("dashboard.html wasn't found.");
                return newFixedLengthResponse(Response.Status.OK, "text/html",
                        new String(is.readAllBytes(), StandardCharsets.UTF_8));
            } catch (IOException e) {
                return newFixedLengthResponse("Error");
            }
        }
    }
}