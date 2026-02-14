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


public class WebServer extends SubsystemBase {
    
            private static CommandSwerveDrivetrain drivetrain;
            private static Shooter shooter;
            private static final Map<String, Double> dashboardValues = new ConcurrentHashMap<>();
            private static final Map<String, String> autoChooser = new ConcurrentHashMap<>();
            private static volatile String selectedAuto = "Taxi";
        
            static {
                autoChooser.put("Taxi", "Taxi");
                autoChooser.put("Still", "Still");
                autoChooser.put("Right", "Right");
                autoChooser.put("Middle", "Middle");
                autoChooser.put("Left", "Left");
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
        
            public WebServer(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
            WebServer.drivetrain = drivetrain;
            WebServer.shooter = shooter;
            server = new DashboardServer(5810);
            try {
                server.start(5000, false);
                System.out.println("Dashboard: http://" + getLocalIp() + ":5810/");
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
                System.out.println("/set parameters are missing");
                return newFixedLengthResponse("Missing parameters");
                }
                String setTarget = p.get("target").get(0);
                double setValue = Double.parseDouble(p.get("value").get(0));
                double setTime = Double.parseDouble(p.get("time").get(0));
                if ("shooterSpeedRotations".equals(setTarget)) {
                CommandScheduler.getInstance().schedule(Commands.run(() -> shooter.matchRotations(setValue), shooter).withTimeout(setTime));
                }
                return newFixedLengthResponse("OK");
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