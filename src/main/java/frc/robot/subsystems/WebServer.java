package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import fi.iki.elonen.NanoHTTPD;

import java.io.IOException;
import java.net.InetAddress;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class WebServer extends SubsystemBase {

    // Remember name and value
    private static final Map<String, Double> dashboardValues = new ConcurrentHashMap<>();

    public static void putNumber(String key, double value) {
        dashboardValues.put(key, value);
    }

    public static double getNumber(String key, double defaultVal) {
        return dashboardValues.getOrDefault(key, defaultVal);
    }

    private final DashboardServer server;

    public WebServer() {
        server = new DashboardServer(2839);
        try {
            server.start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
            System.out.println("Robot Dashboard Started: http://" + getLocalIp() + ":2839/");
        } catch (IOException e) {
            System.err.println("Failed to start webserver: " + e.getMessage());
            stopServer();
        }
    }

    @Override
    public void periodic() {}

    public void stopServer() {
        if (server != null) {
            server.stop();
            System.out.println("Webserver stopped.");
        }
    }

    private String getLocalIp() {
        try {
            return InetAddress.getLocalHost().getHostAddress();
        } catch (Exception e) {
            System.err.println("Failed to get local IP: " + e.getMessage());
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
            // Wait for a /data request in uri and build json
            if ("/data".equals(uri)) {
                String json = "{";
                boolean first = true;

                // for every entry in the dashboard values entry set add a json section
                for (Map.Entry<String, Double> entry : dashboardValues.entrySet()) {
                    if (!first) {
                        json += ",";
                    }
                    json += "\"" + entry.getKey() + "\":" + entry.getValue();
                    first = false;
                }

                json += "}";

                Response res = newFixedLengthResponse(json);
                res.addHeader("Content-Type", "application/json");
                return res;
            }

            String html = """
            <html>
              <head>
                <title>2839 Robot Dashboard</title>
                <script>
                  async function updateDashboard() {
                    const res = await fetch('/data');
                    const data = await res.json();
                    for (const key in data) {
                      const el = document.getElementById(key);
                      if (el) el.innerText = data[key];
                    }
                  }
                  setInterval(updateDashboard, 200);
                </script>
              </head>
              <body>
                <h1>Robot Dashboard</h1>
                <p>Distance from Hopper: <span id="DistancefromHopper">0</span></p>
                <p>Omega: <span id="Omega">0</span></p>
                <p>Applied Omega: <span id="AppliedOmega">0</span></p>
                <p>intakeSpeed: <span id="intakeSpeed">0</span></p>
                <p>targetShooterAngle: <span id="targetShooterAngle">0</span></p>
                <p>ShooterSpeed: <span id="ShooterSpeed">0</span></p>
              </body>
            </html>
            """;
            return newFixedLengthResponse(html);
        }
    }
}