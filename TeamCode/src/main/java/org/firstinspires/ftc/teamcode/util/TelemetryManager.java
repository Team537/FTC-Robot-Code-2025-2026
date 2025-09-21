package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class TelemetryManager {
    private static Telemetry telemetry;
    private static final Map<String, Object> values = new HashMap<>();

    // Call this once in your OpMode init()
    public static void init(Telemetry tel) {
        telemetry = tel;
    }

    public static void put(String key, Object value) {
        values.put(key, value);
    }

    // Call this once per loop (e.g. in runOpMode or a Scheduler)
    public static void update() {
        if (telemetry == null) return;
        telemetry.clear();

        for (Map.Entry<String, Object> entry : values.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }

        telemetry.update();
    }
}
