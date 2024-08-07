package frc.robot.util;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Telemetry {
    private List<WeakReference<Object>> loggables;

    public static interface Loggable {
        public default Map<String, Object> getLoggedFields() {
            Map<String, Object> fields = new HashMap<>();

            for (Field field : this.getClass().getDeclaredFields()) {
                try {
                    fields.put(field.getName(), field.get(this));
                } catch (IllegalAccessException err) {
                    System.out.println("Unreachable error: " + err);
                }
            }

            return fields;
        }
    }
}
