import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/** Validates production C++ replay output with AdvantageKit's actual reader. */
public final class AkitReplayOutputAcceptanceTest {
  @Test
  public void actualAdvantageKitReaderRecoversCppReplayInputCycles() throws Exception {
    String output = System.getProperty("akit.acceptance.output");
    if (output == null || output.isBlank()) {
      throw new IllegalArgumentException("akit.acceptance.output system property is required");
    }
    String expectedBackend = System.getProperty("akit.acceptance.backend", "phoenix");
    dumpRawReplaySamples(Path.of(output));

    WPILOGReader reader = new WPILOGReader(Path.of(output).toAbsolutePath().toString());
    reader.start();
    LogTable table = new LogTable(0);
    long[] timestamps = {1_000_000, 1_020_000, 1_040_000};
    double[] poseX = {1.0, 1.2, 1.4};
    double[] speedX = {0.25, 0.5, 0.0};
    for (int cycle = 0; cycle < timestamps.length; cycle++) {
      final double expectedPoseX = poseX[cycle];
      final double expectedSpeedX = speedX[cycle];
      boolean hasMoreCycles = reader.updateTable(table);
      if (cycle < timestamps.length - 1 && !hasMoreCycles) {
        throw new AssertionError("AdvantageKit ended before replay output cycle " + (cycle + 1));
      }
      if (cycle == timestamps.length - 1 && hasMoreCycles) {
        throw new AssertionError("AdvantageKit reported an unexpected fourth replay cycle");
      }
      if (table.getTimestamp() != timestamps[cycle]) {
        throw new AssertionError("cycle " + cycle + " timestamp=" + table.getTimestamp()
            + "; expected=" + timestamps[cycle]);
      }
      Map<String, LogTable.LogValue> values = table.getAll(false);
      assertValue(values, "/ReplayInputs/Drive/backend", "String",
          value -> expectedBackend.equals(value.getString("<missing>")));
      assertValue(values, "/ReplayInputs/Drive/pose", "DoubleArray",
          value -> {
            double[] pose = value.getDoubleArray();
            return pose.length == 3 && close(pose[0], expectedPoseX)
                && close(pose[1], -0.4);
          });
      assertValue(values, "/ReplayInputs/Drive/speeds", "DoubleArray",
          value -> {
            double[] speeds = value.getDoubleArray();
            return speeds.length == 3 && close(speeds[0], expectedSpeedX)
                && close(speeds[1], -0.1);
          });
    }
    System.out.println("PASS: AdvantageKit 26.0.2 WPILOGReader recovered all three timestamped "
        + "C++ production replay input cycles");
  }

  private static void dumpRawReplaySamples(Path path) throws Exception {
    DataLogReader raw = new DataLogReader(path.toAbsolutePath().toString());
    if (!raw.isValid() || !"AdvantageKit".equals(raw.getExtraHeader())) {
      throw new AssertionError("raw WPILib reader rejected C++ output log: " + path);
    }
    Map<Integer, String> names = new HashMap<>();
    for (DataLogRecord record : raw) {
      if (record.isControl()) {
        if (record.isStart()) {
          var start = record.getStartData();
          names.put(start.entry, start.name);
        }
        continue;
      }
      String name = names.get(record.getEntry());
      if (name == null) continue;
      if (name.equals("/Timestamp")) {
        System.out.println("RAW_DIAG /Timestamp recordTimestamp=" + record.getTimestamp()
            + " value=" + record.getInteger());
      } else if (name.equals("/ReplayInputs/Drive/pose")
          || name.equals("/ReplayInputs/Drive/speeds")) {
        System.out.println("RAW_DIAG " + name + " recordTimestamp=" + record.getTimestamp()
            + " value=" + Arrays.toString(record.getDoubleArray()));
      } else if (name.equals("/ReplayInputs/Drive/backend")) {
        System.out.println("RAW_DIAG " + name + " recordTimestamp=" + record.getTimestamp()
            + " value=" + record.getString());
      }
    }
  }

  private static boolean close(double actual, double expected) {
    return Double.isFinite(actual) && Math.abs(actual - expected) <= 1e-7;
  }

  @FunctionalInterface
  private interface ValueCheck {
    boolean matches(LogTable.LogValue value);
  }

  private static void assertValue(Map<String, LogTable.LogValue> values, String key,
      String type, ValueCheck check) {
    LogTable.LogValue value = values.get(key);
    if (value == null || !value.type.name().equals(type) || !check.matches(value)) {
      throw new AssertionError("AdvantageKit did not recover " + key + " as " + type
          + "; actual=" + (value == null ? "missing" : value.type)
          + "; keys=" + values.keySet());
    }
  }
}
