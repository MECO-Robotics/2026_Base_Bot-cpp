import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/** Reads a C++-generated WPILOG with the actual AdvantageKit 26.0.2 reader. */
public final class AkitReaderAcceptanceTest {
  @Test
  public void actualAdvantageKitReaderAcceptsCppLog() throws Exception {
    String input = System.getProperty("akit.acceptance.input");
    if (input == null || input.isBlank()) {
      throw new IllegalArgumentException("akit.acceptance.input system property is required");
    }
    Path path = Path.of(input).toAbsolutePath();
    if (!Files.isRegularFile(path) || Files.size(path) == 0) {
      throw new AssertionError("C++ WPILOG is missing or empty: " + path);
    }
    assertRawOutputRecords(path);

    WPILOGReader reader = new WPILOGReader(path.toString());
    reader.start();
    LogTable table = new LogTable(0);
    if (!reader.updateTable(table)) {
      throw new AssertionError("AdvantageKit did not expose the first cycle of the C++ log");
    }
    assertCycle(table, 1_000_000, "rev", 1.25);
    if (reader.updateTable(table)) {
      throw new AssertionError("AdvantageKit reported an unexpected third C++ log cycle");
    }
    assertCycle(table, 1_020_000, "rev", 1.5);

    Map<String, LogTable.LogValue> values = table.getAll(false);
    System.out.println(
        "PASS: AdvantageKit 26.0.2 WPILOGReader recovered both timestamped C++ fixture cycles"
            + "; final table keys=" + values.keySet() + "; source=" + path);
  }

  private static void assertRawOutputRecords(Path path) throws Exception {
    DataLogReader raw = new DataLogReader(path.toString());
    if (!raw.isValid() || !"AdvantageKit".equals(raw.getExtraHeader())) {
      throw new AssertionError("WPILib raw reader rejected C++ AdvantageKit header");
    }
    Map<Integer, String> names = new HashMap<>();
    Map<Integer, String> types = new HashMap<>();
    int outputSamples = 0;
    for (DataLogRecord record : raw) {
      if (record.isControl()) {
        if (record.isStart()) {
          var start = record.getStartData();
          names.put(start.entry, start.name);
          types.put(start.entry, start.type);
          System.out.println("RAW_DIAG start id=" + start.entry + " name=" + start.name
              + " type=" + start.type);
        }
        continue;
      }
      String name = names.get(record.getEntry());
      String type = types.get(record.getEntry());
      String value = "<unsupported>";
      if ("int64".equals(type)) value = Long.toString(record.getInteger());
      else if ("double".equals(type)) value = Double.toString(record.getDouble());
      else if ("string".equals(type)) value = record.getString();
      System.out.println("RAW_DIAG record id=" + record.getEntry() + " name=" + name
          + " type=" + type + " timestamp=" + record.getTimestamp() + " value=" + value);
      if (!"/ReplayOutputs/Drive/voltage".equals(name)) continue;
      if (!"double".equals(types.get(record.getEntry())))
        throw new AssertionError("C++ replay output voltage was not logged as double");
      double expected = record.getTimestamp() == 1_000_000 ? 6.0 :
          record.getTimestamp() == 1_020_000 ? -2.0 : Double.NaN;
      if (Double.compare(record.getDouble(), expected) != 0) {
        throw new AssertionError(
            "C++ raw replay output value/timestamp mismatch: time=" + record.getTimestamp()
                + ", value=" + record.getDouble());
      }
      outputSamples++;
    }
    if (outputSamples != 2) {
      throw new AssertionError("expected 2 raw C++ ReplayOutputs samples, got " + outputSamples);
    }
  }

  private static void assertCycle(
      LogTable table, long timestampUs, String backend, double poseX) {
    if (table.getTimestamp() != timestampUs) {
      throw new AssertionError(
          "timestamp mismatch: expected=" + timestampUs + ", actual=" + table.getTimestamp());
    }
    Map<String, LogTable.LogValue> values = table.getAll(false);
    assertValue(values, "/ReplayInputs/Drive/backend", "String",
        value -> backend.equals(value.getString("<missing>")));
    assertValue(values, "/ReplayInputs/Drive/pose/0", "Double",
        value -> Double.compare(value.getDouble(Double.NaN), poseX) == 0);
  }

  @FunctionalInterface
  private interface ValueCheck {
    boolean matches(LogTable.LogValue value);
  }

  private static void assertValue(
      Map<String, LogTable.LogValue> values, String key, String type, ValueCheck check) {
    LogTable.LogValue value = values.get(key);
    if (value == null || !value.type.name().equals(type) || !check.matches(value)) {
      throw new AssertionError(
          "AdvantageKit did not recover " + key + " as " + type
              + "; actual=" + (value == null ? "missing" : value.type)
              + "; available keys=" + values.keySet());
    }
  }
}
