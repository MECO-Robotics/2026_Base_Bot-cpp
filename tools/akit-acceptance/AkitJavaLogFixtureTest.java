import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Exports a typed Java AdvantageKit log fixture for the C++ replay acceptance test. */
public final class AkitJavaLogFixtureTest {
  @Test
  public void writeAdvantageKitLogWithDriverStationAndDriveStructs() throws Exception {
    String output = System.getProperty("akit.acceptance.javaOutput");
    if (output == null || output.isBlank()) {
      throw new IllegalArgumentException("akit.acceptance.javaOutput system property is required");
    }
    Path path = Path.of(output).toAbsolutePath();
    Files.createDirectories(path.getParent());
    Files.deleteIfExists(path);

    WPILOGWriter writer = new WPILOGWriter(
        path.toString(), WPILOGWriter.AdvantageScopeOpenBehavior.NEVER);
    writer.start();
    try {
      appendCycle(writer, 1_000_000, true, true, false, false, false, true, true, 1.0, 0.25);
      appendCycle(writer, 1_020_000, true, true, false, false, false, true, true, 1.2, 0.50);
      appendCycle(writer, 1_040_000, false, false, false, true, false, true, false, 1.4, 0.0);
    } finally {
      writer.end();
    }
    if (!Files.isRegularFile(path) || Files.size(path) == 0) {
      throw new AssertionError("AdvantageKit did not write the Java fixture: " + path);
    }
    System.out.println("JAVA_AKIT_FIXTURE=" + path);
  }

  private static void appendCycle(
      WPILOGWriter writer,
      long timestampUs,
      boolean enabled,
      boolean autonomous,
      boolean teleop,
      boolean testMode,
      boolean eStopped,
      boolean dsAttached,
      boolean fmsAttached,
      double poseX,
      double speedX) {
    LogTable table = new LogTable(timestampUs);
    table.put("DriverStation/Enabled", enabled);
    table.put("DriverStation/Autonomous", autonomous);
    table.put("DriverStation/Teleop", teleop);
    table.put("DriverStation/Test", testMode);
    table.put("DriverStation/EmergencyStop", eStopped);
    table.put("DriverStation/DSAttached", dsAttached);
    table.put("DriverStation/FMSAttached", fmsAttached);
    table.put("DriverStation/AllianceStation", 4);
    table.put("DriverStation/MatchTime", 135.0 - (timestampUs - 1_000_000) / 1_000_000.0);
    table.put("NetworkInputs/SmartDashboard/Auto Choices", "None");
    for (int stick = 0; stick < 6; stick++) {
      appendJoystick(table, stick);
    }
    table.put("DriveState/Pose", Pose2d.struct,
        new Pose2d(poseX, -0.4, Rotation2d.fromDegrees(15.0)));
    table.put("DriveState/Speeds", ChassisSpeeds.struct,
        new ChassisSpeeds(speedX, -0.1, 0.2));
    table.put("DriveState/ModuleStates", SwerveModuleState.struct,
        new SwerveModuleState[] {
            new SwerveModuleState(1.0 + speedX, Rotation2d.fromDegrees(10.0)),
            new SwerveModuleState(1.1 + speedX, Rotation2d.fromDegrees(20.0)),
            new SwerveModuleState(1.2 + speedX, Rotation2d.fromDegrees(30.0)),
            new SwerveModuleState(1.3 + speedX, Rotation2d.fromDegrees(40.0))
        });
    writer.putTable(table);
  }

  private static void appendJoystick(LogTable table, int stick) {
    String prefix = "DriverStation/Joystick" + stick + "/";
    boolean primary = stick == 0;
    table.put(prefix + "AxisValues", primary ? new float[] {0.25f, -0.5f} : new float[0]);
    table.put(prefix + "POVs", primary ? new int[] {90} : new int[0]);
    table.put(prefix + "ButtonValues", primary ? 5 : 0);
    table.put(prefix + "ButtonCount", primary ? 10 : 0);
    table.put(prefix + "Type", primary ? 1 : 0);
    table.put(prefix + "Xbox", primary);
    table.put(prefix + "Name", primary ? "Acceptance Fixture Pad" : "");
  }
}
