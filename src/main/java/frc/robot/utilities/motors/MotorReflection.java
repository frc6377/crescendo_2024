package frc.robot.utilities.motors;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.hal.CANAPITypes.CANManufacturer;
import edu.wpi.first.hal.CANStreamMessage;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStreamOverflowException;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

public class MotorReflection implements AutoCloseable {
  private static final int MESSAGE_BUFFER_SIZE = 255; // This is an arbitary number

  private final int CTRE_MOTOR_CAN_STREAM;
  private final int REV_MOTOR_CAN_STREAM;

  public MotorReflection() {
    CANStreamFilter ctreFilterBuilder = new CANStreamFilter();
    ctreFilterBuilder
        .setDeviceType(CANDeviceType.kMotorController)
        .setManufactor(CANManufacturer.kCTRE);
    CTRE_MOTOR_CAN_STREAM =
        CANJNI.openCANStreamSession(
            ctreFilterBuilder.getFilter(), ctreFilterBuilder.getMask(), MESSAGE_BUFFER_SIZE);

    CANStreamFilter revFilterBuilder = new CANStreamFilter();
    revFilterBuilder
        .setDeviceType(CANDeviceType.kMotorController)
        .setManufactor(CANManufacturer.kCTRE);
    REV_MOTOR_CAN_STREAM =
        CANJNI.openCANStreamSession(
            revFilterBuilder.getFilter(), revFilterBuilder.getMask(), MESSAGE_BUFFER_SIZE);
  }

  public void unusedMotorDetection() {
    List<Integer> unusedREVIds = MotorFactory.getRevIds();
    unusedREVIds.removeAll(Arrays.stream(scanForIds(REV_MOTOR_CAN_STREAM)).boxed().toList());

    List<Integer> unusedCTREIds = MotorFactory.getCTREIds();
    unusedCTREIds.removeAll(Arrays.stream(scanForIds(CTRE_MOTOR_CAN_STREAM)).boxed().toList());

    for (Integer i : unusedREVIds) {
      DriverStation.reportError("REV Motor ID " + i + " is unused, check all CAN ids", false);
    }

    for (Integer i : unusedCTREIds) {
      DriverStation.reportError("CTRE Motor ID " + i + " is unused, check all CAN ids", false);
    }
  }

  public Motor[] discoverMotorsOnNetwork() {
    int[] ctreIDs = scanForIds(CTRE_MOTOR_CAN_STREAM);
    int[] revIDs = scanForIds(REV_MOTOR_CAN_STREAM);
    Motor[] allDiscoveredMotors = new Motor[ctreIDs.length + revIDs.length];

    for (int i = 0; i < ctreIDs.length; i++) {
      allDiscoveredMotors[i] = new Motor(CANManufacturer.kCTRE, ctreIDs[i]);
    }
    for (int i = 0; i < revIDs.length; i++) {
      allDiscoveredMotors[i] = new Motor(CANManufacturer.kREV, revIDs[i]);
    }
    return allDiscoveredMotors;
  }

  private static int[] scanForIds(int streamId) {
    try {
      CANStreamMessage[] messageBuffer = new CANStreamMessage[MESSAGE_BUFFER_SIZE];
      for(int i =0; i < MESSAGE_BUFFER_SIZE; i++){
        messageBuffer[i] = new CANStreamMessage();
      }
      int numberOfMessages = CANJNI.readCANStreamSession(streamId, messageBuffer, streamId);
      HashSet<Integer> idsSet = new HashSet<>();
      for (int i = 0; i < numberOfMessages; i++) {
        idsSet.add(extractDeviceNumberFromMessageID(messageBuffer[i].messageID));
      }

      // Convert from HashSet to int[]
      int numberOfids = idsSet.size();
      int[] idsArr = new int[numberOfids];
      Iterator<Integer> idsIterator = idsSet.iterator();

      for (int i = 0; i < numberOfids; i++) {
        idsArr[i] = idsIterator.next();
      }

      return idsArr;
    } catch (CANStreamOverflowException overflow) {
      // This should be unreachable because the stream can only buffer the same number of messages
      // as the local buffer can.
      throw new RuntimeException(overflow);
    }
  }

  private static int extractDeviceNumberFromMessageID(int messageID) {
    // Extracts the first 6 bits
    return messageID & 0x3F;
  }

  /** Handles constuctring a filter for a openCANStream. */
  private class CANStreamFilter {
    private int manufactor = -1;
    private int type = -1;

    public int getMask() {
      int manufactorSet = (manufactor == -1) ? 0 : 1;
      int deviceTypeSet = (type == -1) ? 0 : 1;
      return (0x1F << 24) * deviceTypeSet | (0xFF << 16) * manufactorSet;
    }

    public int getFilter() {
      return manufactor << 16 | type << 24;
    }

    public CANStreamFilter setManufactor(CANManufacturer manufactor) {
      // Sanitizes the number to be 8 bits
      this.manufactor = manufactor.id & 0xFF;
      return this;
    }

    public CANStreamFilter setDeviceType(CANDeviceType deviceType) {
      // Sanitizes the number to be 5 bits
      this.manufactor = deviceType.id & 0x1F;
      return this;
    }
  }

  public record Motor(CANManufacturer manufactor, int id) {}

  @Override
  public void close() {
    CANJNI.closeCANStreamSession(CTRE_MOTOR_CAN_STREAM);
    CANJNI.closeCANStreamSession(REV_MOTOR_CAN_STREAM);
  }
}
