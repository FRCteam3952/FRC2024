package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.NetworkTablesConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class NetworkTablesUtil {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    public static final NetworkTable MAIN_ROBOT_TABLE = INSTANCE.getTable(NetworkTablesConstants.MAIN_TABLE_NAME);
    private static final Map<String, GenericPublisher> publishers = new HashMap<>();
    private static final Map<String, GenericSubscriber> subscribers = new HashMap<>();
    private static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /**
     * Gets the NetworkTablesConstants Instance being used by the program
     *
     * @return {@link NetworkTableInstance} used
     */
    public static NetworkTableInstance getNTInstance() {
        return INSTANCE;
    }

    /**
     * Returns the table reference from NetworkTablesConstants
     *
     * @param tableName The name of the table
     * @return {@link NetworkTable} corresponding
     */
    public static NetworkTable getTable(String tableName) {
        return INSTANCE.getTable(tableName);
    }

    public static void setLimelightPipeline(int pipeline) {
        NetworkTable table = INSTANCE.getTable("limelight");
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public static int getLimeLightPipeline() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("getpipe").getNumber(1).intValue();
    }

    public static float getLimeLightErrorX() {
        NetworkTable table = INSTANCE.getTable("limelight");
        if (getLimeLightPipeline() == 1) {
            return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[1].floatValue() - 160.0f;
        } else {
            return table.getEntry("tx").getNumber(0.0).floatValue() * 5.369f;
        }

    }

    public static float getLimelightTX() {
        return getEntry("limelight", "tx").getNumber(0.0).floatValue();
    }

    public static float getLimeLightErrorY() {
        NetworkTable table = INSTANCE.getTable("limelight");
        if (getLimeLightPipeline() == 1) {
            return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[2].floatValue() - 120.0f;
        } else {
            return table.getEntry("ty").getNumber(0.0).floatValue() * 5.2516f;
        }
    }

    public static float getLimeLightArea() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[3].floatValue();
    }

    public static float getConeOrientation() {
        NetworkTable table = INSTANCE.getTable("limelight");
        return table.getEntry("llpython").getNumberArray(new Number[]{0, 0, 0, 0})[0].floatValue();
    }

    // Gets key from keyboard
    public static String getKeyString() {
        NetworkTable table = INSTANCE.getTable("robogui");
        return table.getEntry("key_string").getString("default");
    }

    // Gets key from keyboard
    public static int getKeyInteger() {
        NetworkTable table = INSTANCE.getTable("robogui");
        return table.getEntry("key_int").getNumber(0).intValue();
    }

    /**
     * Returns the current robot pose according to AprilTags on Jetson, in meters since that's what they want. The rotation is really the gyro's rotation, since we know that the gyro is accurate.
     *
     * @return A {@link Translation2d} representing the robot's pose ([x, y, radians])
     */
    public static ArrayList<DistanceAndAprilTagDetection> getJetsonAprilTagPoses() {
        if (!jetsonHasPose()) {
            return new ArrayList<>();
        }

        NetworkTable table = INSTANCE.getTable("jetson");
        double[] readTags = table.getEntry("apriltags_pose").getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // The jetson outputs a list with 8 elements: tag id, followed by (x, y, z) of tag, followed by Quaternion (x, y, z, w)
        if (readTags.length % 8 != 0) {
            System.out.println("Error: bad tag array");
            return new ArrayList<>();
        }

        ArrayList<DistanceAndAprilTagDetection> poses = new ArrayList<>(readTags.length / 8);
        for (int i = 0; i < readTags.length; i += 8) {
            int tagId = (int) readTags[i + 0];
            Optional<Pose3d> fieldRelTagPoseOpt = TAG_FIELD_LAYOUT.getTagPose(tagId);
            if (fieldRelTagPoseOpt.isEmpty()) {
                System.out.println("No tag id " + tagId + " is on the field, skipping");
                continue;
            }
            Pose3d originToTag = fieldRelTagPoseOpt.get();
            System.out.println("pose of tag: " + originToTag);
            Translation3d pose = new Translation3d(readTags[i + 1], readTags[i + 2], readTags[i + 3]);
            Quaternion q = new Quaternion(readTags[i + 4], readTags[i + 5], readTags[i + 6], readTags[i + 7]);
            Pose3d tagOriginPose = CoordinateSystem.convert(new Pose3d(pose, new Rotation3d(q)), Util.APRILTAGS_COORD_SYSTEM, CoordinateSystem.NWU()); // a pose where the tag is treated as the origin.
            System.out.println("tag origin pose: " + tagOriginPose);
            System.out.println("tag angle: " + tagOriginPose.getRotation().getY());
            var a = originToTag.minus(new Pose3d());
            System.out.println("a: " + a);
            Pose3d finalPose = originToTag.plus(tagOriginPose.minus(new Pose3d()));

            if (checkRequestedPoseValues(finalPose)) {
                poses.add(new DistanceAndAprilTagDetection(finalPose, tagOriginPose.getTranslation().getDistance(new Translation3d())));
            }
        }

        return poses;
    }

    /**
     * Whether the given pose seems "reasonable"
     *
     * @param pose The given pose
     * @return True if the pose can reasonably be kept, false otherwise.
     */
    private static boolean checkRequestedPoseValues(Pose3d pose) {
        return true;
    }

    public static boolean jetsonHasPose() {
        NetworkTableEntry entry = getEntry("jetson", "apriltags_pose"); // ????? wdym "resource leak"
        return entry.getDoubleArray(new double[]{1.0}).length != 1;
    }

    /**
     * Gets whether the robot is on the blue alliance, according to the Driver Station.
     *
     * @return True if on blue, false if on red. If alliance is not present, will default to true.
     */
    public static boolean getIfOnBlueTeam() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get().equals(Alliance.Blue);
        }
        System.out.println("Alliance not present!");
        DriverStation.reportError("Alliance not present!", false);
        return true;
    }

    /**
     * Returns the entry reference from NetworkTablesConstants
     *
     * @param tableName Name of the table
     * @param entryName Name of the entry
     * @return {@link NetworkTableEntry} corresponding
     */
    public static NetworkTableEntry getEntry(String tableName, String entryName) {
        return getTable(tableName).getEntry(entryName);
    }

    public static GenericPublisher getPublisher(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = publishers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newPublisher = entry.getTopic().genericPublish(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true));
        publishers.put(path, newPublisher);
        return newPublisher;
    }

    public static GenericSubscriber getSubscriber(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = subscribers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newSubscriber = entry.getTopic().genericSubscribe(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        subscribers.put(path, newSubscriber);
        return newSubscriber;
    }

    public static void getConnections() {
        for (ConnectionInfo connection : INSTANCE.getConnections()) {
            System.out.println("Connection: Using version " + connection.protocol_version + ", ID: " + connection.remote_id + ", IP: " + connection.remote_ip + ", last update: " + connection.last_update);
        }
        if (INSTANCE.getConnections().length == 0) {
            System.out.println("NO CONNECTIONS FOUND");
        }
        System.out.println("END CONNECTIONS LIST \n\n\n\n");
    }

    /**
     * Use in conjunction w/ latencyTest() in network_tables.py to test latency.
     * (Also good example code)
     */
    public static void latencyTesterPeriodicRun() {
        var trajectorySub = getSubscriber("test", "test");
        final var EMPTY = new double[]{};

        TimestampedDoubleArray tsDA = new TimestampedDoubleArray(NetworkTablesJNI.now(), trajectorySub.getLastChange(), trajectorySub.getDoubleArray(EMPTY));
        var timeDiff = (tsDA.timestamp - tsDA.serverTime) / 1000;
        if (timeDiff > 1000) {
            System.out.println(timeDiff - 1000);
        }
    }

    public record DistanceAndAprilTagDetection(Pose3d fieldRelativePose, double distanceFromRobot) {
    }
}
