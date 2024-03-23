package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

public final class AprilTagHandler {
    private double[] previousReading = new double[] {};

    /**
     * Returns the current robot pose according to AprilTags on Jetson, in meters since that's what they want. The rotation is really the gyro's rotation, since we know that the gyro is accurate.
     *
     * @return A {@link Translation2d} representing the robot's pose ([x, y, radians])
     */
    public ArrayList<RobotPoseAndTagDistance> getJetsonAprilTagPoses() {
        if (!jetsonHasPose()) {
            return new ArrayList<>();
        }

        double[] readTags = NetworkTablesUtil.getAprilTagEntry(); // The jetson outputs a list with 8 elements: tag id, followed by (x, y, z) of tag, followed by Quaternion (x, y, z, w)
        if(readTags.length == previousReading.length) {
            boolean flag = true;
            for(int i = 0; i < readTags.length; i++) {
                if(Math.abs(previousReading[i] - readTags[i]) > 0.0001) { // all values being the same implies that the table hasn't been updated
                    flag = false;
                    break;
                }
            }
            if(flag) {
                // System.out.println("no AT readout");
                return new ArrayList<>();
            }
        }

        previousReading = readTags;

        if (readTags.length % 8 != 0) {
            System.out.println("Error: bad tag array");
            return new ArrayList<>();
        }

        ArrayList<RobotPoseAndTagDistance> poses = new ArrayList<>(readTags.length / 8);
        for (int i = 0; i < readTags.length; i += 8) {
            int tagId = (int) readTags[i + 0];
            Optional<Pose3d> fieldRelTagPoseOpt = Util.TAG_FIELD_LAYOUT.getTagPose(tagId);
            if (fieldRelTagPoseOpt.isEmpty()) {
                System.out.println("No tag id " + tagId + " is on the field, skipping");
                continue;
            }
            Pose3d originToTag = fieldRelTagPoseOpt.get();
            // System.out.println("pose of tag: " + originToTag);
            Translation3d pose = new Translation3d(readTags[i + 1], readTags[i + 2], readTags[i + 3]);
            Quaternion q = new Quaternion(readTags[i + 4], readTags[i + 5], readTags[i + 6], readTags[i + 7]);
            Pose3d tagOriginPose = CoordinateSystem.convert(new Pose3d(pose, new Rotation3d()), Util.JETSON_APRILTAGS_COORD_SYSTEM, CoordinateSystem.NWU()); // a pose where the tag is treated as the origin.
            Pose2d newPose = DriveTrainSubsystem.fixPose(tagOriginPose.toPose2d());
            // System.out.println("tag origin pose: " + tagOriginPose);
            // System.out.println("new pose: " + newPose);
            // System.out.println("tag angle: " + tagOriginPose.getRotation().getY());
            //var a = originToTag.minus(new Pose3d());
            // System.out.println("a: " + a);
            Pose2d finalPose = originToTag.toPose2d().plus(newPose.minus(new Pose2d()));
            //System.out.println("final pose: " + finalPose);
            if (checkRequestedPoseValues(finalPose)) {
                poses.add(new RobotPoseAndTagDistance(finalPose, tagOriginPose.getTranslation().getDistance(new Translation3d())));
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
    
    private static boolean checkRequestedPoseValues(Pose2d pose) {
        return true;
    }

    public static boolean jetsonHasPose() {
        return NetworkTablesUtil.getAprilTagEntry().length != 1;
    }

    public record RobotPoseAndTagDistance(Pose2d fieldRelativePose, double tagDistanceFromRobot) {
    }
}
