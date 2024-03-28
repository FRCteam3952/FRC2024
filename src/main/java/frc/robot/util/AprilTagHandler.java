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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

public final class AprilTagHandler {
    private double[] previousReading = new double[] {};

    /**
     * Returns the current robot pose according to AprilTags on Jetson, in meters since that's what they want. The rotation is really the gyro's rotation, since we know that the gyro is accurate.
     *
     * @return A {@link Translation2d} representing the robot's pose ([x, y, radians])
     */
    public ArrayList<RobotPoseAndTagDistance> getJetsonAprilTagPoses() {
        if (!jetsonHasPose() && !DriverStation.isAutonomous()) { // we don't want to update ourselves mid-auton
            return new ArrayList<>();
        }

        double[] readTags = NetworkTablesUtil.getAprilTagEntry(); // The jetson outputs a list with 8 elements: tag id, followed by (x, y, z) of tag, followed by Quaternion (x, y, z, w)
        if(readTags.length == previousReading.length) {
            boolean flag = true;
            for(int i = 0; i < readTags.length; i++) {
                if(Math.abs(previousReading[i] - readTags[i]) > 0.0001) { // all values being the same implies that the table hasn't been updated - the values will fluctuate if a tag is being seen.
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

        if (readTags.length % 8 != 0) { // this shouldn't happen
            System.out.println("Error: bad tag array");
            return new ArrayList<>();
        }

        ArrayList<RobotPoseAndTagDistance> poses = new ArrayList<>(readTags.length / 8);
        // System.out.println("received: " + readTags.length / 8 + " tags");
        for (int i = 0; i < readTags.length; i += 8) {
            int tagId = (int) readTags[i + 0];
            Optional<Pose3d> fieldRelTagPoseOpt = Util.TAG_FIELD_LAYOUT.getTagPose(tagId);
            if (fieldRelTagPoseOpt.isEmpty()) {
                System.out.println("No tag id " + tagId + " is on the field, skipping");
                continue;
            }
            Pose3d originToTag = fieldRelTagPoseOpt.get();
            // System.out.println("pose of tag: " + originToTag);
            Translation3d pose = new Translation3d(readTags[i + 1], readTags[i + 2], Math.cos(Math.toRadians(50)) * readTags[i + 3]); // the camera is tilted at ~50deg angle, so adjust our z (axis straight out of the camera lens) distance so we get just the horizontal component.
            Quaternion q = new Quaternion(readTags[i + 4], readTags[i + 5], readTags[i + 6], readTags[i + 7]); // we don't even really use this
            Pose3d tagOriginPose = CoordinateSystem.convert(new Pose3d(pose, new Rotation3d(q)), Util.JETSON_APRILTAGS_COORD_SYSTEM, CoordinateSystem.NWU()); // a pose where the tag is treated as the origin.
            Pose2d newPose = AprilTagHandler.fixPose(tagOriginPose.toPose2d());
            // System.out.println("tag origin pose: " + tagOriginPose);
            // System.out.println("new pose: " + newPose);
            // System.out.println("tag angle: " + tagOriginPose.getRotation().getY());
            //var a = originToTag.minus(new Pose3d());
            // System.out.println("a: " + a);
            Pose2d finalPose = originToTag.toPose2d().plus(newPose.minus(new Pose2d()));
            //System.out.println("final pose: " + finalPose);
            if (checkRequestedPoseValues(finalPose)) {
                poses.add(new RobotPoseAndTagDistance(finalPose, tagOriginPose.getTranslation().getDistance(new Translation3d()), tagId));
            }
        }

        return poses;
    }

    /**
     * Derived by build president isaac an, 2024 season (thank you)
     * Corrects the pose based of the rotation of the robot.
     * <p>
     * Since the pose returned by the jetson is relative to the camera's field of vision, the pose is affected by the robot's rotation.
     * This uses the robot's gyroscope (more accurate than the jetson's attempt at determining its own rotation) to get a field-relative position of the pose.
     * 
     * @param pose Pose to correct
     * @return The corrected pose, but keeping the original estimated rotation (inaccurate).
     */
    private static Pose2d fixPose(Pose2d pose) {
        double c = pose.getTranslation().getDistance(new Translation2d());
        double thi = Math.asin(pose.getY() / c);
        double y = c * Math.sin(RobotGyro.getRotation2d().getRadians() + thi);
        double x = c * Math.cos(RobotGyro.getRotation2d().getRadians() + thi);
    
        return new Pose2d(new Translation2d(x, y), pose.getRotation());
    }

    /**
     * Whether the given pose seems "reasonable"
     *
     * @param pose The given pose
     * @return True if the pose can reasonably be kept, false otherwise.
     */
    
    private static boolean checkRequestedPoseValues(Pose2d pose) {
        return pose.getX() > 0 && pose.getY() > 0 && pose.getX() < 16.764 && pose.getY() < 8.382;
    }

    public static boolean jetsonHasPose() {
        return NetworkTablesUtil.getAprilTagEntry().length != 1;
    }

    public record RobotPoseAndTagDistance(Pose2d fieldRelativePose, double tagDistanceFromRobot, int tagId) {
    }
}
