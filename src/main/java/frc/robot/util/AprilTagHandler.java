package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

import javax.swing.text.html.Option;
import java.util.List;
//import java.util.HashSet;

public final class AprilTagHandler {
    private final GenericPublisher skippedApriltagHandling = NetworkTablesUtil.getPublisher("robot", "skippedATHandling", NetworkTableType.kBoolean);

    private final List<Pose2d> previousAutoPositions = new ArrayList<>();

    private double[] previousReading = new double[] {};
    private double previousReadingTime = Timer.getFPGATimestamp();

    private final Field2d fieldTag = new Field2d();

    public AprilTagHandler() {
        SmartDashboard.putData("fieldTag", fieldTag);
    }

    private static final double c_camTranslation = DriveTrainSubsystem.cameraLocation.getDistance(new Translation2d());
    private static final double thi_camTranslation = Math.asin(DriveTrainSubsystem.cameraLocation.getY() / c_camTranslation);

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
            skippedApriltagHandling.setBoolean(flag);
            if(flag) { // if we don't get tag updates often enough due to 30fps on cam, we can supplement by checking the previousReadingTime
                // System.out.println("no AT readout");
                double dT = Timer.getFPGATimestamp() - previousReadingTime;
                System.out.println("dT in reading of " + dT);
                if(dT < 0.07) {
                    return new ArrayList<>();
                }
            }
        }

        previousReading = readTags;
        previousReadingTime = Timer.getFPGATimestamp();

        if (readTags.length % 8 != 0) { // this shouldn't happen
            System.out.println("Error: bad tag array");
            return new ArrayList<>();
        }

        ArrayList<RobotPoseAndTagDistance> poses = new ArrayList<>(readTags.length / 8);

        Rotation2d robotRotation = RobotGyro.getRotation2d();
        double theta = robotRotation.getRadians();
        double yTranslation = c_camTranslation * Math.sin(theta - thi_camTranslation);
        double xTranslation = c_camTranslation * Math.cos(theta - thi_camTranslation);
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
            Pose2d tagToCamPose = AprilTagHandler.fixPose(tagOriginPose.toPose2d());
            // System.out.println("tag origin pose: " + tagOriginPose);
            // System.out.println("new pose: " + newPose);
            // System.out.println("tag angle: " + tagOriginPose.getRotation().getY());
            //var a = originToTag.minus(new Pose3d());
            // System.out.println("a: " + a);
            var temp = tagToCamPose.minus(new Pose2d());
            System.out.println("tagToCamTranslation: " + temp);
            var originAs2d = originToTag.toPose2d();
            Pose2d camPoseFieldRel = new Pose2d(new Translation2d(originAs2d.getX() + temp.getX(), originAs2d.getY() + temp.getY()), new Rotation2d());// originToTag.toPose2d().plus(temp);
            //fieldTag.setRobotPose(tagToCamPose);

            System.out.println("using a robot rotation of: " + robotRotation + " to correct pose");

            Translation2d newTranslation = camPoseFieldRel.getTranslation().plus(new Translation2d(xTranslation, yTranslation));
            System.out.println("translating the pose by " + xTranslation + ", " + yTranslation);
            Pose2d estimatedRobotPose = new Pose2d(newTranslation, RobotGyro.getRotation2d());
            //System.out.println("final pose: " + finalPose);
            fieldTag.setRobotPose(estimatedRobotPose);
            if (checkRequestedPoseValues(estimatedRobotPose)) {
                poses.add(new RobotPoseAndTagDistance(estimatedRobotPose, tagOriginPose.getTranslation().getDistance(new Translation3d()), tagId));
            } else {
                System.out.println("rejected the pose of tag " + tagId + ", est'd loc of " + estimatedRobotPose);
            }
        }

        System.out.println("call to apriltaghandler gave: " + poses.size() + " poses.");
        return poses;
    }

    public Optional<Pose2d> averageAutoAimPose(int tagId) {
        // does not filter w/ wpilib filters.

        // 5 is an ARBITRARY VARIABLE
        // that SHOULD BE CHANGED
        // wait until we've collected some data to start moving
        final int dataPointsCollectedToStartMoving = 5;

        // update by adding another pose.
        // does getJetsonAprilTagPoses already filter? is that a problem?
        getJetsonAprilTagPoses()
                .stream()
                .filter((tag) -> tag.tagId() == tagId)
                .findFirst()
                .map(AprilTagHandler.RobotPoseAndTagDistance::fieldRelativePose)
                .filter(pose ->
                        // check if the pose is new: none of the old ones are equal to it.
                    previousAutoPositions.stream().noneMatch(
                            other ->
                                Math.abs(other.getTranslation().getX()    - pose.getTranslation().getX())    < 1E-4 &&
                                Math.abs(other.getTranslation().getY()    - pose.getTranslation().getY())    < 1E-4 &&
                                Math.abs(other.getRotation().getDegrees() - pose.getRotation().getDegrees()) < 1E-4
                    )
                )
                .filter(_e -> previousAutoPositions.size() < dataPointsCollectedToStartMoving)
                .map(previousAutoPositions::add);



        int newTotalNumPositions = previousAutoPositions.size();


        // this coould be empty, if we can't collect any data
        // e.g. apriltag finder not working, robot can't see any.
        Optional<Pose2d> averagePosition = previousAutoPositions
                .stream()
                // deconstruct into a double array, average them, then reconstruct into a Pose2d.
                // probably could be done better
                .map(pose -> new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()})
                // sum
                .reduce((e1, e2) -> new double[]{
                        e1[0] + e2[0],
                        e1[1] + e2[1],
                        e1[2] + e2[2],
                })
                // sum -> average
                .map(e -> new double[]{
                        e[0] / newTotalNumPositions,
                        e[1] / newTotalNumPositions,
                        e[2] / newTotalNumPositions,
                })
                .map(e -> new Pose2d(new Translation2d(e[0], e[1]), new Rotation2d(e[2])))
                // don't move if we haven't collected enough data.
                .filter(_e -> newTotalNumPositions >= dataPointsCollectedToStartMoving);

        return averagePosition;
    }

    public void resetAverageAutoAimPose() {
        previousAutoPositions.clear();
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
