package frc.robot.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTagVision {

    // First camera
    PhotonCamera aprilTagsCamera = new PhotonCamera("AprilTags_front_Camera");
    // Second camera
    PhotonCamera secondCamera = new PhotonCamera("Second-Camera");

    // Field layout with AprilTag locations
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Transform for the first camera
    Transform3d robotToCam1 = new Transform3d(new Translation3d(0.3, 0.27, 0.23), new Rotation3d(Math.toRadians(50),0,0)); // Camera 1 location
    // Transform for the second camera
    Transform3d robotToCam2 = new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0,0,0)); // Camera 2 location

    // Pose estimators for each camera
    PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, aprilTagsCamera, robotToCam1);
    PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, secondCamera, robotToCam2);

    public AprilTagVision() {
        // Constructor
    }

    // Method to combine or choose the best pose estimate from both cameras
    public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        // Set reference pose for both estimators
        poseEstimator1.setReferencePose(prevEstimatedRobotPose);
        poseEstimator2.setReferencePose(prevEstimatedRobotPose);

        // Get the pose estimates from both cameras
        Optional<EstimatedRobotPose> pose1 = poseEstimator1.update();
        Optional<EstimatedRobotPose> pose2 = poseEstimator2.update();

        // If both cameras have pose estimates, combine them or choose the best one
        if (pose1.isPresent() && pose2.isPresent()) {
            // Example strategy: Average the two poses (could be refined based on your needs)
            Pose2d averagePose = new Pose2d(
                pose1.get().estimatedPose.toPose2d().getTranslation().plus(pose2.get().estimatedPose.toPose2d().getTranslation()).div(2),
                pose1.get().estimatedPose.toPose2d().getRotation()
            );
             return Optional.of(averagePose);
            } 
        // If only one camera has a valid estimate, return it
         else if (pose1.isPresent()) {
            return Optional.of(pose1.get().estimatedPose.toPose2d());
        } else if (pose2.isPresent()) {
            return Optional.of(pose2.get().estimatedPose.toPose2d());
        }
        return Optional.empty();

    }
}
