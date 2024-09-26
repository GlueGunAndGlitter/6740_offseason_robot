// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class AprilTagVision {

    // create photonvision camera 
    PhotonCamera aprilTagsCamera = new PhotonCamera("AprilTags-Camera");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, aprilTagsCamera, robotToCam);

    public AprilTagVision() {
        
    }

    /* 
     get the wanted aprilTag on locion on the photonvisin dashbord
     if it dose not see aperilTag it will retern -1 
    */
    public int seesSpecificAprilTag(int aprilTag_ID, PhotonPipelineResult result) {
        if (result.hasTargets()) {
            var targets = result.getTargets();

            for (int i = 0; i < targets.size(); i++) {
                if (targets.get(i).getFiducialId() == aprilTag_ID) {
                    return i;
                }
            }
        }
        return -1;
    }

    /*
     get the forward distance from the camera to whanted aprilTag 
     if the camera dose not see aprilTag it will return 10 
    */
    public double distanceFromAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID, result)).getBestCameraToTarget().getX();
        } else {
            return 10;
        }
    }

    /*
     get the horizontal destance from the cameera to wanted aprilTag 
     if it dose not see aprilTags it will return 0
    */
    public double distanceFromTheMiddleOfTheAprilTag(int aprilTag_ID) {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            return targets.get(seesSpecificAprilTag(aprilTag_ID, result)).getBestCameraToTarget().getY();
        } else {
            return 0;
        }
    }

    // chack if the camera see aprilTag
    public boolean seesAprilTags() {
        var result = aprilTagsCamera.getLatestResult();
        if (result.hasTargets()) {
            return true;
        } else {
            return false;
        }    
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}