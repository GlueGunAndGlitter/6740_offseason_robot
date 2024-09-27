// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;
import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class NoteVision {

    // create potonvision camera
    public static PhotonCamera notesCamera;

    public NoteVision() {

        // set photonvision camera
        notesCamera = new PhotonCamera("dsds");
    }

    // get the angel from the camera th the note if dose not see note return 0
    public double getRobotToNoteYaw() {
        var result = notesCamera.getLatestResult();
        if (!result.hasTargets())
            return 0.0;
        else
            return result.getBestTarget().getYaw();
    }

    // get if the camera see notes
    public boolean seesNote() {
        return notesCamera.getLatestResult().hasTargets();
    }

}