// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  PhotonPipelineResult result;

  public CameraSubsystem(PhotonCamera camera) {
    m_camera = camera;
  }

  @Override
  public void periodic() {
    //result = m_camera.getLatestResult();
  }

  public PhotonPipelineResult getCameraResult(){
    return result;
  }

}
