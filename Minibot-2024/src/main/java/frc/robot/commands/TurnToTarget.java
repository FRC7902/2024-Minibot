// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends SequentialCommandGroup {
  /** Creates a new TurnToTarget. */
  public TurnToTarget(CameraSubsystem camera, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double targetAngle = 0;
    PhotonPipelineResult targetResult = camera.getCameraResult();

    if(targetResult.hasTargets()){
      targetAngle = targetResult.getBestTarget().getYaw();
      
    }

    SmartDashboard.putNumber("AprilTag target angle", targetAngle);
    addCommands(
      //new TurnToAngle(drive, -targetAngle, true)
    );
  }
}
