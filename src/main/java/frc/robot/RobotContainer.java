// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class RobotContainer {

  private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_driveSubsystem.driveArcade(
          m_driverController.getRawAxis(1), 
          m_driverController.getRawAxis(0)),
          m_driveSubsystem
      ));    
  }

  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}