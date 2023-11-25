// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem driveSubsystem, double distance) {
    super(
        new PIDController(0.5, 0.5, 0.1),
        // This should return the measurement
        driveSubsystem::getPosition,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output -> driveSubsystem.driveRaw(output),
        driveSubsystem
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
