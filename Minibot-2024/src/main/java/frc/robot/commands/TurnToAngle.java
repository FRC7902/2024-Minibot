// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveSubsystem driveSubsystem, double angle) {
    
    super(
        // The controller that the command will use

    //ku = 0.17
    //tu = 0.1

    //kp = 0.6 * ku
    //ki = (1.2 * ku) / tu
    //kd = ku * tu * 0.075

        new PIDController(0.102, 2.04, 0.001275),
        // This should return the measurement
        driveSubsystem::getHeading,
        // This should return the setpoint (can also be a constant)
        angle,
        // This uses the output
        output -> {
          driveSubsystem.turn(output);
        });

    getController().setTolerance(0.1, 1);

    SmartDashboard.putNumber("angle 2", driveSubsystem.getHeading());

    getController().enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
