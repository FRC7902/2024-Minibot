// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;

  private final PIDController drivePID = new PIDController(1, 0, 0);
  private double initialPositionX;
  private double initialPositionY;
  private double angle;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double distance) {
    m_driveSubsystem = drive;
    targetDistance = distance;
    m_driveSubsystem.resetEncoders();
    drivePID.setTolerance(0.01, 1);
    //addRequirements(drive);
    
    initialPositionX = m_driveSubsystem.getDisplacementX();
    initialPositionY = m_driveSubsystem.getDisplacementY();
    angle = convertRange(m_driveSubsystem.getHeading());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initialPositionX = m_driveSubsystem.getDisplacementX();
    initialPositionY = m_driveSubsystem.getDisplacementY();
    angle = convertRange(m_driveSubsystem.getHeading());


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;

    if((45 < angle && angle < 135) || (225 < angle && angle < 315)){
      speed = drivePID.calculate(m_driveSubsystem.getDisplacementY(), initialPositionY + (targetDistance * sin(angle)));

      if(sin(angle) < 0){
        m_driveSubsystem.driveRaw(-speed);
      }else{
        m_driveSubsystem.driveRaw(speed);
      }

    }else{
      speed = drivePID.calculate(m_driveSubsystem.getDisplacementX(), initialPositionX + (targetDistance * cos(angle)));

      if(cos(angle) < 0){
        m_driveSubsystem.driveRaw(-speed);
      }else{
        m_driveSubsystem.driveRaw(speed);
      }

    }

    SmartDashboard.putNumber("Current Pos", m_driveSubsystem.getDisplacementX());
    SmartDashboard.putNumber("Target Pos", targetDistance);

  }

  public double cos(double angle){
    return Math.cos(Math.toRadians(angle));
  }

  public double sin(double angle){
    return Math.sin(Math.toRadians(angle));
  }

  public double convertRange(double angle){
    if(angle < 0){
      return angle + 360;
    }else{
      return angle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}