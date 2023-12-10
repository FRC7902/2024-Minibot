// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetAngle;
  private double trueTarget;
  private final PIDController turnPID = new PIDController(0.102, 2.04, 0.001275);

  private boolean isAdditive;
  private double initialAngle;
  private int direction;
  /** Creates a new TurnToAngleB. */
  public TurnToAngle(DriveSubsystem drive, double angle, boolean IsAdditive) {

    m_driveSubsystem = drive;
    targetAngle = angle;
    isAdditive = IsAdditive;
    turnPID.setTolerance(0.01, 1);


    initialAngle = convertRange(m_driveSubsystem.getHeading());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = convertRange(m_driveSubsystem.getHeading());

    double trueAngle = initialAngle - targetAngle;

    trueAngle = convertRange(modAngle(trueAngle));
    
    if(isAdditive){
      trueTarget = Math.round(convertRange(modAngle(targetAngle + initialAngle)));
    }else{
      trueTarget = targetAngle;
    }

    if(trueAngle < 180){
      direction = 1;
    }else{
      direction = 1;
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;

    SmartDashboard.putNumber("Target angle", trueTarget);

    if(trueTarget > 340 || trueTarget < 20){
      speed = turnPID.calculate(m_driveSubsystem.getHeading(), trueTarget);
      m_driveSubsystem.turn(direction * speed);

    }else{
      speed = turnPID.calculate(convertRange(m_driveSubsystem.getHeading()), convertRange(trueTarget));
      m_driveSubsystem.turn(direction * speed);

    }
  }

  public double convertRange(double angle){
    if(angle < 0){
      return angle + 360;
    }else{
      return angle;
    }
  }

  public double modAngle(double angle){
    return Math.IEEEremainder(angle, 360);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}