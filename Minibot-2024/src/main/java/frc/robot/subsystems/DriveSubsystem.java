// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftA = new CANSparkMax(DriveConstants.LeftMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_leftB = new CANSparkMax(DriveConstants.LeftMotorCAN[1], MotorType.kBrushless);
  private final CANSparkMax m_rightA = new CANSparkMax(DriveConstants.RightMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_rightB = new CANSparkMax(DriveConstants.RightMotorCAN[1], MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftA, m_leftB);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightA, m_rightB);

  private final DifferentialDrive m_drive = new DifferentialDrive(left, right);

  private final Encoder m_leftEncoder = new Encoder(DriveConstants.LeftEncoderCAN[0], DriveConstants.LeftEncoderCAN[1]);
  private final Encoder m_rightEncoder = new Encoder(DriveConstants.RightEncoderCAN[0], DriveConstants.RightEncoderCAN[1]);

  private final AnalogGyro m_gyro = new AnalogGyro(DriveConstants.GyroCAN);

  //SIMULATION:
  private DifferentialDriveOdometry m_odometry;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private AnalogGyroSim m_gyroSim;
  public DifferentialDrivetrainSim m_driveTrainSim;
  private Field2d m_fieldSim;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_leftEncoder.setDistancePerPulse(DriveConstants.WheelDiameterMeters * Math.PI / DriveConstants.EncoderTicksPerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.WheelDiameterMeters * Math.PI / DriveConstants.EncoderTicksPerPulse);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance(), 
      new Pose2d(5.0, 5.0, new Rotation2d())
      );

      m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDualCIMPerSide, 
        KitbotGearing.k10p71, 
        KitbotWheelSize.kSixInch, 
        null
        );

    m_fieldSim = new Field2d();
    SmartDashboard.putData("Field", m_fieldSim);

    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    m_gyroSim = new AnalogGyroSim(m_gyro);

    left.setInverted(true);
    right.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance()
    );

    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    //connect the motors to update the drivetrain
    
    m_driveTrainSim.setInputs(
      left.get() * RobotController.getBatteryVoltage(),
      right.get() * RobotController.getBatteryVoltage()
    );
    
    m_driveTrainSim.update(0.02);

    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());

    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_driveTrainSim.getHeading().getDegrees());

  }

  public void driveArcade(double xForward, double zRotation){
    m_drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power){
    left.set(power);
    right.set(power);
  }

  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public double getPosition(){
    return m_rightEncoder.getDistance();
  }

  public void turn(double power){
    left.set(power);
    right.set(-power);
  }

}
