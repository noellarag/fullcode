// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSub extends SubsystemBase {

  private final CANSparkMax m_leftMaster = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_leftFollower = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_rightMaster = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightFollower = new CANSparkMax(2, MotorType.kBrushed);

  private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMaster::set, m_rightMaster::set);

  private final static AHRS navx = new AHRS(SPI.Port.kMXP);

  private final Encoder m_leftEncoder = new Encoder(5, 6, false, EncodingType.k2X);
  private final Encoder m_rightEncoder = new Encoder(3, 4,true, EncodingType.k2X);
  boolean buttonPressedOnce;
  boolean buttonPressedTwice;
  
  // Creating my kinematics object: track width of 27 inches
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.52);

  DifferentialDriveOdometry m_odometry;


  DifferentialDriveWheelSpeeds wheelSpeeds;
  // Convert to wheel speeds

  ChassisSpeeds chassisSpeeds;

  double linearVelocity = 3.0;
  double angularVelocity = 6.28;




  /** Creates a new DrivetrainSub. */
  public DrivetrainSub() {
    
    setSmartLimit(50);
    m_rightMaster.setInverted(true);
    m_rightFollower.setInverted(true);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    buttonPressedOnce = false;
    buttonPressedTwice = false;


    m_leftEncoder.setDistancePerPulse(Constants.kDriveTickToMeters);
    m_rightEncoder.setDistancePerPulse(Constants.kDriveTickToMeters);
    
    
    resetEncoders();

    AutoBuilder.configureRamsete(this::getPose, this::resetOdometry, this::getChassisSpeeds,this::driveRobotRelative, new ReplanningConfig(),
    () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }, this);
  }

  @Override
  public void periodic() {
    m_odometry.update(navx.getRotation2d(), m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance());

    // This method will be called once per scheduler run
  }



  public void drive(double d, double e) {
    chassisSpeeds = new ChassisSpeeds(d * linearVelocity, 0.0, e * angularVelocity);
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    m_drivetrain.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public void setCoastMode() {
    m_leftMaster.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightMaster.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);
  }



  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void limitSpeed(double MaxSpeed, double MidSpeed, double LowSpeed, boolean button) {
    if (button) {
      if (buttonPressedTwice == true) {
        m_drivetrain.setMaxOutput(MaxSpeed);
        buttonPressedTwice = false;
      }
      else {
        if (buttonPressedOnce == true) {
          m_drivetrain.setMaxOutput(MidSpeed);
          buttonPressedOnce = false;
          buttonPressedTwice = true;
        }
        else {
          m_drivetrain.setMaxOutput(LowSpeed);
          buttonPressedOnce = true;
        }
      }
    }
  }

  public double get() {
    return m_leftMaster.get();
  }

  public double getRightEncoderPosition() {
    return m_rightEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getLeftEncoderPosition() {
    return m_leftEncoder.getDistance();
    //CHECK IF NEEDS TO BE INVERTED
  }

  public double getRightEncoderVelocity() {
    return m_rightEncoder.getRate();
  }

    public double getLeftEncoderVelocity() {
    return m_rightEncoder.getRate();
  }

  public double getTurnRate() {
    return navx.getRate();
  }
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(navx.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), getPose());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_leftFollower.setVoltage(leftVolts);
    m_rightFollower.setVoltage(rightVolts);    
    m_drivetrain.feed();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }

  public double getAverageEncoderVel() {
    return ((getLeftEncoderVelocity() + getRightEncoderVelocity()) / 2.0);
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_drivetrain.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    navx.reset();
  }

  public AHRS getGyro() {
    return navx;
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void resetEverything(boolean button) {
    zeroHeading();
    resetEncoders();
  }

  public void setSmartLimit(int MaxOutput) {
    m_leftMaster.setSmartCurrentLimit(MaxOutput);
    m_rightMaster.setSmartCurrentLimit(MaxOutput);
    m_leftFollower.setSmartCurrentLimit(MaxOutput);
    m_rightFollower.setSmartCurrentLimit(MaxOutput);
  }

  public boolean getButtonPressedOnce() {
    return buttonPressedOnce;
  }

  public boolean getButtonPressedTwice() {
    return buttonPressedTwice;
  }





}
