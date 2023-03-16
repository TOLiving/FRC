// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.lib.PIDGains;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSubsystem extends SubsystemBase {
  private CANSparkMax m_frontLeftMotor;
  private CANSparkMax m_frontRightMotor;
  private CANSparkMax m_rearLeftMotor;
  private CANSparkMax m_rearRightMotor;

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private SparkMaxPIDController m_frontleftPID;
  private SparkMaxPIDController m_frontrightPID;

  private double kDistanceConversion = 1 / 42 * 6 * Math.PI;

  private double kP;
  private double kI;
  private double kD;
  private double errorSum;
  private double lastTimestamp;
  private double iLimit;
  private double lastError;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_frontLeftMotor  = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
    m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontLeftMotor.setIdleMode(IdleMode.kBrake);
    m_frontLeftEncoder = m_frontLeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_frontleftPID = m_frontLeftMotor.getPIDController();
    PIDGains.setSparkMaxGains(m_frontleftPID, Constants.Drivetrain.kDrivePIDGains);
    m_frontLeftMotor.burnFlash();

    m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
    m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_frontRightMotor.setIdleMode(IdleMode.kBrake);
    m_frontRightEncoder = m_frontRightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    m_frontrightPID = m_frontRightMotor.getPIDController();
    PIDGains.setSparkMaxGains(m_frontrightPID, Constants.Drivetrain.kDrivePIDGains);
    m_frontRightMotor.burnFlash();

    m_rearLeftMotor   = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
    m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearLeftMotor.setIdleMode(IdleMode.kBrake);
    m_rearLeftMotor.burnFlash();

    m_rearRightMotor  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
    m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    m_rearRightMotor.setIdleMode(IdleMode.kBrake);
    m_rearRightMotor.burnFlash();


    kP = 0.3;
    kI = 0;
    iLimit = 1;
    kD = 0;

    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    
  }

  public void driveArcade(double _straight, double _turn) {
    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

    m_frontLeftMotor.set(left);
    m_frontRightMotor.set(right);
    m_rearLeftMotor.set(left);
    m_rearRightMotor.set(right);
  }

  
  public void distancePID(double setpoint){
    // get encoder position
    double sensorPosition = m_frontRightEncoder.getPosition() * kDistanceConversion;
    // calculations
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if(Math.abs(error) < iLimit) {
      errorSum += error*dt;

    }

    double errorRate = (error - lastError) / dt;
    double outputSpeed = kP * error; //+ kI * errorSum + kD * errorRate;// Still need to test the first part of the code before I implement anything else

    //output to motors
    m_frontLeftMotor.set(-outputSpeed);
    m_frontRightMotor.set(outputSpeed);
    m_rearLeftMotor.set(-outputSpeed);
    m_rearRightMotor.set(outputSpeed);

    // update variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Value", m_frontRightEncoder.getPosition()* kDistanceConversion);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", m_controller);
  }
}
