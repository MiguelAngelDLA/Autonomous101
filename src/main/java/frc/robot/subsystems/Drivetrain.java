// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax leftMaster = new CANSparkMax(Constants.mFrontLeft, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(Constants.mRearLeft, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(Constants.mFrontRight, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(Constants.mRearRight, MotorType.kBrushless);

  CANEncoder leftEncoder = leftMaster.getEncoder();
  CANEncoder rightEncoder = rightMaster.getEncoder();

  PIDController driveController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  
  public Drivetrain() {
    
    leftMaster.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    leftSlave.restoreFactoryDefaults();
    rightSlave.restoreFactoryDefaults();

    leftMaster.setSmartCurrentLimit(40);
    rightMaster.setSmartCurrentLimit(40);
    leftSlave.setSmartCurrentLimit(40);
    rightSlave.setSmartCurrentLimit(40);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    leftSlave.follow(leftMaster, false);
    rightSlave.follow(rightMaster, false);
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Metros Izquierdo", getLeftMeters());
    SmartDashboard.putNumber("Metros Derecho", getRightMeters());
    SmartDashboard.putNumber("Promedio Encoders", getAveragePosition());
  }

  public double getLeftEncoderValue(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoderValue(){
    return rightEncoder.getPosition();
  }

  public double getAveragePosition(){
    return (getRightEncoderValue() + getLeftEncoderValue()) / 2;
  }

  public double getLeftMeters(){

    double wheelRotations = getLeftEncoderValue() / 12;
    double distance = wheelRotations * (Math.PI * Constants.wheelDiameter);

    return distance;
  }

  public double getRightMeters(){

    double wheelRotations = getRightEncoderValue() / 12;
    double distance = wheelRotations * (Math.PI * Constants.wheelDiameter);

    return distance;
  }

  public double getAverageDistance(){
    return (getRightMeters() + getLeftMeters()) / 2;
  }
  public void setArcadeDrive(double speed, double turn) {

    double left = speed - turn;
    double right = speed + turn;

    leftMaster.set(left);
    rightMaster.set(right);
  }
  
  public void setMotors(double RightMotor, double LeftMotor){
    leftMaster.set(LeftMotor);
    rightMaster.set(RightMotor);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
