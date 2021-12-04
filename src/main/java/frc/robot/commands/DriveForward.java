// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForward extends CommandBase {
  private final Drivetrain driveSubsystem;
  private final double distance;

  public DriveForward(Drivetrain driveSubsystem, double distance) {
      this.driveSubsystem = driveSubsystem;
      this.distance = driveSubsystem.getAverageDistance() + distance;
      addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
      System.out.println("DriveForwardCmd started!");
  }

  @Override
  public void execute() {
      driveSubsystem.setMotors(0.2, 0.2);
  }

  @Override
  public void end(boolean interrupted) {
      driveSubsystem.setMotors(0, 0);
      System.out.println("DriveForwardCmd ended!");
  }

  @Override
  public boolean isFinished() {
      if (driveSubsystem.getAverageDistance() > distance)
          return true;
      else
          return false;
  }
}