// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardTimed extends CommandBase {
  /** Creates a new DriveForwardTimed. */
  private final Drivetrain driveSubsystem;
  double finalTime, initTime;

  public DriveForwardTimed(Drivetrain driveSubsystem, double time) {
      this.driveSubsystem = driveSubsystem;
      this.finalTime = time;
      addRequirements(driveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotors(0.2, 0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - initTime >= finalTime;
  }
}
