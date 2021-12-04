// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForwardBangBang extends CommandBase {
  /** Creates a new DriveForwardBangBang. */
  private final Drivetrain driveSubsystem;
  private final double distance;
  
  public DriveForwardBangBang(Drivetrain driveSubsystem, double distance) {

    this.driveSubsystem = driveSubsystem;
    this.distance = driveSubsystem.getAverageDistance() + distance;
    addRequirements(driveSubsystem);

}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveSubsystem.getAverageDistance() < distance){
      driveSubsystem.setMotors(0.4, 0.4);
    }
    else if(driveSubsystem.getAverageDistance() > distance){
      driveSubsystem.setMotors(-0.4, -0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
