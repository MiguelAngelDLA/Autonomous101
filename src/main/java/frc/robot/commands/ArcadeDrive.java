// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */


  private final Drivetrain driveSubsystem;
  private  Supplier<Double> speed, turn;

  public ArcadeDrive(Drivetrain driveSubsystem,
          Supplier<Double> speed, Supplier<Double> turn) {
            
      this.speed = speed;
      this.turn = turn;
      this.driveSubsystem = driveSubsystem;
      addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    driveSubsystem.setArcadeDrive(-speed.get(), turn.get());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}