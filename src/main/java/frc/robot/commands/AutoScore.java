// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FlyWheel;

public class AutoScore extends CommandBase {
  private final FlyWheel m_FlyWheel;
  private double speed;
  private double time;
  private double runTime;
  /** Creates a new AutoScore. */
  public AutoScore(FlyWheel subsystem, double speed, double time) {
    m_FlyWheel = subsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_FlyWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  runTime = Timer.getMatchTime();
  m_FlyWheel.getDumpMotor().set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_FlyWheel.getDumpMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return runTime >= time;
  }
}
