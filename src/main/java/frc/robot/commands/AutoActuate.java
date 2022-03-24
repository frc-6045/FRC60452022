// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutoActuate extends CommandBase {
  /** Creates a new AutoActuate. */
  private Intake m_intake;
  private double speed;
 

  public AutoActuate(Intake subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = subsystem;
    addRequirements(m_intake);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.getActuateIntake().set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.getActuateIntake().set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.getActuateIntake().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
