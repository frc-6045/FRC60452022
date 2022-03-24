// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class AutoScore extends CommandBase {
  private final FlyWheel m_FlyWheel;
  private final Intake m_Conveyor;
  private double DumpSpeed;
  private double ConveyorSpeed;
  /** Creates a new AutoScore. */
  public AutoScore(FlyWheel subsystem, Intake m_Conveyor, double DumpSpeed, double ConveyorSpeed ) {
    m_FlyWheel = subsystem;
    this.m_Conveyor = m_Conveyor;
    this.DumpSpeed = DumpSpeed;
    this.ConveyorSpeed = ConveyorSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_FlyWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_FlyWheel.getDumpMotor().set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  m_FlyWheel.getDumpMotor().set(DumpSpeed);
  m_Conveyor.getConveyIntake().set(ConveyorSpeed);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_FlyWheel.getDumpMotor().set(0);
    m_Conveyor.getConveyIntake().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
