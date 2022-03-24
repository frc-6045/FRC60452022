// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class AutoIntake extends CommandBase {
  private final Intake m_Intake;
  private double SpinSpeed;
  private double ConveyorSpeed;

  
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake m_Intake, double SpinSpeed, double ConveyorSpeed) {
    this.m_Intake = m_Intake;
    this.SpinSpeed = SpinSpeed;
    this.ConveyorSpeed = ConveyorSpeed;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    
   m_Intake.getSpinIntake().set(SpinSpeed);
    m_Intake.getConveyIntake().set(ConveyorSpeed);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.getSpinIntake().set(0);
    m_Intake.getConveyIntake().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
    }
}
