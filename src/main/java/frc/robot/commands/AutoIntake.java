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
  private double startTime = 1;
  private double endTime = 2;
  
  /** Creates a new AutoIntake. */
  public AutoIntake(Intake m_Intake) {
    this.m_Intake = m_Intake;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
  /* m_Intake.getSpinIntake().set(speed);
    m_Intake.getConveyIntake().set(speed); */
    if ( AutonomousCommand.timer >= startTime && AutonomousCommand.timer < endTime ){
     m_Intake.getSpinIntake().set(Constants.intakeSpinSpeed);
    m_Intake.getConveyIntake().set(Constants.conveyorSpeed);
    }
    else if (0 <= AutonomousCommand.timer && AutonomousCommand.timer < startTime){
      m_Intake.getSpinIntake().set(0);
      m_Intake.getConveyIntake().set(0);
    }
    else{
      m_Intake.getSpinIntake().set(0);
    m_Intake.getConveyIntake().set(0);
    }
    
    System.out.println(AutonomousCommand.timer);
  
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
    if ( AutonomousCommand.timer >= startTime && AutonomousCommand.timer < endTime ){
      return false;
    }
    else if (0 <= AutonomousCommand.timer && AutonomousCommand.timer < startTime){
      return false;
    }
    else {
      return true;
    } 
    //return false;
  }
}
