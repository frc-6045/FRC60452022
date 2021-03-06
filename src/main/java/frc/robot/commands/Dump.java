// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;

public class Dump extends CommandBase {
        private final FlyWheel m_flyWheel;
        private final Intake m_intake;
       private double dumpScale;
        private Joystick rightJoystick; 

    public Dump(FlyWheel subsystem, Intake m_intake, Joystick rightJoystick) {

        m_flyWheel = subsystem;
        this.m_intake = m_intake;
        this.rightJoystick = rightJoystick;
        addRequirements(m_flyWheel, m_intake);
   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    dumpScale = ((1+ rightJoystick.getRawAxis(3)) * .5);
    m_flyWheel.getDumpMotor().set(Constants.dumpSpeed);
    //m_flyWheel.getDumpMotor().set(Constants.dumpSpeed * dumpScale);
    m_intake.getConveyIntake().set(Constants.conveyorSpeed *-1); 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    m_flyWheel.getDumpMotor().set(0);
    m_intake.getConveyIntake().set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
