// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

// Subsystems
public final Lift m_lift = new Lift();
public final FlyWheel m_flyWheel = new FlyWheel();
public final Intake m_intake = new Intake();
public final DriveTrain m_driveTrain = new DriveTrain();
// Joysticks
private final Joystick arcadeJoystick = new Joystick(2);
private final Joystick rightTankJoystick = new Joystick(1);
private final Joystick leftTankJoystick = new Joystick(0);
// Commands
private final ArcadeDrive m_ArcadeDrive = new ArcadeDrive(m_driveTrain, arcadeJoystick);
private final TankDrive m_TankDrive = new TankDrive(m_driveTrain, leftTankJoystick, rightTankJoystick);
private final ChangeDirection m_ChangeDirection = new ChangeDirection(m_driveTrain, leftTankJoystick, rightTankJoystick, arcadeJoystick);
private final IntakeIn m_IntakeIn = new IntakeIn(m_intake);
private final IntakeOut m_IntakeOut = new IntakeOut(m_intake);
private final IntakeRise m_IntakeRise = new IntakeRise(m_intake);
private final IntakeFall m_IntakeFall = new IntakeFall(m_intake);
private final Dump m_Dump = new Dump(m_flyWheel, m_intake, rightTankJoystick);

// Command Getters
public ArcadeDrive getArcadeDrive(){ return m_ArcadeDrive;}
public TankDrive getTankDrive(){ return m_TankDrive;}
public ChangeDirection getChangeDirection(){ return m_ChangeDirection;}
public IntakeIn getIntakeIn(){ return m_IntakeIn;}
public IntakeOut getIntakeOut(){ return m_IntakeOut;}
public IntakeRise getIntakeRise(){ return m_IntakeRise;}
public IntakeFall getIntakeFall(){ return m_IntakeFall;}
public Dump getDump(){ return m_Dump;}

// Gyro
public static ADIS16470_IMU gyro = new ADIS16470_IMU();



  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
  
    // Smartdashboard Subsystems
      

    // SmartDashboard Buttons
   /*
   SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("TankDrive", new TankDrive( m_driveTrain ));
    SmartDashboard.putData("Dump", new Dump( m_flyWheel ));
    SmartDashboard.putData("Climb", new Climb( m_lift ));
    SmartDashboard.putData("IntakeRise", new IntakeRise( m_intake ));
    SmartDashboard.putData("IntakeIn", new IntakeIn( m_intake ));
*/
   
    // Configure the button bindings
    configureButtonBindings();
        

    // Configure autonomous sendable chooser
    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand(m_driveTrain, m_intake, m_flyWheel));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /*
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        
// Create some buttons
if (Constants.DrivePrefrance == 0){
//TankDrive
final JoystickButton rightTrigger = new JoystickButton(rightTankJoystick, Constants.rightTriggerID);        
rightTrigger.whenHeld(new Dump( m_flyWheel, m_intake, rightTankJoystick ) ,true);

final JoystickButton leftTrigger = new JoystickButton(leftTankJoystick, Constants.leftTriggerID);        
leftTrigger.whenHeld(new IntakeIn( m_intake) ,true);

final JoystickButton leftStickTopRight = new JoystickButton(leftTankJoystick, Constants.leftStickTopRightID);
final JoystickButton rightStickTopLeft = new JoystickButton(rightTankJoystick, Constants.rightStickTopLeftID);
leftStickTopRight.whenHeld(new IntakeOut(m_intake), true);
rightStickTopLeft.whenHeld(new IntakeOut(m_intake), true);

final JoystickButton rightOutsideBigBase = new JoystickButton(rightTankJoystick, Constants.rightOutsideBigBaseID);
final JoystickButton rightOutsideSmallBase = new JoystickButton(rightTankJoystick, Constants.rightOutsideSmallBaseID);
rightOutsideBigBase.whenPressed(new ChangeDirection(m_driveTrain, leftTankJoystick, rightTankJoystick, arcadeJoystick), true);
rightOutsideSmallBase.whenPressed(new TankDrive(m_driveTrain, leftTankJoystick, rightTankJoystick), true);
   
final JoystickButton arcadeLeftUpStick = new JoystickButton(arcadeJoystick, Constants.arcadeLeftUpStickID);
arcadeLeftUpStick.whenHeld(new IntakeRise(m_intake) ,true);

final JoystickButton arcadeLeftDownStick = new JoystickButton(arcadeJoystick, Constants.arcadeLeftDownStickID);
arcadeLeftDownStick.whenHeld(new IntakeFall(m_intake) ,true);

final JoystickButton arcadeTrigger = new JoystickButton(arcadeJoystick, Constants.arcadeTriggerID);        
arcadeTrigger.whenHeld(new Climb( m_lift ) ,true);

}else{
//ArcadeDrive
final JoystickButton arcadeTrigger = new JoystickButton(arcadeJoystick, Constants.arcadeTriggerID);        
arcadeTrigger.whenPressed(new Dump( m_flyWheel, m_intake, rightTankJoystick ) ,true);
  
final JoystickButton arcadeLeftUpStick = new JoystickButton(arcadeJoystick, Constants.arcadeLeftUpStickID);
arcadeLeftUpStick.whenHeld(new IntakeOut(m_intake) ,true);

final JoystickButton arcadeLeftDownStick = new JoystickButton(arcadeJoystick, Constants.arcadeLeftDownStickID);
arcadeLeftDownStick.whenHeld(new IntakeIn(m_intake) ,true);

final JoystickButton arcadeBottomLeft = new JoystickButton(arcadeJoystick, Constants.arcadeBottomLeftID);
arcadeBottomLeft.whenPressed(new ArcadeDrive(m_driveTrain, arcadeJoystick)); 

final JoystickButton arcadeBottomRight = new JoystickButton(arcadeJoystick, Constants.arcadeBottomRightID);
arcadeBottomRight.whenPressed(new ChangeDirection(m_driveTrain, leftTankJoystick, rightTankJoystick, arcadeJoystick));

final JoystickButton rightOutsideBigBase = new JoystickButton(rightTankJoystick, Constants.rightOutsideBigBaseID);        
rightOutsideBigBase.whenPressed(new Climb( m_lift ) ,true);

final JoystickButton rightTrigger = new JoystickButton(rightTankJoystick, Constants.rightTriggerID);        
rightTrigger.whenHeld(new IntakeRise( m_intake ) ,true);

final JoystickButton leftTrigger = new JoystickButton(leftTankJoystick, Constants.leftTriggerID);        
leftTrigger.whenHeld(new IntakeFall( m_intake) ,true);
}

}

    
public Joystick getLeftTankJoystick() {
        return leftTankJoystick;
    }

public Joystick getRightTankJoystick() {
        return rightTankJoystick;
    }

public Joystick getArcadeJoystick() {
        return arcadeJoystick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
  
  
  
    // The selected command will be run in autonomous
   return m_chooser.getSelected();
  }
  

}

