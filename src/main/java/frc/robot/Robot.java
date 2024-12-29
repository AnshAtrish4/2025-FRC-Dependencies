package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.IO.LED;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
import frc.robot.subsystems.vision.DualCamera;

import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends LoggedRobot {
  private DualCamera dualCamera;
  private Drivebase drivebase;
 
  private LED litty;

  private static XboxController driver;
  private static XboxController operator;

  private Command m_autoSelected;

 


  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    dualCamera = DualCamera.getInstance();
    drivebase = Drivebase.getInstance();
    
    litty = LED.getInstance();
  

    driver = new XboxController(0);
    operator = new XboxController(1);

   
    
    Pose2d defaultPose = new Pose2d(0, 0, new Rotation2d(0));

 

    // m_chooser.addOption("New Auto", new PathPlannerAuto("New Auto"));

  
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override

  public void robotPeriodic() {
      


    CommandScheduler.getInstance().run();
    drivebase.periodic();

    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

    //SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());

  
  }

  @Override
  public void autonomousInit() {
     

     
    m_autoSelected = m_chooser.getSelected();

    drivebase.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(m_chooser.getSelected().getName()));
    
    

    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
   



    
  }

  @Override
  public void teleopInit() {

    litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

  
  }

  @Override
  public void teleopPeriodic() {

    /* DRIVE CONTROLS */


    double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    double rot = drivebase.inputDeadband(-driver.getRightX());

    if (driver.getAButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 180);
    } else if (driver.getBButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 270);
    } else if (driver.getYButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 0);
    } else if (driver.getXButton()) {
      drivebase.currHeading = -1;
      drivebase.rotateTo(xSpeed, ySpeed, 90);
    } else if (driver.getLeftTriggerAxis() > 0) {
      // drivebase.holdHeading(xSpeed, ySpeed);
    } else {
      drivebase.currHeading = -1;
      drivebase.drive(xSpeed, ySpeed, rot);
    }
    // drivebase.lockWheels();

    

    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    if(driver.getPOV() == 180){
      litty.setRed();
    }
    
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}