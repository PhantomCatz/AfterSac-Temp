// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.Autonomous.CatzAutonomous;
import frc.Autonomous.CatzAutonomousPaths;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Mechanisms.CatzBalance;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzRGB;
import frc.Mechanisms.CatzIndexer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //----------------------------------------------------------------------------------------------
  //  Shared Libraries & Utilities
  //----------------------------------------------------------------------------------------------
  public static CatzConstants       constants;

  public static DataCollection      dataCollection;
  public ArrayList<CatzLog>         dataArrayList;

  //----------------------------------------------------------------------------------------------
  //  Shared Robot Components (e.g. not mechanism specific)
  //----------------------------------------------------------------------------------------------
  public static PowerDistribution PDH;
  //Camera
  //Limelight???
  

  public static AHRS                navX;

  public final int PH_CAN_ID = 1;
  public PneumaticHub pneumaticHub;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static final int DPAD_UP = 0;
  public static final int DPAD_DN = 180;
  public static final int DPAD_LT = 270;
  public static final int DPAD_RT = 90;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  public static boolean robotDisabled = false;
  public static boolean coneOnboard = false;
  public static boolean cubeOnboard = false;
  public static boolean cubeScoringReady = false;
  public static boolean coneScoringReady = false;
  public static boolean inAuton = false;
  public static boolean autobalancing = false;
  public static boolean noGamePiece = false;
  

  //----------------------------------------------------------------------------------------------
  //  Autonomous
  //----------------------------------------------------------------------------------------------
  public static CatzAutonomous      auton;
  public static CatzBalance         balance;
  public static CatzAutonomousPaths paths;

  //----------------------------------------------------------------------------------------------
  //  Mechanisms
  //----------------------------------------------------------------------------------------------
  public static CatzDrivetrain      drivetrain;
  public static CatzIntake          intake;
  public static CatzIndexer         indexer; 
  public static CatzRGB             led;

  public static Timer               currentTime;


  // put into mechanism classes 
  public static final boolean DEPLOYED = true;
  public static final boolean STOWED   = false;
  public static boolean elevatorState = STOWED;
  public static  boolean intakeState   = STOWED;
  public static boolean cubeRequest = false;
  public static boolean coneRequest = false;

  /*
   * For autobalancing
  */
  private final double OFFSET_DELAY = 0.5;    // put into mechanism classes

  /*-----------------------------------------------------------------------------------------
  *  
  *  robotXxx
  *
  *----------------------------------------------------------------------------------------*/
  /*-----------------------------------------------------------------------------------------
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  *----------------------------------------------------------------------------------------*/

  
  @Override
  public void robotInit()
  {
    //----------------------------------------------------------------------------------------------
    //  Shared Libraries & Utilities
    //----------------------------------------------------------------------------------------------
    constants      = new CatzConstants();

    dataCollection = new DataCollection();
    dataArrayList  = new ArrayList<CatzLog>();
    
    dataCollection.dataCollectionInit(dataArrayList);


    //----------------------------------------------------------------------------------------------
    //  Shared Robot Components (e.g. not mechanism specific)
    //----------------------------------------------------------------------------------------------
    PDH = new PowerDistribution();
        
    navX = new AHRS();
    navX.reset();


    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    //----------------------------------------------------------------------------------------------
    //  Autonomous
    //----------------------------------------------------------------------------------------------
    auton          = new CatzAutonomous();
    balance        = new CatzBalance();
    paths          = new CatzAutonomousPaths();

    //----------------------------------------------------------------------------------------------
    //  Mechanisms
    //----------------------------------------------------------------------------------------------
    drivetrain     = new CatzDrivetrain();
    intake         = new CatzIntake();
    
    indexer        = new CatzIndexer();    

    led            = new CatzRGB();

    currentTime = new Timer();
  }



  /*-----------------------------------------------------------------------------------------
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   *----------------------------------------------------------------------------------------*/
  @Override
  public void robotPeriodic()
  {
    dataCollection.updateLogDataID();
    

    //For each mechanism - debug and comp
    SmartDashboard.putNumber("NavX", navX.getAngle());

    balance.SmartDashboardBalance();
    drivetrain.smartDashboardDriveTrain();
        drivetrain.smartDashboardDriveTrain_DEBUG();

    indexer.SmartDashboardIndexer();
    intake.smartDashboardIntake();
    
   
   //debug should be commented out for comp

    balance.SmartDashboardBalanceDebug();
    //drivetrain.smartDashboardDriveTrain_DEBUG();
    //indexer.SmartDashboardIndexer_Debug();
    intake.smartDashboardIntake_Debug();

    led.LEDWork();

    
    
    
  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  autonomousXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
  *----------------------------------------------------------------------------------------*/
  @Override
  public void autonomousInit()
  {    
    currentTime.reset();
    currentTime.start();

    indexer.setBrakeMode();

    dataCollection.startDataCollection(); 
    navX.reset();

    navX.setAngleAdjustment(-navX.getYaw() + 180.0); //set navx's zero position to opposite way robot is facing
    
    Timer.delay(OFFSET_DELAY);

    paths.executeSelectedPath();
  }


  /*-----------------------------------------------------------------------------------------
  *
  *  This function is called periodically during autonomous.
  *
  *----------------------------------------------------------------------------------------*/
  @Override
  public void autonomousPeriodic()
  {
    
  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  teleopXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when teleop is enabled.
  *----------------------------------------------------------------------------------------*/
  @Override
  public void teleopInit() 
  {
    isInAuton();
    currentTime.reset();
    currentTime.start();

    balance.StopBalancing();
    indexer.setBrakeMode();

    dataCollection.startDataCollection();
  }


  /*-----------------------------------------------------------------------------------------
  *
  *  This function is called periodically during operator control.
  *
  *----------------------------------------------------------------------------------------*/
  @Override
  public void teleopPeriodic()
  {
    //----------------------------------------------------------------------------------------------
    //  Drivetrain
    //----------------------------------------------------------------------------------------------
    drivetrain.cmdProcSwerve(xboxDrv.getLeftX(), xboxDrv.getLeftY(), xboxDrv.getRightX(), navX.getAngle(), 
                             xboxDrv.getRightTriggerAxis());
    
    if(xboxDrv.getStartButtonPressed())
    {
      zeroGyro();
    }
    
    //----------------------------------------------------------------------------------------------
    //  Intake  intake stow/deploy/roller comands are changed back to driver. 
    //----------------------------------------------------------------------------------------------
    intake.procCmdIntake(xboxDrv.getLeftStickButton(),    //Deploy //THis should be set back to xboxAux during comp
                         xboxDrv.getRightStickButton()); //Stow

    intake.procCmdRoller(xboxDrv.getRightTriggerAxis(),   //Roller In
                         xboxDrv.getLeftTriggerAxis() );  //Roller Out


    intake.cmdProcIntakeShooting(xboxAux.getBButtonPressed(),    //shoot mid
                                 xboxAux.getYButtonPressed(),    //shoot high
                                 false,  //(xboxAux.getPOV() == DPAD_RT),  //shoot formula mid
                                 false,  //(xboxAux.getPOV() == DPAD_UP)); //shoot formula high
                                 xboxAux.getXButtonPressed());  //manual shooting
    
                                 
    if(xboxAux.getLeftBumperPressed())
    {
      intake.disengageLatch();
    }
    if(xboxAux.getRightBumperPressed())
    {
      intake.engageLatch();
    }

    if(xboxDrv.getPOV() == DPAD_DN)
    {
      intake.resetPivotDeployPos();
    }
    else if(xboxDrv.getPOV() == DPAD_UP)
    {
      intake.resetPivotStowPos();
    }



    //----------------------------------------------------------------------------------------------
    //  Indexer - Right bumper ejects stored game piece out front of robot
    //----------------------------------------------------------------------------------------------
    indexer.procCmdIndexerFrnt(xboxAux.getAButton());  //Front Indexer Out //needs to be set from xboxDrv to xboxAux

    // Auto Balance Controls
    if(xboxDrv.getAButtonPressed())
    {
      balance.StartBalancing();
    }
    else if(xboxDrv.getBButtonPressed())
    {
      balance.StopBalancing();
    }
    if(xboxAux.getPOV() == DPAD_LT)
    {
      drivetrain.lockWheels();
    }

    if(DriverStation.getMatchTime() < 2.0)
    {
      led.matchDone = true;

    }
    else if(DriverStation.getMatchTime() < 15.0)
    {
      led.endGame = true;
    }

  }

  /*-----------------------------------------------------------------------------------------
  *  
  *  disabledXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    currentTime.stop();
    indexer.setCoastMode();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

    
  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  testXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit()
  {
    navX.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  simulateXxx
  *
  *----------------------------------------------------------------------------------------*/
 /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}



  /*-----------------------------------------------------------------------------------------
  *  
  *  TBD - MOVE THIS TO APPROPRIATE CLASS
  *
  *----------------------------------------------------------------------------------------*/

    public void zeroGyro()
    {
      navX.setAngleAdjustment(-navX.getYaw());
    }

    public static boolean isInAuton()
    {
      return DriverStation.isAutonomous();
    }

    public static boolean isInDisabled()
    {
      return DriverStation.isDisabled();
    }

}