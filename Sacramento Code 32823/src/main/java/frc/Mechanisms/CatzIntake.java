package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzIntake {

  private Thread intakeThread;
  private final double INTAKE_THREAD_PERIOD  = 0.020;
  
  private CatzLog data;

  //------------------------------------------------------------------------------------------------
  //
  //  Roller
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakeRollerMotor;

  private final int    INTAKE_ROLLER_MC_ID           = 11; 

  private final double INTAKE_MOTOR_POWER_ROLLER_IN  = 0.8; //-1.0
  public final double  INTAKE_MOTOR_POWER_ROLLER_IN_AUTON = 0.9;
  private final double INTAKE_MOTOR_POWER_ROLLER_OUT =  -1.0; //1.0  may be fixing wiring issue on motor controller (original values used to work)
  private final double INTAKE_MOTOR_POWER_OFF =  0.0;
  private double intakeStowPower = 0.0;

  private final double INTAKE_INPUT_THRESHOLD = 0.2;

  //------------------------------------------------------------------------------------------------
  //
  //  Deploy/Stow and Pivot
  //
  //------------------------------------------------------------------------------------------------
  private WPI_TalonFX intakePivotMotor;
  
  private final PneumaticsModuleType PH_TYPE = PneumaticsModuleType.REVPH;
    
  private static DoubleSolenoid intakeSolenoid;

  private final int INTAKE_LATCH_ENGAGE_PCM_PORT  = 0;  /* TBD warning... temporary values, port #'s not confirmed yet by electronics' */
  private final int INTAKE_LATCH_RELEASE_PCM_PORT = 1;  


  private final int     INTAKE_PIVOT_MC_ID           = 10; 
  private final double  INTAKE_PIVOT_REL_ENCODER_CPR = 2048.0;

  private final double  CURRENT_LIMIT_AMPS            = 70.0; //60.0
  private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 70.0; //60.0
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;

  private SupplyCurrentLimitConfiguration EncCurrentLimit;


  //Pivot status
  private  final int INTAKE_PIVOT_MODE_NULL       = 0;
  private  final int INTAKE_PIVOT_MODE_DEPLOY     = 1;
  private  final int INTAKE_PIVOT_MODE_STOW       = 6;
  public   final int INTAKE_PIVOT_MODE_SHOOT      = 7;
  private  final int INTAKE_PIVOT_MODE_STOW_SHOOT = 8;

  //Pivot IntakeMode initialization
  public int intakePivotMode = INTAKE_PIVOT_MODE_NULL;

  //------------------------------------------------------------------------------------------------
  //  Gear ratio
  //------------------------------------------------------------------------------------------------
  private final double INTAKE_PIVOT_PINION_GEAR = 11.0;
  private final double INTAKE_PIVOT_SPUR_GEAR   = 56.0;  
  private final double INTAKE_PIVOT_GEAR_RATIO  =INTAKE_PIVOT_SPUR_GEAR/INTAKE_PIVOT_PINION_GEAR;

  private final double INTAKE_PIVOT_SPROCKET_1  = 16.0;
  private final double INTAKE_PIVOT_SPROCKET_2  = 56.0;
  private final double INTAKE_PIVOT_SPROCKET_RATIO  = INTAKE_PIVOT_SPROCKET_2/INTAKE_PIVOT_SPROCKET_1;
  
  private final double INTAKE_PIVOT_FINAL_RATIO = INTAKE_PIVOT_GEAR_RATIO*INTAKE_PIVOT_SPROCKET_RATIO;
  
  
  //------------------------------------------------------------------------------------------------
  //  Angle Definitions
  //------------------------------------------------------------------------------------------------
  private final static double INTAKE_STOWED_ANGLE   = 0.0;
  private final static double INTAKE_DEPLOYED_ANGLE = 40.0;   //TBD changing for shooting mode v2

  private final double INTAKE_STOW_CUT_ANGLE = 35.0;

  //------------------------------------------------------------------------------------------------
  //  Motion Magic Approach
  //------------------------------------------------------------------------------------------------

  private final double INTAKE_DEPLOYED_ANGLE_COUNTS   = -(INTAKE_DEPLOYED_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  private final double INTAKE_STOWED_ANGLE_COUNTS     =  (INTAKE_STOWED_ANGLE   * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);
  
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE        = 0.0;  //Setting to zero assuming deploy code will not run motor into the hard-stops
  private final double INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS = (INTAKE_SOFTLIMIT_OFFSET_ANGLE * (INTAKE_PIVOT_REL_ENCODER_CPR / 360.0) * INTAKE_PIVOT_FINAL_RATIO);

  private final double INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS    = INTAKE_DEPLOYED_ANGLE_COUNTS - INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;
  private final double INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS    = INTAKE_STOWED_ANGLE_COUNTS   + INTAKE_SOFTLIMIT_OFFSET_ANGLE_COUNTS;


  private final double INTAKE_DEPLOY_LATCH_ENGAGE_DELAY = 0.2;

  private final double INTAKE_PIVOT_DEPLOY_POWER = -0.2;    //TBD May not need depending on final implementation

  //------------------------------------------------------------------------------------------------
  //  Intake Shooting
  //------------------------------------------------------------------------------------------------
  private final double INTAKE_SHOOT_ANGLE = 25.0; //40.0
  private double shootAngleError;

  private final double INTAKE_PIVOT_SHOOT_KP = 0.008; //0.02
  private final double MAX_PIVOT_POWER_SHOOT = 0.35;
  private final double MIN_PIVOT_POWER_SHOOT = 0.2;
  private double intakePivotPower;
  private int intakePivotShootCounter = 0;

  private final double INTAKE_PIVOT_EASE_POWER = 0.05; //0.026

  private double intakeShootPower;
  private final double POWER_SHOOT_HIGH = -0.75;
  private final double POWER_SHOOT_MID  = -0.6;

  private double currentAngle      = 0.0;

  private Timer  pivotTimer;
  private double time = 0.0;

  private double deploymentMotorRawPosition;

  public boolean intakeActive;

  private int intakeStowCount = -1;

  private boolean shoot = false;
  private int intakePivotShootStowCounter = 0;

  private int traceID = 0;

//---------------------------------------------definitions part end--------------------------------------------------------------
  
    /*-----------------------------------------------------------------------------------------
    *  
    *  CatzIntake()
    *
    *----------------------------------------------------------------------------------------*/
    public CatzIntake() {
    //need add softlimits

    intakeRollerMotor = new WPI_TalonFX(INTAKE_ROLLER_MC_ID);
    intakePivotMotor  = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

    EncCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                CURRENT_LIMIT_TIMEOUT_SECONDS);
    intakeRollerMotor.configFactoryDefault();
    intakeRollerMotor.setNeutralMode(NeutralMode.Coast);
    intakeRollerMotor.configSupplyCurrentLimit(EncCurrentLimit);


    intakePivotMotor.configFactoryDefault();
    intakePivotMotor.setNeutralMode(NeutralMode.Brake);
    intakePivotMotor.configSupplyCurrentLimit(EncCurrentLimit);

    intakePivotMotor.configForwardSoftLimitThreshold(INTAKE_SOFTLIMIT_MIN_ANGLE_COUNTS);
    intakePivotMotor.configReverseSoftLimitThreshold(INTAKE_SOFTLIMIT_MAX_ANGLE_COUNTS);

    intakePivotMotor.configForwardSoftLimitEnable(true);
    intakePivotMotor.configReverseSoftLimitEnable(true);

    

    intakeSolenoid = new DoubleSolenoid(PH_TYPE, INTAKE_LATCH_ENGAGE_PCM_PORT,
                                                 INTAKE_LATCH_RELEASE_PCM_PORT);

    engageLatch();

    pivotTimer = new Timer();
   
    intakeControl();

  }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Roller Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeRollerIn()
    {
      Robot.indexer.indexerFrntIngest();
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_IN);
      intakeActive = true;
      
    }

    public void intakeRollerInAuton()
    {
      Robot.indexer.indexerFrntIngest();
      intakeRollerMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_ROLLER_IN_AUTON);
      intakeActive = true;
    }

    public void intakeRollerOut()
    {
      intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_ROLLER_OUT);
    }

    public void intakeShootOut()
    {
      intakeRollerMotor.set(ControlMode.PercentOutput, intakeShootPower);
    }


    public void intakeRollerOff()
    {
        intakeRollerMotor.set(ControlMode.PercentOutput,INTAKE_MOTOR_POWER_OFF);
        intakeActive = false;
        if(Robot.indexer.indexerActive == false)
        {
          Robot.indexer.indexerFrntStop();
        }
    }


    public void procCmdRoller(double xboxValueIn, double xboxValueOut)
    {
      if (xboxValueIn > INTAKE_INPUT_THRESHOLD)
      {
        intakeRollerIn();  
      }
      else if (xboxValueOut > INTAKE_INPUT_THRESHOLD)
      { 
        intakeRollerOut();     
      }
      else if(intakePivotMode != INTAKE_PIVOT_MODE_SHOOT)
      {
        intakeRollerOff();
      }
    }


    public void procCmdIntake(boolean xboxValueDeploy, boolean xboxValueStow)
    {

      if (xboxValueDeploy == true)
      {
          if(Robot.elevatorState == Robot.DEPLOYED)
          {
              //flash colors indicating that you can't deploy
          }
          else
          {
            intakeDeploy();
            Robot.intakeState = Robot.DEPLOYED;
          }
      }
      else if (xboxValueStow == true)
      {
        intakeStow();
        Robot.intakeState = Robot.STOWED;
      }
    }

    public void cmdProcIntakeShooting(boolean shootMid, boolean shootHigh, boolean formulaShootMid, boolean formulaShootHigh, boolean manualShoot)
    {      
      if(intakePivotMode != INTAKE_PIVOT_MODE_SHOOT)
      {
        if(shootMid == true)
        {
          intakeShootMid();
        }
        else if(shootHigh == true)
        {
          intakeShootHigh();
        }
        else if(formulaShootMid == true)
        {
          intakeShootFormMid();
        }
        else if(formulaShootHigh == true)
        {
          intakeShootFormHigh();
        }
      }

      if(manualShoot == true)
      {
        shoot = true;
        System.out.println("ms");
      }
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Deploy/Stow Methods
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeControl()
    {
      intakeThread = new Thread(() ->
      { 
        while(true)
        {
          time = pivotTimer.get();

          switch(intakePivotMode)
          {
              case INTAKE_PIVOT_MODE_NULL:
            
              break;
               
              case INTAKE_PIVOT_MODE_DEPLOY:
                currentAngle = getIntakeAngle();
                if(currentAngle > 20.0)  //INTAKE_DEPLOY_SET_POSITION_START_ANGLE )
                {
                  intakePivotMotor.set(ControlMode.PercentOutput, 0.0);

                  intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                  
                }
              break;

              case INTAKE_PIVOT_MODE_SHOOT:

                currentAngle = getIntakeAngle();
                shootAngleError = currentAngle - INTAKE_SHOOT_ANGLE;
                intakePivotPower = shootAngleError * INTAKE_PIVOT_SHOOT_KP;

                if(intakePivotPower >= MAX_PIVOT_POWER_SHOOT)
                {
                  intakePivotPower = MAX_PIVOT_POWER_SHOOT;
                }
                else if(intakePivotPower <= MIN_PIVOT_POWER_SHOOT)
                {
                  intakePivotPower = MIN_PIVOT_POWER_SHOOT;
                }


                if(currentAngle >= INTAKE_SHOOT_ANGLE)
                {
                  intakePivotMotor.set(ControlMode.PercentOutput, intakePivotPower);
                } 
                else if(currentAngle >= INTAKE_SHOOT_ANGLE - 10.0)
                {
                  intakePivotPower = INTAKE_PIVOT_EASE_POWER;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakePivotPower);
                }
                else if(currentAngle > INTAKE_SHOOT_ANGLE - 20.0)
                {
                  intakePivotPower = 0.0;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakePivotPower);
                }


                if(currentAngle <= INTAKE_SHOOT_ANGLE + 1 && currentAngle >= INTAKE_SHOOT_ANGLE - 1)
                {
                  intakePivotShootCounter++;
                }

                if(intakePivotShootCounter >= 5)
                {
                  shoot = true;
                  System.out.println("AS");
                }

                if(shoot == true)
                {
                  intakeShootOut();
                  Robot.indexer.indexerFrntEject();
                  intakePivotShootStowCounter++;

                  if(intakePivotShootStowCounter >= 25) //after 1/2 second
                  {
                    shoot = false;
                    intakeStowShoot();
                  }
                }
                else if(time >= 3.0) //might be able to lower timeout?
                {
                  shoot = false;
                  intakeStowShoot();
                }
                
              break;

              case INTAKE_PIVOT_MODE_STOW:

                disengageLatch();

                currentAngle = getIntakeAngle();
                
                

                if(currentAngle <= 1.0)
                {
                  intakeStowCount++;
                  if(intakeStowCount >= 15)
                  { 
                    intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                    engageLatch();
                    intakeStowPower = INTAKE_MOTOR_POWER_OFF;
                    intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                  }
                }
                else if(currentAngle <= 5.0)
                {
                  intakeStowCount = 0;
                  intakeStowPower = 0.25;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                }
                else if(currentAngle <= 10.0)
                {
                  intakeStowCount = 0;
                  intakeStowPower = 0.25;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                }
                else if(currentAngle <= INTAKE_STOW_CUT_ANGLE)
                {
                  if(intakeStowCount < 0)
                  {
                    intakeStowCount = 0;
                    intakeStowPower = INTAKE_MOTOR_POWER_OFF;
                    intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                  }
                  else
                  {
                    //keep the previous power
                  }
                }

              break;

              case INTAKE_PIVOT_MODE_STOW_SHOOT:
                disengageLatch();

                currentAngle = getIntakeAngle();
                
                

                if(currentAngle <= 1.0)
                {
                  intakeStowCount++;
                  if(intakeStowCount >= 15)
                  { 
                    intakePivotMode = INTAKE_PIVOT_MODE_NULL;
                    engageLatch();
                    intakeStowPower = INTAKE_MOTOR_POWER_OFF;
                    intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                  }
                }
                else if(currentAngle <= 5.0)
                {
                  intakeStowCount = 0;
                  intakeStowPower = 0.25;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                }
                else if(currentAngle <= 10.0)
                {
                  intakeStowCount = 0;
                  intakeStowPower = 0.25;
                  intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
                }
                
              break;

              default:
                intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER_OFF);
              break;

          }   //end of switch

          if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE))
          {
            DataCollection.booleanDataLogging(shoot, 0);


            data = new CatzLog(time, currentAngle, intakeStowCount, intakePivotPower, 
                                                                    intakePivotMotor.getMotorOutputPercent(), 
                                                                    intakePivotShootCounter, 
                                                                    intakePivotShootStowCounter,
                                                                    intakeStowPower, shootAngleError, -999.0, 
                                                                    -999.0, -999.0, -999.0, -999.0, -999.0,
                                                                    DataCollection.boolData);  
            Robot.dataCollection.logData.add(data);
          }

          Timer.delay(INTAKE_THREAD_PERIOD);
        
        }  //end of while true
              
      });
      intakeThread.start();
    

    }   //end of intakeControl();


    /*-----------------------------------------------------------------------------------------
    *  
    * intakeDeploy()
    *
    *----------------------------------------------------------------------------------------*/
    public void intakeDeploy()
    {
      pivotTimer.reset();
      pivotTimer.start();

      disengageLatch();
      Timer.delay(INTAKE_DEPLOY_LATCH_ENGAGE_DELAY);        //Wait for latch to clear 
      
      intakePivotMotor.set(ControlMode.PercentOutput,INTAKE_PIVOT_DEPLOY_POWER);
      intakePivotMode = INTAKE_PIVOT_MODE_DEPLOY;
    }

    public void intakeStow()
    {
      pivotTimer.reset();
      pivotTimer.start();

      intakeStowCount = -1;
      intakeStowPower = 0.45;
      intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
      intakeRollerOff();      
      intakePivotMode = INTAKE_PIVOT_MODE_STOW;
    }

    public void intakeStowShoot()
    {
      pivotTimer.reset();
      pivotTimer.start();

      intakeStowCount = -1;
      intakeStowPower = 0.40;
      intakePivotMotor.set(ControlMode.PercentOutput, intakeStowPower);
      intakeRollerOff();      
      intakePivotMode = INTAKE_PIVOT_MODE_STOW_SHOOT;
    }

    public void intakeShootMid()
    {
      intakeShootPower = POWER_SHOOT_MID;

      intakeShoot();
    }

    public void intakeShootHigh()
    {
      intakeShootPower = POWER_SHOOT_HIGH;

      intakeShoot();
    }
    

    public void intakeShootFormMid()
    {
      intakeShootPower = -0.6; //formula calculation

      intakeShoot();
    }

    public void intakeShootFormHigh()
    {
      intakeShootPower = -0.8; //formula calculation

      intakeShoot();
    }

    public void intakeShoot()
    {
      pivotTimer.reset();
      pivotTimer.start();

      disengageLatch();
      Timer.delay(INTAKE_DEPLOY_LATCH_ENGAGE_DELAY);        //Wait for latch to clear


      intakeStowCount             = -1;
      intakePivotShootCounter     = 0;
      intakePivotShootStowCounter = 0;
      shoot                       = false;
 
      intakePivotMotor.set(ControlMode.PercentOutput, INTAKE_PIVOT_DEPLOY_POWER);
      intakePivotMode = INTAKE_PIVOT_MODE_SHOOT;
    }

    public void shootOverrideAuton()
    {
      shoot = true;
    }


    /*-----------------------------------------------------------------------------------------
    *
    *  getIntakePositionDegrees
    *
    *----------------------------------------------------------------------------------------*/
    public double getIntakeAngle()
    {
        deploymentMotorRawPosition = intakePivotMotor.getSelectedSensorPosition();

        double motorShaftRevolution = deploymentMotorRawPosition / INTAKE_PIVOT_REL_ENCODER_CPR;
        double pivotShaftRevolution = motorShaftRevolution       / INTAKE_PIVOT_FINAL_RATIO;
        double pivotAngle           = pivotShaftRevolution * -360.0; //motor  spin forward is positive 
        
        return pivotAngle;   
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void smartDashboardIntake()
    {
        SmartDashboard.putNumber("PivotAngle", getIntakeAngle());
        SmartDashboard.putNumber("Shooting Counter", intakePivotShootCounter);
    }

    public void smartDashboardIntake_Debug()
    {
      // SmartDashboard.putNumber("PivotCounts", deploymentMotorRawPosition);
      // SmartDashboard.putNumber("getClosedLoopError", intakePivotMotor.getClosedLoopError());
      // SmartDashboard.putNumber("getClosedLoopTarget", intakePivotMotor.getClosedLoopTarget());
      // SmartDashboard.putNumber("getStatorCurrent", intakePivotMotor.getStatorCurrent());
      // SmartDashboard.putNumber("Intake state", intakePivotMode);
      // SmartDashboard.putNumber("CurrentPivotAngle", currentAngle);
      SmartDashboard.putBoolean("Shoot", shoot);
      
    }
  

    public void engageLatch()
    {
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void disengageLatch()
    {
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void resetPivotStowPos()
    {
      intakePivotMotor.setSelectedSensorPosition(0.0);
    }
    public void resetPivotDeployPos()
    {
      intakePivotMotor.setSelectedSensorPosition(101.0);
    }
} 