
package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

public class CatzIndexer{
    //          SPARKMAX DEFS
    public CANSparkMax indexerMtrCtrlFrnt;

    private final int INDEXER_FRNT_MC_CAN_ID        = 30;//1;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;
    public final double INDEXER_FRNT_SPEED_FAST = 1;

    //
    public final double INDEXER_SPEED_OFF = 0.0;

    public boolean indexerActive = false;
    
    //          SECOND MOTOR DEFS
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;


//              RIGHT WHEELS
    public CANSparkMax indexerMtrCtrlRGT_WHEELS;

    private final int INDEXER_MC_CAN_ID_RGT       = 37;//2;

    public final double INDEXER_SIDE_WHEEL_SPEED_INJEST = -0.3;

    public CANSparkMax indexerMtrCtrlLFT_WHEELS;

    private final int INDEXER_MC_CAN_ID_LFT       = 38;//2;

    public final double INDEXER_SIDE_WHEEL_SPEED_EJECT = 0.35;
    private final double SIDE_WHEEL_SPEED_STOP         = 0.0;


    //          BEAMBREAK DEFS
  
    public DigitalInput beamBreak;
    public DigitalInput LimitSwitch;
    private final int BEAM_BREAK_DIO_PORT_LFT = 5;

    private int beamBreakCounter = 0;
    private final int GAME_PIECE_PRESENT_THRESHOLD = 3;   

    //
    private Thread Indexer;

    private Timer indexerTimer;
    private double indexerTime;

    //          CONSTANTS

    private double INDEXER_MAX_BELT_TIME = 2.0;   

    public CatzLog data;


    public CatzIndexer() 
    {
        indexerMtrCtrlFrnt = new CANSparkMax(INDEXER_FRNT_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlFrnt.restoreFactoryDefaults();
        indexerMtrCtrlFrnt.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlFrnt.setSmartCurrentLimit(INDEXER_FRNT_MC_CURRENT_LIMIT);

        
        indexerMtrCtrlRGT_WHEELS = new CANSparkMax(INDEXER_MC_CAN_ID_RGT, MotorType.kBrushless); 

        indexerMtrCtrlRGT_WHEELS.restoreFactoryDefaults();
        indexerMtrCtrlRGT_WHEELS.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlRGT_WHEELS.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);
 

        indexerMtrCtrlLFT_WHEELS = new CANSparkMax(INDEXER_MC_CAN_ID_LFT, MotorType.kBrushless); 

        indexerMtrCtrlLFT_WHEELS.restoreFactoryDefaults();
        indexerMtrCtrlLFT_WHEELS.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlLFT_WHEELS.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);
        indexerMtrCtrlLFT_WHEELS.follow(indexerMtrCtrlRGT_WHEELS, true);
    
        indexerTimer = new Timer();
        
        beamBreak = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);        
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Indexer Belt Controls
    *
    *----------------------------------------------------------------------------------------*/

    public void runIndexerBeltRev() 
    {
        indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED);
    }

    public void runIndexerBelt() 
    {
        indexerMtrCtrlFrnt.set(-INDEXER_FRNT_SPEED);
    }

    public void runIndexerBeltFast()
    {
        indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED_FAST);
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Side Wheel Controls
    *
    *----------------------------------------------------------------------------------------*/

    public void runSideWheelsFwd()
    {
        indexerMtrCtrlRGT_WHEELS.set(INDEXER_SIDE_WHEEL_SPEED_INJEST);

    } 
    
    public void runSideWheelsFev()
    {
        indexerMtrCtrlRGT_WHEELS.set(INDEXER_SIDE_WHEEL_SPEED_EJECT);

    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Front Indexer Controls
    *
    *----------------------------------------------------------------------------------------*/


    public void indexerFrntIngest() 
    {
        if(isGamePieceThere())
        {
            indexerFrntStop();
        }
        else
        {
            indexerActive = true;
            indexerMtrCtrlRGT_WHEELS.set(INDEXER_SIDE_WHEEL_SPEED_INJEST);
            indexerMtrCtrlFrnt.set(INDEXER_FRNT_SPEED);   
        }
        
    }

    public void indexerFrntEject() 
    {
        indexerActive = true;
        indexerMtrCtrlRGT_WHEELS.set(INDEXER_SIDE_WHEEL_SPEED_EJECT);
        indexerMtrCtrlFrnt.set(INDEXER_BACK_SPEED);
    }

    public void indexerFrntStop() 
    {
        indexerActive = false;
        indexerMtrCtrlRGT_WHEELS.set(SIDE_WHEEL_SPEED_STOP);
        indexerMtrCtrlFrnt.set(SIDE_WHEEL_SPEED_STOP);
    }


    /*-----------------------------------------------------------------------------------------
    *  
    *  Xbox control
    *
    *----------------------------------------------------------------------------------------*/

    public void procCmdIndexerFrnt(boolean xboxValueOut)
    {
      
      if (xboxValueOut == true)
      { 
        indexerFrntEject();
      }
      else if(Robot.intake.intakePivotMode != Robot.intake.INTAKE_PIVOT_MODE_SHOOT)
      {
        if(Robot.intake.intakeActive == false)
        {
            indexerFrntStop();
        }
      }
    }
    /*-----------------------------------------------------------------------------------------
    *  
    *  Data Collection
    *
    *----------------------------------------------------------------------------------------*/

    public void DataCollectionIndexer()
    {
        if (DataCollection.LOG_ID_INDEXER == DataCollection.chosenDataID.getSelected())
        {
            data = new CatzLog(Robot.currentTime.get(),0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);  
            Robot.dataCollection.logData.add(data);
        }
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void SmartDashboardIndexer()
    {
        SmartDashboard.putBoolean("BeamBreak", beamBreak.get());
    }
    public void SmartDashboardIndexer_Debug()
    {
        
    }

    public boolean isGamePieceThere()
    {
        boolean gamePiece = false;

        if(beamBreak.get())
        {
            beamBreakCounter--;
        }
        else
        {
            beamBreakCounter++;
        }

        if(beamBreakCounter <= 0)
        {
            beamBreakCounter = 0;
        }
        else if(beamBreakCounter >= 5)
        {
            beamBreakCounter = 5;
        }

        if(beamBreakCounter >= GAME_PIECE_PRESENT_THRESHOLD)
        {
           gamePiece = true;
        }
        
        return gamePiece;
    }

    public void setBrakeMode()
    {
        indexerMtrCtrlRGT_WHEELS.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlLFT_WHEELS.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode()
    {
        indexerMtrCtrlRGT_WHEELS.setIdleMode(IdleMode.kCoast);
        indexerMtrCtrlLFT_WHEELS.setIdleMode(IdleMode.kCoast);
    }

}