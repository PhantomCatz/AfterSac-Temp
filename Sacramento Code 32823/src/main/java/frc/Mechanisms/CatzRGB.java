package frc.Mechanisms;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.PDPSim;

import java.util.Random;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;


public class CatzRGB 
{
    Random rand;

  
    private final int LED_COUNT = 54;

    private final int LED_PWM_PORT = 6;
    
    private AddressableLED led;   

    private AddressableLEDBuffer ledBuffer;
    
    
    private Color RED    = Color.kRed;
    private Color ORANGE = Color.kOrange;
    private Color YELLOW = Color.kYellow;
    private Color GREEN  = Color.kGreen;
    private Color BLUE   = Color.kBlue;
    private Color PURPLE = Color.kPurple;
    private Color WHITE  =  new Color(128,128,128);
    private Color TEAM_COLOR = Color.kDodgerBlue;
    private Color PINK  = Color.kHotPink;   
   

    private int RandomDelayCounter = 0;
    private int FlashDelayCounter = 0;
    private int FLowDelayCounter = 0;
    private int PingpongDelayCounter = 0;

    private final int RandomDelay = 15;
    private final int FLASH_DELAY   = 15;
    private final int FLOW_DELAY   = 3;
    private final int PINGPONG_DELAY   = 1;
    private int m_rainbowFirstPixelHue = 0;

    private final double PDH_MAX_PWR = 500.0;

    private final int PINGPONG_RADIUS = 5;
    private final int FLOW_WIDTH = 6;

    private int flowCounter = 0;
    private int PingpongCounter = 0;

    private boolean PingpongUp = true;

    private int nextFlashColor = 1;


    public  boolean robotDisabled = false;
    public  boolean coneOnboard = false;
    public  boolean cubeOnboard   = false;
    public  boolean cubeRequest   = false;
    public  boolean coneRequest   = true;
    public  boolean inAuton       = false;
    public  boolean autobalancing = false;
    public  boolean noGamePiece   = false;
    public  boolean endGame       = false;
    public  boolean matchDone     = false;


    private final int FLOW3_DELAY = 30;
    private final int FLOW3_WIDTH = 5;

    public CatzRGB()
    {
        rand = new Random();
        led = new AddressableLED(LED_PWM_PORT);

        ledBuffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
      

    }


    public void LEDWork()
    {
        if(matchDone == true)
        {
            rainbow();
        }
        else if(Robot.isInDisabled() == true)
        {
            //solidColor(TEAM_COLOR);
            if(Robot.paths.chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
            {
                pingpong(TEAM_COLOR, RED);
                // flowUp(RED, RED, YELLOW); //CHINA
                
            }
            else
            {
                pingpong(RED, TEAM_COLOR);
                // flowUp(RED, WHITE, BLUE); //USA
            }
           
        }
        else if(Robot.isInAuton() == true)
        {
            if(Robot.balance.startBalance == true)
            {
                solidColor(GREEN);
            }
            else
            {
                flowUp(TEAM_COLOR,WHITE);
            }
        }
        else if(endGame == true)
        {
            if(Robot.paths.chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
            {
                flash2(new Color(232,12,12),new Color(135,12,12), new Color(214,94,94));
            }
            else
            {
                flash2(new Color(50,167,220),new Color(7,72,242), new Color(8,8,138));
            }
        }
        else 
        {
            if(Robot.paths.chosenAllianceColor.getSelected() == Robot.constants.RED_ALLIANCE)
            {
                solidColor(RED);
            }
            else
            {
                solidColor(TEAM_COLOR);
            }
            //progressBar(YELLOW,PURPLE,Robot.PDH.getTotalPower()/PDH_MAX_PWR*2.0-1.0);
        }
        
        led.setData(ledBuffer);
    }

    public void flowUp(Color color1, Color color2, Color color3)
    {
        FLowDelayCounter++;
        if(FLowDelayCounter > FLOW3_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
            {
                if((i - flowCounter) % (FLOW3_WIDTH * 3) < FLOW3_WIDTH)
                {
                    ledBuffer.setLED(i, color1);
                }
                else if ((i - flowCounter) % (FLOW3_WIDTH * 3) < FLOW3_WIDTH * 2)
                {
                    ledBuffer.setLED(i, color2);

                } else {
                    ledBuffer.setLED(i, color3);
                }
            }

            flowCounter++;
            if(flowCounter >= FLOW3_WIDTH * 3) 
            {
                flowCounter = 0;
            }

            FLowDelayCounter = 0;
        }
    }
    public void flowUp(Color color1, Color color2)
    {
        FLowDelayCounter++;
        if(FLowDelayCounter > FLOW_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
        {
            if((i-flowCounter) % (FLOW_WIDTH*2) < FLOW_WIDTH)
            {
                ledBuffer.setLED(i, color1);
            }
            else
            {
                ledBuffer.setLED(i, color2);
            }
        }

        flowCounter++;
        if(flowCounter >= (FLOW_WIDTH*2))
        {
            flowCounter = 0;
        }

            FLowDelayCounter = 0;
        }
    }

    public void flowDown(Color color1, Color color2)
    {
        FLowDelayCounter++;
        if(FLowDelayCounter > FLOW_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
        {
            if((i+flowCounter) % (FLOW_WIDTH*2) < FLOW_WIDTH)
            {
                ledBuffer.setLED(i, color1);
            }
            else
            {
                ledBuffer.setLED(i, color2);
            }
        }

        flowCounter++;
        if(flowCounter >= (FLOW_WIDTH*2))
        {
            flowCounter = 0;
        }

            FLowDelayCounter = 0;
        }
    }

    public void flash(Color color1, Color color2)
    {
        FlashDelayCounter++;
        if(FlashDelayCounter > FLASH_DELAY)
        {
            if(nextFlashColor == 1)
            {
                solidColor(color1);
                nextFlashColor = 2;
            }
            else
            {
                solidColor(color2);
                nextFlashColor = 1;
            }

            FlashDelayCounter = 0;
        }
    }

    public void flash2(Color color1, Color color2, Color color3)
    {
        FlashDelayCounter++;
        if(FlashDelayCounter > FLASH_DELAY)
        {
            if(nextFlashColor == 1)
            {
                solidColor(color1);
                nextFlashColor = 2;
            }
            else if(nextFlashColor == 2)
            {
                solidColor(color2);
                nextFlashColor = 3;
            }
            else
            {
            solidColor(color3);
            nextFlashColor = 1;
            }

            FlashDelayCounter = 0;
        }
    }


    public void pingpong(Color color1, Color color2)
    {
        PingpongDelayCounter++;
        if(PingpongDelayCounter > PINGPONG_DELAY)
        {
            for(int i = 0; i < LED_COUNT; i++)
            {
                if(i<=(PingpongCounter+PINGPONG_RADIUS) && i>=(PingpongCounter-PINGPONG_RADIUS))
                {
                    ledBuffer.setLED(i, color1);
                }
                else
                {
                    ledBuffer.setLED(i, color2);
                }
            }

            if(PingpongUp)
            {
                PingpongCounter++;
            }
            else
            {
                PingpongCounter--;
            }

            if(PingpongCounter >= LED_COUNT)
            {
                PingpongUp = false;
            }
            else if(PingpongCounter < 0)
            {
                PingpongUp = true;
            }

            PingpongDelayCounter = 0;
        }
    }

    public void solidColor(Color color)
    {
        for(int i = 0; i < LED_COUNT; i++)
        {
            ledBuffer.setLED(i, color);
        }
    }

    public void rainbow() 
    {
        // For every pixel
        for (int i = 0; i < ledBuffer.getLength(); i++) {

          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

          ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 1;
        m_rainbowFirstPixelHue %= 180;
    }

    public void random()
    {
        RandomDelayCounter++;
        if(RandomDelayCounter > RandomDelay)
        {
            for(int j = 0; j < LED_COUNT; j++)
            {
                    Color color1 = new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
                    ledBuffer.setLED(j, color1);
            }
            RandomDelayCounter = 0;
        }
        

    }

    public void Rainbow()
    {
            FlashDelayCounter++;
            if (FlashDelayCounter > FLASH_DELAY);
            {
                if (nextFlashColor == 1)
                {
                    solidColor(RED);
                    nextFlashColor = 2;
                    
                }
                else if(nextFlashColor==2)
                {
                    solidColor(ORANGE);
                    nextFlashColor=3;
                   
                }
                else if(nextFlashColor==3)
                {
                    solidColor(YELLOW);
                    nextFlashColor=4;
                   
                }
                else if(nextFlashColor==4)
                {
                    solidColor(GREEN);
                    nextFlashColor=5;
                  
                }
                else if(nextFlashColor==5)
                {
                    solidColor(BLUE);
                    nextFlashColor=6;
        
                }
                else if(nextFlashColor==6)
                {
                    solidColor(PURPLE);
                    nextFlashColor=1;
                   
                }
                FlashDelayCounter=0;
            }    
        }

        public void progressBar(Color color1,Color color2, double val) //val -1 ~ 1
    {
        if(val>1)
        {
            val = 1;
        }else if(val<-1)
        {
            val = -1;
        }
        val = (val + 1.0)/2.0;
        int progressBarOnNum = (int)(val * (double)LED_COUNT);
        for(int i = 0; i < LED_COUNT; i++)
        {
            if(i > (LED_COUNT - progressBarOnNum))
            {
                ledBuffer.setLED(i, color1);
            }else
            {
                ledBuffer.setLED(i, color2);
            }
        }
    }

    
}