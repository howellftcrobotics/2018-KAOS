
package org.firstinspires.ftc.teamcode;

public class KILTS_AutoCore implements IAutoCore
{
    private KILTS_RobotMecCore m_robotCore;
    private OpModeConfig m_opModeConfig;

    // constructor to take op mode config and create the base robot object
    public KILTS_AutoCore(OpModeConfig opModeConfig)
    {
        m_opModeConfig = opModeConfig;
        m_robotCore = new KILTS_RobotMecCore(m_opModeConfig);
    }

    // entry point for this autonomous class
    public void runOpMode()
    {
        // make sure the robot hardware can all be found and initialized
        m_opModeConfig.LogInfo("Entry, initializing hardware...");
        if (!m_robotCore.init())
        {
            m_opModeConfig.LogError("Hardware Init Failed.");
            m_opModeConfig.OpMode.sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // initialize the devices
        m_opModeConfig.LogInfo("Initializing devices...");
        m_robotCore.InitializeGryo();

        // Wait for the game to start (driver presses PLAY)
        m_opModeConfig.LogInfo("Waiting for start...");
        m_opModeConfig.OpMode.waitForStart();
        m_opModeConfig.LogInfo("Starting...");

// do more autonomous here...
        //m_robotCore.LowerRobot();
        // m_robotCore.StrafeRight(4, .5, 2);
        // m_robotCore.TurnByEncoder(90, 0.5, 2);

        m_robotCore.LowerRobot();
        m_robotCore.StrafeLeft(2,.5,2);
        // Move forward to the minerals
        m_robotCore.DriveForward(10,.8,1);
        // Find where the gold is
        //RobotMecCore.MEC_DRIVE_AUTO_DIRECTION direction = m_robotCore.ReadMineralVision();
        {// Gold is in the middle
            //if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.CENTER) {
            // m_robotCore.StrafeLeft(4,.5,2);
            // m_robotCore.DriveForward(70, 1, 1);
            //m_robotCore.DriveBackward(10,.5,2);
            //   m_robotCore.TurnByEncoder(100,.5,2);
            //   m_robotCore.DriveForward(40,1,2);
        }
        //Gold is on the right
        //else if(direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.RIGHT)
        {
            // m_robotCore.StrafeRight(4, .5, 2);
            //m_robotCore.DriveForward(37, 1, 2);
            //m_robotCore.DriveForward(10,.5,2);
            //m_robotCore.DriveBackward(10,.5,2);
            //m_robotCore.TurnByEncoder(100,.5,2);
            //m_robotCore.DriveForward(40,1,2);
        }
        //Gold is on the left
        //else if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.LEFT) ;
        {
            //Move towards the depot to place team marker
            //m_robotCore.StrafeLeft(4, .5, 2);
            //m_robotCore.DriveForward(35, 1, 2);
            //m_robotCore.DriveBackward(10,.5,2);
            //m_robotCore.TurnByEncoder(100,.5,2);
            //m_robotCore.DriveForward(40,1,2);
        }
        // Drop off Marker
        m_robotCore.StrafeLeft(6,.5,2);
        m_robotCore.DriveForward(45, 1, 4);
        m_robotCore.DriveBackward(10,.5,2);
        m_robotCore.TurnByEncoder(90,.5,4);
        m_robotCore.DriveForward(40,1,4);
        // pull arm out to get points

        //       m_robotCore.DriveForward(24, 1, 2);
        //       m_robotCore.StrafeLeft(24, 1, 2);
        //       m_robotCore.DriveBackward(24, 1, 2);
        //       m_robotCore.StrafeRight(24, 1, 2);

        //m_opModeConfig.OpMode.sleep(1000);

        //       m_robotCore.TurnByEncoder(90, 0.5, 2);

        //m_opModeConfig.OpMode.sleep(2000);

        //       m_robotCore.StrafeDiagonalForwardLeft(42, 1, 2);
        //       m_robotCore.StrafeDiagonalForwardRight(42, 1, 2);
        //       m_robotCore.StrafeDiagonalReverseRight(42, 1, 2);
        //       m_robotCore.StrafeDiagonalReverseLeft(42, 1, 2);
    }
}
