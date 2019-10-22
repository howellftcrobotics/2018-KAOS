
package org.firstinspires.ftc.teamcode;

public class KUDOS_AutoCore implements IAutoCore
{
    private KUDOS_RobotMecCore m_robotCore;
    private OpModeConfig m_opModeConfig;

    // constructor to take op mode config and create the base robot object
    public KUDOS_AutoCore(OpModeConfig opModeConfig)
    {
        m_opModeConfig = opModeConfig;
        m_robotCore = new KUDOS_RobotMecCore(m_opModeConfig);
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

        //changed mineral code as lift/lower is on front
        /*
        m_robotCore.LowerRobot();
        // m_robotCore.TurnByEncoder
        m_robotCore.DriveForward(65, .5, 3);
        m_robotCore.DriveBackward(8.5,.5,3);
        m_robotCore.TurnByEncoder(90, .5, 2);
        m_robotCore.DriveForward(16, .5, 3);
        m_robotCore.TurnByEncoder(45, .5, 2);
        m_robotCore.StrafeLeft(4, .5, 2);
        m_robotCore.StrafeRight(.5, .5, 2);
        m_robotCore.DriveForward(83, 1, 3);
        */
        //right
        m_robotCore.LowerRobot();
        m_robotCore.TurnByEncoder(30,.5,3);
        m_robotCore.DriveForward(30, .5, 3);
        m_robotCore.TurnByEncoder(-50,.5,2);
        m_robotCore.DriveForward(26, .5, 2);
        m_robotCore.DriveBackward(26, .5, 2);
        m_robotCore.TurnByEncoder(140,.5,2);
        m_robotCore.DriveForward(96,.5,4);
        /*
        //left
        m_robotCore.LowerRobot();
        m_robotCore.TurnByEncoder(-30,.5,2);
        m_robotCore.DriveForward(30, .5, 2);
        m_robotCore.TurnByEncoder(50,.5,2);
        m_robotCore.DriveForward(36, .5, 2);
        m_robotCore.DriveBackward(5, .5, 2);
        m_robotCore.StrafeRight(5, .5, 2);
        m_robotCore.DriveForward(5, .5, 2);
        m_robotCore.TurnByEncoder(140, .5, 2);
        m_robotCore.DriveForward(96,.5,4);
        */

        //RobotMecCore.MEC_DRIVE_AUTO_DIRECTION direction = m_robotCore.ReadMineralVision();

        /*if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.CENTER)
        {
            m_robotCore.LowerRobot();
            // m_robotCore.TurnByEncoder
            m_robotCore.DriveForward(65, .5, 3);
            m_robotCore.DriveBackward(8.5,.5,3);
            m_robotCore.TurnByEncoder(90, .5, 2);
            m_robotCore.DriveForward(16, .5, 3);
            m_robotCore.TurnByEncoder(45, .5, 2);
            m_robotCore.StrafeLeft(4, .5, 2)
            m_robotCore.DriveForward(83, 1, 3);
        }
        else if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.RIGHT)
        {
            m_robotCore.LowerRobot();
            m_robotCore.TurnByEncoder(30,.5,3);
            m_robotCore.DriveForward(30, .5, 3);
            m_robotCore.TurnByEncoder(-120,.5,2);
            m_robotCore.DriveForward(24, .5, 2);
            m_robotCore.DriveBackward(24, .5, 2);
            m_robotCore.TurnByEncoder(180,.5,2);
            m_robotCore.DriveForward(96,.5,2);
        }
        else if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.LEFT)
        {
            m_robotCore.LowerRobot();
            m_robotCore.TurnByEncoder(-30,.5,2);
            m_robotCore.DriveForward(30, .5, 2);
            m_robotCore.TurnByEncoder(120,.5,2);
            m_robotCore.DriveForward(36, .5, 2);
            m_robotCore.DriveBackward(5, .5, 2);
            m_robotCore.StrafeRight(5, .5, 2);
            m_robotCore.DriveForward(5, .5, 2);
            m_robotCore.TurnByEncoder(90, .5, 2);
            m_robotCore.DriveForward(96,.5,2);
        }
        /*
        m_robotCore.DriveForward(24, 1, 2);
        m_robotCore.StrafeLeft(24, 1, 2);
        m_robotCore.DriveBackward(24, 1, 2);
        m_robotCore.StrafeRight(24, 1, 2);

        m_opModeConfig.OpMode.sleep(1000);

        m_robotCore.TurnByEncoder(90, 0.5, 2);

        m_opModeConfig.OpMode.sleep(2000);

        m_robotCore.StrafeDiagonalForwardLeft(42, 1, 2);
        m_robotCore.StrafeDiagonalForwardRight(42, 1, 2);
        m_robotCore.StrafeDiagonalReverseRight(42, 1, 2);
        m_robotCore.StrafeDiagonalReverseLeft(42, 1, 2);
        */
    }
}
