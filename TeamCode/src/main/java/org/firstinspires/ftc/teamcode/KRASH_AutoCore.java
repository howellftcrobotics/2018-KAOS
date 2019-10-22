
package org.firstinspires.ftc.teamcode;

public class KRASH_AutoCore implements IAutoCore
{
    private KRASH_RobotMecCore m_robotCore;
    private OpModeConfig m_opModeConfig;

    // constructor to take op mode config and create the base robot object
    public KRASH_AutoCore(OpModeConfig opModeConfig)
    {
        m_opModeConfig = opModeConfig;
        m_robotCore = new KRASH_RobotMecCore(m_opModeConfig);
    }

    // entry point for this autonomous class
    public void runOpMode() {
        // make sure the robot hardware can all be found and initialized
        m_opModeConfig.LogInfo("Entry, initializing hardware...");
        if (!m_robotCore.init()) {
            m_opModeConfig.LogError("Hardware Init Failed.");
            m_opModeConfig.OpMode.sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // initialize the devices
        m_opModeConfig.LogInfo("Initializing devices...");
        m_robotCore.InitializeGyro();

        // Wait for the game to start (driver presses PLAY)
        m_opModeConfig.LogInfo("Waiting for start...");
        m_opModeConfig.OpMode.waitForStart();
        m_opModeConfig.LogInfo("Starting...");

        // add code to LowerRobot to make it work properly
        m_robotCore.LowerRobot();

        // turn and face minerals

        // line up with golden mineral and remember which way we went
        KRASH_RobotMecCore.MINERAL_VISION_DIRECTIONS direction = m_robotCore.FindMineral(10);
        m_opModeConfig.LogInfo("direction:" + direction.toString());

        // drive forward some amount??

        if (direction == KRASH_RobotMecCore.MINERAL_VISION_DIRECTIONS.CENTER)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {

            }else  if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {


            } else
            {
                // error
            }

        } else if (direction == KRASH_RobotMecCore.MINERAL_VISION_DIRECTIONS.RIGHT)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {

            }else  if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {


            } else
            {
                // error
            }
        } else if (direction == KRASH_RobotMecCore.MINERAL_VISION_DIRECTIONS.LEFT)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {

            }else  if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {


            } else
            {
                // error
            }
        }


// do more autonomous here...
        //m_robotCore.LowerRobot();
        //turn -90
        //turning after droping off
        //m_robotCore.TurnByEncoder(-90, 0.5, 3);
        //the lander




        //!!TODO MIIDDLE GOLD DEPOT

        //NOW PLACE HOLDER FOR MINARAL IN MIDDLE NO VISON YET
        // m_robotCore.DriveForward(50, 1.00,3);
        //for deploying team marker
        m_opModeConfig.OpMode.sleep(3000);
        //m_robotCore.DriveBackward(5,0.5,1);
        //m_robotCore.TurnByEncoder(90, 0.5, 2);
        //getting to position for crater course ajustment
        //m_robotCore.DriveForward(10, 0.5,3);
        //crater course ajustment
        // m_robotCore.TurnByEncoder(45,0.5,3);
        //crater injetion burn
        //m_robotCore.DriveForward(70, 1.0,3);



        //!! TODO Left Gold Spot

        // m_robotCore.DriveForward(25, 1.00,5);
        //m_robotCore.TurnByEncoder(45,0.5,5);
        ///m_robotCore.DriveForward(24,1.0,5);
        // m_opModeConfig.OpMode.sleep(3000);
        //m_robotCore.DriveBackward(8,0.5,5);
        //m_robotCore.TurnByEncoder(90,0.5,2);
        //m_robotCore.DriveForward(90,1.0,5);


        /// TODO right gold stuff


        //m_robotCore.DriveForward(29,1.00,5);
        //m_robotCore.TurnByEncoder(45,1.00,5);
        //m_robotCore.DriveForward(2,5.0,2);
        //m_robotCore.DriveBackward(27,1.00,5);



        // m_robotCore.TurnByEncoder(90, 0.5, 2);
        //RobotMecCore.MEC_DRIVE_AUTO_DIRECTION direction = m_robotCore.ReadMineralVision();

        // if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.CENTER)

        // m_robotCore.DriveForward(16, 1, 2);

        // else if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.RIGHT)


        // }else if (direction == RobotMecCore.MEC_DRIVE_AUTO_DIRECTION.LEFT)


        //  m_robotCore.DriveForward(24, 1, 2);
        //  m_robotCore.StrafeLeft(24, 1, 2);
        // m_robotCore.DriveBackward(24, 1, 2);
        // m_robotCore.StrafeRight(24, 1, 2);

        //m_opModeConfig.OpMode.sleep(1000);

        //m_robotCore.TurnByEncoder(90, 0.5, 2);

        //m_opModeConfig.OpMode.sleep(2000);

        // m_robotCore.StrafeDiagonalForwardLeft(42, 1, 2);
        //  m_robotCore.StrafeDiagonalForwardRight(42, 1, 2);
        // m_robotCore.StrafeDiagonalReverseRight(42, 1, 2);
        //  m_robotCore.StrafeDiagonalReverseLeft(42, 1, 2);
    }
}
