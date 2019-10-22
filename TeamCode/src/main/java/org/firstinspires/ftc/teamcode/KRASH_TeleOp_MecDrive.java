package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="KRASH_TeleOp_MecDrive", group="KRASH")
@Disabled
public class KRASH_TeleOp_MecDrive extends LinearOpMode
{
    private enum POWER_MODE
    {
        FULL,
        REDUCED
    }

    private POWER_MODE m_powerMode = POWER_MODE.FULL;
    private KRASH_RobotMecCore m_robot;
    private OpModeConfig m_coreConfig;
    private ElapsedTime m_runtime = new ElapsedTime();

    public KRASH_TeleOp_MecDrive()
    {
        m_coreConfig = new OpModeConfig(
            OpModeConfig.ROBOT_TEAM.KRASH,
            OpModeConfig.ROBOT_OPERATING_MODE.TELEOP,
            OpModeConfig.ROBOT_STARTING_POSITION.UNKNOWN,
            telemetry, hardwareMap, this);
        m_robot = new KRASH_RobotMecCore(m_coreConfig);
    }

    // main entry point for this class
    @Override
    public void runOpMode()
    {
        // initialize the the logger with the current team name
        RobotLogger.InitRobotLog("Team:XXX", telemetry, RobotLogger.SEVERITY.INFO);

        m_coreConfig.LogInfo("Entry");
        if (!m_robot.init())
        {
            m_coreConfig.LogError("Hardware Init Failed");
            sleep(10000);  // sleep for 10 seconds so the error can be read on the phone
            return;
        }

        // initialize the devices
        m_coreConfig.LogInfo("Init all devices...");
        m_robot.InitializeGyro();
        m_robot.TrySetDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        m_coreConfig.LogInfo("Waiting for start...");
        waitForStart();
        m_coreConfig.LogInfo("Starting...");

        // set the watch values, which are constantly repeat logged values
        m_coreConfig.InitWatchValues(OpModeConfig.WATCHES_MOTOR_POWER, OpModeConfig.WATCHES_COUNTS_LEFT_FRONT,
            OpModeConfig.WATCHES_COUNTS_RIGHT_FRONT, OpModeConfig.WATCHES_COUNTS_LEFT_REAR, OpModeConfig.WATCHES_COUNTS_RIGHT_REAR);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            // set the stateful power mode (defaults to full power)
            if ((gamepad1.left_bumper) && (m_powerMode != POWER_MODE.FULL))
            {
                m_powerMode = POWER_MODE.FULL;
                m_coreConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
            }
            else if ((gamepad1.right_bumper) && (m_powerMode != POWER_MODE.REDUCED))
            {
                m_powerMode = POWER_MODE.REDUCED;
                m_coreConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
            }

            // strafe drive the bot based on the correct power mode
            if (m_powerMode == POWER_MODE.FULL)
            {
                m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1.0);
            }
            else
            {
                m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, .33);
            }

            // what are we going to write next for TeleOP?

        }
        m_coreConfig.LogInfo("Exit while loop...");
    }
}
