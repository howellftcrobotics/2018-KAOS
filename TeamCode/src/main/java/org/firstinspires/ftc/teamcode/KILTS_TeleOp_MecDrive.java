package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="KILTS_TeleOp_MecDrive", group="KILTS")
@Disabled
public class KILTS_TeleOp_MecDrive extends LinearOpMode
{
    private enum POWER_MODE
    {
        FULL,
        REDUCED
    }

    private POWER_MODE m_powerMode = POWER_MODE.FULL;
    private KILTS_RobotMecCore m_robot;
    private OpModeConfig m_opModeConfig;
    private ElapsedTime m_runtime = new ElapsedTime();

    public KILTS_TeleOp_MecDrive()
    {
        m_opModeConfig = new OpModeConfig(
            OpModeConfig.ROBOT_TEAM.KILTS,
            OpModeConfig.ROBOT_OPERATING_MODE.TELEOP,
            OpModeConfig.ROBOT_STARTING_POSITION.UNKNOWN,
            telemetry, hardwareMap, this);
        m_robot = new KILTS_RobotMecCore(m_opModeConfig);
    }

    // main entry point for this class
    @Override
    public void runOpMode()
    {
        // initialize the the logger with the current team name
        RobotLogger.InitRobotLog("Team:XXX", telemetry, RobotLogger.SEVERITY.INFO);

        m_opModeConfig.LogInfo("Entry");
        if (!m_robot.init())
        {
            m_opModeConfig.LogError("Hardware Init Failed");
            sleep(10000);  // sleep for 10 seconds so the error can be read on the phone
            return;
        }

        // initialize the devices
        m_opModeConfig.LogInfo("Init all devices...");
        m_robot.InitializeGryo();
        m_robot.TrySetDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        m_opModeConfig.LogInfo("Waiting for start...");
        waitForStart();
        m_opModeConfig.LogInfo("Starting...");

        // set the watch values, which are constantly repeat logged values
        m_opModeConfig.InitWatchValues(OpModeConfig.WATCHES_MOTOR_POWER, OpModeConfig.WATCHES_COUNTS_LEFT_FRONT,
            OpModeConfig.WATCHES_COUNTS_RIGHT_FRONT, OpModeConfig.WATCHES_COUNTS_LEFT_REAR, OpModeConfig.WATCHES_COUNTS_RIGHT_REAR);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // set the stateful power mode (defaults to full power)
//            if ((gamepad1.left_bumper) && (m_powerMode != POWER_MODE.FULL))
//            {
//                m_powerMode = POWER_MODE.FULL;
//                m_opModeConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
//            }
//            else if ((gamepad1.right_bumper) && (m_powerMode != POWER_MODE.REDUCED))
//            {
//                m_powerMode = POWER_MODE.REDUCED;
//                m_opModeConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
//            }

            // strafe drive the bot based on the correct power mode
            if (m_powerMode == POWER_MODE.FULL)
            {
                m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1.0);
            }
            else
            {
                m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, .33);
            }

            // control the extender arm
            m_robot.extenderMotor.setPower(-gamepad2.left_stick_y);
//            if (gamepad2.left_stick_y)
//            {
//                m_robot.extenderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                m_robot.extenderMotor.setPower(.5);
//            }
//            else if (gamepad2.right_bumper)
//            {
//                m_robot.extenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                m_robot.extenderMotor.setPower(.5);
//            }
//            else
//            {
//                m_robot.extenderMotor.setPower(0);
//            }

            // control the mineral intake
            if (gamepad2.left_trigger > 0)
            {
                m_robot.mineralMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                m_robot.mineralMotor.setPower(1);
            }
            else if (gamepad2.right_trigger > 0)
            {
                m_robot.mineralMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                m_robot.mineralMotor.setPower(1);
            }
            else
            {
                m_robot.mineralMotor.setPower(0);
            }

            // control the grabber servo
            if (gamepad2.right_bumper)
            {
                m_robot.OpenGrabber();
            }
            else
            {
                m_robot.CloseGrabber();
            }

            // control the lift motors
            m_robot.leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            m_robot.rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            m_robot.leftLiftMotor.setPower(-gamepad2.right_stick_y);
            m_robot.rightLiftMotor.setPower(gamepad2.right_stick_y);
//            if (gamepad2.right_stick_y)
//            {
//                m_robot.leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                m_robot.rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                m_robot.leftLiftMotor.setPower(.5);
//                m_robot.rightLiftMotor.setPower(.5);
//            }
//            else if (gamepad1.dpad_down)
//            {
//                m_robot.leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//                m_robot.rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                m_robot.leftLiftMotor.setPower(.5);
//                m_robot.rightLiftMotor.setPower(.5);
//            }
//            else
//            {
//                m_robot.leftLiftMotor.setPower(0);
//                m_robot.rightLiftMotor.setPower(0);
//            }

        }
        m_opModeConfig.LogInfo("Exit while loop...");
    }
}
