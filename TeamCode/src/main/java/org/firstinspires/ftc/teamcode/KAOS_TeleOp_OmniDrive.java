package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp OmniDrive", group="KAOS - TeleOp")
//@Disabled
public class KAOS_TeleOp_OmniDrive extends LinearOpMode
{
    private KAOS_RobotOmniCore m_robot;
    private OpModeConfig m_opModeConfig;

    private ElapsedTime runtime = new ElapsedTime();

    public KAOS_TeleOp_OmniDrive()
    {
        // set up the default config for teleOp
        m_opModeConfig = new OpModeConfig(
            OpModeConfig.ROBOT_TEAM.KAOS,
            OpModeConfig.ROBOT_OPERATING_MODE.TELEOP,
            OpModeConfig.ROBOT_STARTING_POSITION.DEPOT,
            telemetry, hardwareMap, this);
        m_robot = new KAOS_RobotOmniCore(m_opModeConfig);
    }

    // for a controlled reduction of power
    private enum POWER_MODE
    {
        FULL,
        REDUCED
    }
    private KAOS_TeleOp_OmniDrive.POWER_MODE m_powerMode = KAOS_TeleOp_OmniDrive.POWER_MODE.FULL;

    // so we can change what is considered the "front" of the robot to making driving it easier
    public enum ROBOT_FRONT_MODE
    {
        PHONE,
        ELEVATOR_BASE,
        LIFT_BASE,
        INTAKE
    }
    private KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE m_frontMode = KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.INTAKE;

    @Override
    public void runOpMode()
    {
        if (!m_robot.init()) {
            telemetry.addData("Error", "Hardware Init Failed");
            telemetry.update();
            sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // set all of the motors to run without encoders during teleop
        try
        {
            m_robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_robot.mineralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_robot.extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_robot.TrySetDriveMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_robot.TrySetDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.mineralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception ex)
        {
            // something went wrong
            m_opModeConfig.LogError("Setting motors to run without encoders Exception: " + ex.getMessage());
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // set the stateful power mode (defaults to full power)
            if ((gamepad1.left_bumper) && (m_powerMode != POWER_MODE.FULL))
            {
                m_powerMode = POWER_MODE.FULL;
                m_opModeConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
            }
            else if ((gamepad1.right_bumper) && (m_powerMode != POWER_MODE.REDUCED))
            {
                m_powerMode = POWER_MODE.REDUCED;
                m_opModeConfig.LogInfo("PowerMode: " + m_powerMode.toString(), true);
            }

            // determine which side is the front
            if (gamepad1.a)
            {
                m_frontMode = ROBOT_FRONT_MODE.ELEVATOR_BASE;
            }
            else if (gamepad1.b)
            {
                m_frontMode = ROBOT_FRONT_MODE.INTAKE;
            }
            else if (gamepad1.x)
            {
                m_frontMode = ROBOT_FRONT_MODE.LIFT_BASE;
            }
            else if (gamepad1.y)
            {
                m_frontMode = ROBOT_FRONT_MODE.PHONE;
            }
            else
            {
                // it stays what it was on the last round
            }

            // strafe drive the bot based on the correct power
            try
            {
                if (m_powerMode == KAOS_TeleOp_OmniDrive.POWER_MODE.FULL)
                {
                    m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x,
                            gamepad1.left_stick_x, 1, m_frontMode); // was .90
                }
                else
                {
                    m_robot.TeleOpStrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x,
                            gamepad1.left_stick_x, .33, m_frontMode);
                }
            }
            catch (Exception ex)
            {
                // something went wrong
                m_opModeConfig.LogError("TeleOp Error 1 : Exception: " + ex.getMessage());
            }

            // control the lift motor
            try
            {
                if (m_robot.liftMotor != null)
                {
                    m_robot.liftMotor.setPower(gamepad2.left_stick_y); // * .95);
                }
            }
            catch (Exception ex)
            {
                // something went wrong
                m_opModeConfig.LogError("TeleOp Error 2 : Exception: " + ex.getMessage());
            }

            // control the extender motor
            try
            {
                if (m_robot.extenderMotor != null)
                {
                    m_robot.extenderMotor.setPower(-gamepad2.right_stick_y); // * .90);
                }
            }
            catch (Exception ex)
            {
                // something went wrong
                m_opModeConfig.LogError("TeleOp Error 3 : Exception: " + ex.getMessage());
            }

            try
            {
                // control the intake motor
                if (m_robot.mineralMotor != null)
                {
                    if (gamepad2.left_trigger > 0)
                    {
                        m_robot.mineralMotor.setPower(-gamepad2.left_trigger); // * .85);
                    } else if (gamepad2.right_trigger > 0)
                    {
                        m_robot.mineralMotor.setPower(gamepad2.right_trigger); // * .85);
                    } else
                        {
                        m_robot.mineralMotor.setPower(0);
                    }
                }
            }
            catch (Exception ex)
            {
                // something went wrong
                m_opModeConfig.LogError("TeleOp Error 4 : Exception: " + ex.getMessage());
            }

            // raise or lower the robot
            try
            {
                if (gamepad2.y)
                {
                    m_robot.RaiseRobot(true);
                }
                else if (gamepad2.a)
                {
                    m_robot.LowerRobot(false);
                }
            }
            catch (Exception ex)
            {
                // something went wrong
                m_opModeConfig.LogError("TeleOp Error 5 : Exception: " + ex.getMessage());
            }
        }
    }
}
