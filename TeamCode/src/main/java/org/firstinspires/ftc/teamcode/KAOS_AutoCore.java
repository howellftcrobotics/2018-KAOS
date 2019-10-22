
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KAOS_AutoCore implements IAutoCore
{
    private KAOS_RobotOmniCore m_robot;
    private OpModeConfig m_opModeConfig;
    private ElapsedTime m_elapsedTime = new ElapsedTime();
    private AutoPauseSet m_autoPauseSet = new AutoPauseSet();

    // constructor to take op mode config and create the base robot object
    public KAOS_AutoCore(OpModeConfig opModeConfig)
    {
        m_opModeConfig = opModeConfig;
        m_robot = new KAOS_RobotOmniCore(m_opModeConfig);
    }

    // entry point for this autonomous class
    public void runOpMode()
    {
        // make sure the robot hardware can all be found and initialize
        m_robot.PlaySound("onestep");
        m_opModeConfig.LogInfo("Entry, initializing hardware...");
        if (!m_robot.init())
        {
            m_opModeConfig.LogError("Hardware Init Failed.");
            m_opModeConfig.OpMode.sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        try
        {
            m_robot.RaiseRobot(false);
            m_robot.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_robot.TrySetDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception ex)
        {
            // do nothing
        }

        try
        {
            m_elapsedTime.reset();
            while ((m_elapsedTime.seconds() < 30) && (m_opModeConfig.OpMode.gamepad1.right_trigger == 0) && (!m_opModeConfig.OpMode.isStarted()))
            {
                if (m_opModeConfig.OpMode.gamepad1.left_trigger > 0)
                {
                    m_opModeConfig.LogInfo("Cleared the pauses");
                    m_autoPauseSet.Reset();
                }
                else {
                    if (m_opModeConfig.OpMode.gamepad1.a) {
                        m_autoPauseSet.a = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.b) {
                        m_autoPauseSet.b = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.x) {
                        m_autoPauseSet.x = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.y) {
                        m_autoPauseSet.y = true;
                    }

                    if (m_opModeConfig.OpMode.gamepad1.dpad_down) {
                        m_autoPauseSet.dpad_down = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.dpad_up) {
                        m_autoPauseSet.dpad_up = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.dpad_right) {
                        m_autoPauseSet.dpad_right = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.dpad_left) {
                        m_autoPauseSet.dpad_left = true;
                    }
                    if (m_opModeConfig.OpMode.gamepad1.start) {
                        m_autoPauseSet.useGyro = false;
                    }
                }

                if (m_opModeConfig.OpMode.opModeIsActive()) { break; }

                m_opModeConfig.LogInfo("Current Pauses: A=" + m_autoPauseSet.GetPostionPauseA() +
                        "sec, B=" + m_autoPauseSet.GetPostionPauseB() + "sec");
                m_opModeConfig.LogInfo("Use Gyro:" + m_autoPauseSet.useGyro, true);
            }
            m_opModeConfig.LogInfo("Config time period end.");
        }
        catch (Exception ex)
        {
            // do nothing
        }

        // initialize the devices
        m_opModeConfig.LogInfo("Initializing devices...");
        try
        {
            m_robot.InitializeGryo();
        }
        catch (Exception ex)
        {
            // try again
            try
            {
                m_robot.InitializeGryo();
            }
            catch (Exception ex2)
            {
                // do nothing
            }
        }

        try
        {
            m_robot.EnableMineralVision();
        }
        catch (Exception ex)
        {
            // try again
            try
            {
                m_robot.EnableMineralVision();
            }
            catch (Exception ex2)
            {
                // do nothing
            }
        }

        m_robot.m_useGryo = true; //m_autoPauseSet.useGyro;
        KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS direction = KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.UNKNOWN;

        // set how much of the horizon we see depending on where we are starting from
        if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
        {
            // at the crater, we want to see less of the upper horizon
            m_robot.detector.yPosLimit = 250;
        }
        else if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
        {
            // seeing more of the horizon is fine in this case
            m_robot.detector.yPosLimit = 250;
        }
        else
        {
            m_robot.detector.yPosLimit = 250;
        }

        // Wait for the game to start (driver presses PLAY)
        m_opModeConfig.LogInfo("Waiting for start...");
        try
        {
             m_opModeConfig.OpMode.waitForStart();
        }
        catch (Exception ex)
        {
            m_opModeConfig.LogInfo("Exception occurred in WaitForStart():" + ex.getMessage());
            m_robot.DisableMineralVision();
            return;
        }

        m_opModeConfig.LogInfo("Starting...");
        m_opModeConfig.LogInfo("Initial Mineral Direction:" + direction.toString());
        m_opModeConfig.LogInfo("Searching for mineral...");

        m_opModeConfig.OpMode.sleep(1000);
        //m_opModeConfig.OpMode.sleep(10000);
        try
        {
            m_opModeConfig.LogInfo("Searching for mineral...");
            direction = m_robot.FindMineral();
        }
        catch (Exception ex)
        {
            m_opModeConfig.LogInfo("EXCEPTION occurred in FindMineral():" + ex.getMessage());
        }
        m_opModeConfig.LogInfo("Initial Mineral Direction:" + direction.toString());
        m_robot.LowerRobot(true);


        // TEST CODE!!!
        if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT_TEST)
        {
            // TurnByEncoder: negative goes LEFT
            // TurnByGyro: opposite of TurnByEncoder, positive goes RIGHT

            //FACT: TurnByEncoder come up shorter than TurnByGyro because travel computations

            // ******************************* TEST **************************
            m_robot.TurnByEncoder(-45, .5, 3);
            m_opModeConfig.OpMode.sleep(2000);

            m_robot.TurnByGyro(45);
            //.gyroTurn(.4, 45, 2);
            //m_robot.gyroHold(.3, 45, .75);
            m_opModeConfig.OpMode.sleep(2000);

//                m_robot.TurnByEncoder(-90, .5, 3);
//                m_opModeConfig.OpMode.sleep(2000);
//
//                m_robot.gyroTurn(.4, 90, 2);
//                m_robot.gyroHold(.3, 90, .75);
//                m_opModeConfig.OpMode.sleep(4000);

            // re-center to zero with encoder
            m_robot.TurnByGyro(0);
            //m_robot.gyroTurn(.4, 0, 3);
            //m_robot.gyroHold(.3, 0, .75);
            m_opModeConfig.OpMode.sleep(2000);

            // turn back to 90 with gyro
            m_robot.TurnByGyro(90);
            //m_robot.gyroTurn(.4, 90, 2);
            //m_robot.gyroHold(.3, 90, .75);

            // ******************************* TEST **************************
            m_robot.DisableMineralVision();
            return;
        }

        // use mineral vision to find the gold mineral
//        KAOS_RobotOmniCore.MEC_DRIVE_AUTO_DIRECTION direction = m_robot.FindMineralWithStrafe(
//           1020, .30, .48,8,
//           KAOS_TeleOp_OmniDrive.ROBOT_FRONT_MODE.PHONE);
        if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.UNKNOWN)
        {
            m_opModeConfig.LogInfo("FindMineralWithStrafe(unknown)...");
            m_robot.detector.yPosLimit = 350;
            direction = m_robot.FindMineralWithStrafe(20);
        }
        else {
            //m_opModeConfig.LogInfo("FindMineralWithStrafe()...");
            //direction = m_robot.FindMineralWithStrafe(direction, 20);
            if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.LEFT)
            {
                m_robot.StrafeLeft(12, .7, 4);
            }
            else if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.RIGHT)
            {
                m_robot.StrafeRight(16, .7, 4);
            }
        }
        m_opModeConfig.LogInfo("Found Mineral Direction:" + direction.toString());

        // if mineral vision can't find the gold mineral, default to center
        if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.UNKNOWN)
        {
            direction = KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.CENTER;
        }
        m_opModeConfig.LogInfo("Defaulting to Mineral Direction:" + direction.toString());

        // disable mineral vision so it uses less resources
        m_robot.DisableMineralVision();

        m_opModeConfig.LogInfo("Final Mineral Direction:" + direction.toString());
        //m_opModeConfig.OpMode.sleep(2000);

        // for every position, re-align before starting
        m_robot.TurnByGyro(0);

        //m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000)
        //m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseB() * 1000);

        // handle autonomous for each situation
        if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER_LAND_SAMPLE_EXTEND)
        {
            // in this situation, left, right or center does not matter, drive forward after sample, turn and extend arm
            m_robot.DriveForward(11, .5, 3);
            m_robot.TurnByGyro(-90);
        }
        else if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.CENTER)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {
                // CENTER | CRATER - tested 11/26/18

                m_robot.DriveForward(13, .5, 3);
                m_robot.DriveBackward(10, .5, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(24, .5, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(24, .5, 3);
                m_robot.TurnByGyro(132);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(36, .5, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(132);
                m_robot.DriveBackward(40, .5, 5);
                m_robot.TurnByGyro(130);
                m_robot.DriveBackward(11, .5, 5);
                m_robot.TurnByGyro(-129);
            }
            else if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER_DOUBLE_SAMPLE)
            {

                // CENTER | CRATER2 - needs completing and testing

                m_robot.TurnByGyro(0);
                m_robot.DriveForward(13, .5, 3);
                m_robot.DriveBackward(10, .6, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(43, .8, 4);
                m_robot.TurnByGyro(134);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(43, .7, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(268); // original was 260
                m_robot.DriveForward(23, .7, 3);
                m_robot.DriveBackward(12, .7, 3);
                m_robot.TurnByGyro(270); // was 270
                m_robot.StrafeLeft(18, .7, 3);
                m_robot.TurnByGyro(222); // was 134
                m_robot.StrafeLeft(38, .8, 5); //was DriveBackwards
                //m_robot.TurnByGyro(-133);
            }
            else if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {
                // CENTER | DEPOT - Tested late 11/25/18

                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);
                m_robot.DriveForward(46, .6, 4);
                m_robot.TurnByGyro(-46);
                m_robot.DropMarker();
                m_robot.DriveBackward(35, .5, 6);
                m_robot.TurnByGyro(-46);
                m_robot.DriveBackward(15, .45, 6);
                m_robot.TurnByGyro(45);
            }
        }
        else if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.RIGHT)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {
                // RIGHT | CRATER - tested 11/26/18

                m_robot.DriveForward(13, .5, 3);
                m_robot.DriveBackward(10, .5, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(30, .5, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(28, .5, 2);
                m_robot.TurnByGyro(132);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(39, .5, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(132);
                m_robot.DriveBackward(40, .5, 5);
                m_robot.TurnByGyro(130);
                m_robot.DriveBackward(11, .5, 5);
                m_robot.TurnByGyro(-129);
            }
            else if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER_DOUBLE_SAMPLE)
            {
                // RIGHT | CRATER2 - needs completing and testing

                m_robot.TurnByGyro(0);
                m_robot.DriveForward(13, .7, 3);
                m_robot.DriveBackward(10, .8, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(60, .9, 3);
                m_robot.TurnByGyro(131);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(45, .7, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(291);  // angle toward right mineral (left from this aspect)
                m_robot.DriveForward(27, .7, 3);
                //m_robot.DriveBackward(7, .7, 3);
                m_robot.TurnByGyro(275); // was 275
                m_robot.StrafeLeft(21, .7, 3); //was 18 before backwards was removed
                m_robot.TurnByGyro(222);
                m_robot.StrafeLeft(10, .8, 5);
                //m_robot.TurnByGyro(-133);
            }
            else  if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {
                // RIGHT | DEPOT - tested late 11/25/18

                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);
                m_robot.DriveForward(36, .6, 4);
                m_robot.TurnByGyro(45);
                m_robot.DriveForward(14, .5, 4);
                m_robot.DropMarker();
                m_robot.TurnByGyro(-48);
                m_robot.DriveBackward(40, .5, 6);
                m_robot.TurnByGyro(-48);
                m_robot.DriveBackward(15, .45, 6);
                m_robot.TurnByGyro(49);
            }
        } else if (direction == KAOS_RobotOmniCore.MINERAL_VISION_DIRECTIONS.LEFT)
        {
            if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER)
            {
                // LEFT | CRATER - tested - 11/26/18

                m_robot.DriveForward(13, .5, 3);
                m_robot.DriveBackward(10, .5, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(35, .5, 3);
                m_robot.TurnByGyro(132);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(36, .5, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(132);
                m_robot.DriveBackward(40, .5, 5);
                m_robot.TurnByGyro(130);
                m_robot.DriveBackward(11, .5, 5);
                m_robot.TurnByGyro(-129);
            }
            else if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.CRATER_DOUBLE_SAMPLE)
            {
                // LEFT | CRATER - needs completing and testing

                m_robot.TurnByGyro(0);
                m_robot.DriveForward(13, .6, 3);
                m_robot.DriveBackward(10, .7, 3);
                m_robot.TurnByGyro(-7);
                m_robot.StrafeLeft(33, .8, 3);
                m_robot.TurnByGyro(133);

                // pause before heading to the depot?
                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);

                m_robot.DriveForward(42, .7, 3);
                m_robot.DropMarker();
                m_robot.TurnByGyro(236);  // angle toward left mineral (right from this aspect) //  WAS 240 (then 238)
                m_robot.DriveForward(27, .7, 3);
                m_robot.DriveBackward(12, .7, 3);
                m_robot.TurnByGyro(275); // was 275
                m_robot.StrafeLeft(18, .7, 3);
                m_robot.TurnByGyro(222);
                m_robot.StrafeLeft(41, .8, 5);
                //m_robot.TurnByGyro(-133);
            }
            else  if (m_opModeConfig.StartingPosition == OpModeConfig.ROBOT_STARTING_POSITION.DEPOT)
            {
                // LEFT | DEPOT -  tested on 11/25/18

                m_opModeConfig.OpMode.sleep(m_autoPauseSet.GetPostionPauseA() * 1000);
                m_robot.DriveForward(33, .5, 4);
                m_robot.TurnByGyro(-48);
                m_robot.DriveForward(14, .5, 4);
                m_robot.DropMarker();
                m_robot.TurnByGyro(-48);
                m_robot.DriveBackward(35, .5, 6);
                m_robot.TurnByGyro(-48);
                m_robot.DriveBackward(16, .5, 6);
                m_robot.TurnByGyro(45);
            }
        }

        // do these at the end of each mode
        m_robot.ExtendArmIntoCrater();
        m_robot.StrafeRight(6, .8, 1);
        m_robot.StrafeLeft(10, .8, 1);
        RobotUtil.TryControlMotorByTime(m_robot.mineralMotor, m_opModeConfig, .8, 4);
    }
}
