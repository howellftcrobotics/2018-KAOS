package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="KRASH_Crater_Config", group="KRASH")
@Disabled
public class KRASH_Crater_Config extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        // setup the properties of this OpMode
        OpModeConfig opModeConfig = new OpModeConfig(
                OpModeConfig.ROBOT_TEAM.KRASH,
                OpModeConfig.ROBOT_OPERATING_MODE.AUTONOMOUS,
                OpModeConfig.ROBOT_STARTING_POSITION.CRATER,
                telemetry, hardwareMap, this);
        opModeConfig.runOpMode();
    }
}