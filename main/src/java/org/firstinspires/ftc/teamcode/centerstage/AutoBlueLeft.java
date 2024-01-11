package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.PreciseMovement;

import static org.firstinspires.ftc.teamcode.lib.RobotHardware.*;
import static org.firstinspires.ftc.teamcode.lib.PreciseMovement.preciseMovementInit;

@Autonomous(name="AutoBlueLeft", group="Autonomous")
public class AutoBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        robotHardwareInit(hardwareMap);
        preciseMovementInit();
        waitForStart();

        while (opModeIsActive()) {
            PreciseMovement.moveToPos(1, 100, 1);
            PreciseMovement.updatePos();

            telemetry.addData("x", PreciseMovement.getPos()[0]);
            telemetry.addData("y", PreciseMovement.getPos()[1]);
            telemetry.update();
        }
    }
}