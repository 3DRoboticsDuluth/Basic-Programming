package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;

@TeleOp(name = "Teleop -- ConfigA", group = "Examples")
public class ConfigA extends BasicDriveOpMode {

    @Override
    public void configureFollowerConstants() {
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }
}
