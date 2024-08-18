package org.firstinspires.ftc.teamcode.mybot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous
public class MyAutoTest extends MyAuto {

    MyBot bot = new MyBot();
    public void runOpMode(){
        bot.init(hardwareMap);
        setBot(bot);
        bot.setPose(0, 0, 90);

        waitForStart();

        driveTo(new MotionProfile(10, 30, 10), 48, 0, 90,1);

        turnTo(180, 90, 6,2);

        driveTo(new MotionProfile(10, 30, 10), 0, 48, 180, 1);
    }

}
