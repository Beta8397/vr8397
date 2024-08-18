package org.firstinspires.ftc.teamcode.mybot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous
public class MyAutoTest extends MyAuto {

    // Create a MyBot object; we still have to initialize it in the runOpMode method
    MyBot bot = new MyBot();
    public void runOpMode() {

        // Initialize the MyBot objec
        bot.init(hardwareMap);

        // Provide MyAuto with a reference to the MyBot object
        setBot(bot);

        // Tell the MyBot object where it is starting on the field (coordinates and orientation)
        bot.setPose(0, 0, 90);

        // Wait for the start button to be pressed
        waitForStart();

        /* Drive straight from the starting position to the specified coordinates, while maintaining the
           specified orientation.
         */
        driveTo(new MotionProfile(10, 30, 10), 48, 0, 90, 1);

        // Turn to the specified heading
        turnTo(180, 90, 6, 1);

        // Drive straight to new specified position
        driveTo(new MotionProfile(10, 30, 10), 0, 48, 180, 1);

    }

}
