# Swerve-Drive-2022
FRC Team 3863's Swerve Drive Code for 2022

This code was written for our prototype swerve module, but documented so that it could be used with SDS modules or after alterations in the future.
The prototype is using a Neo with a Spark Max for the drive motor and a 775 Pro with a Talon SRX for steering.

The project includes:
- Fully working Field-Oriented Swerve during Teleop using SwerveDriveKinematics.
- Our own vector-based odometry (after getting weird and unexplainable values from SwerveDriveOdometry).
- PathPlanner trajectory following for fully working autonomous path following (Auto values are WPI).

Documentation:
- Our team found it difficult to implement some of the WPI classes because we weren't sure what values WPILib was expecting. We also weren't sure of the expected behavior of those values (for instance when the wheel turns clockwise, should the angle increase or decrease? It actually decreases, like gyro rotation)
- Once we figured this out we documented the entire system to ensure that we could understand it in the future. The SwerveModule abstract class in frc.robot.util is annotated with some javadocs that explain the range of values and expected behavior of each method we needed for the WPI Libraries.
