# Robot <br>
This is the repository where our robot code will be held year after year.

## 2024 Season
### Robot Abilities
- Swerve
- Ground Intake
- AprilTag Detection
- Shooting
    - Shooting Notes from Multiple Positions
    - Quickly Shooting
- Multi-Piece Auto Routines

### Control Stack 
#### Swerve
- SDS MK4i Modules
    - Geared at L2
    - Motors:
        - Drive: Kraken X60
            - CANBus: CANivore
            - [ ] FOC Enabled
        - Steer: Kraken X60
            - CANBus: CANivore
            - [ ] FOC Enabled
    - Sensors:
        - CTRE CANCoder
            - CANBus: CANivore
    

#### Shooter
- TBD

#### Intake
- LYNK v2 BTB Intake
    - Motors: 
        - Kraken X60
            - CANBus: Rio

#### Climber
- Thrifty Telescoping Climb Kit
    - Geared
        - REV MaxPlanetary: TBD:TBD 
    - Motors:
        - Left Side: Kraken X60
            - CANBus: Rio
        - Right Side: Kraken X60
            - CANBus: Rio

#### Vision
- Limelight 3

#### LEDs (Potentially)
- TBD

### Software Stack
- [PathPlanner](https://github.com/mjansen4857/pathplanner)
    - PathPlanner is a motion profile generator for FRC robots.
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
    - AdvantageScope is a robot diagnostics, log review/analysis, and data visualization application for FIRST Robotics Competition teams.
- [Monologue](https://github.com/shueja/Monologue)
    - Monologue is a Java annotation-based logging library for FRC. With Monologue, extensive telemetry and on-robot logging can be added to your robot code with minimal code footprint and design restrictions.
- [Elastic](https://github.com/Gold872/elastic-dashboard)
    - A simple and modern Shuffleboard alternative
