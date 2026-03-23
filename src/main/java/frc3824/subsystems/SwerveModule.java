class SwerveModule
{
    public:
    
        explicit                   SwerveModule(CanId_t driveMotorCanId, CanId_t angleMotorCanId, CanId_t angleEncoderCanId);

        void                       SetDesiredState(frc::SwerveModuleState &state, std::string description);  // Sets the desired state for the module
        frc::SwerveModuleState     GetState();                                                               // Returns the current state of the module
        frc::SwerveModulePosition  GetPosition();                                                            // Returns the current position of the module
        void                       ResetEncoders();                                                          // Zeroes all the  encoders
        void                       SetWheelAngleToForward(units::angle::degree_t desiredAngle);              // Sets the wheel angle to the forward position
        void                       SimPeriodic();

    private:

        units::angle::degree_t     GetAbsoluteEncoderAngle();

        ctre::phoenix6::hardware::TalonFX  m_driveMotor;
        ctre::phoenix6::hardware::TalonFX  m_angleMotor;
        ctre::phoenix6::hardware::CANcoder m_angleAbsoluteEncoder;

        frc::sim::DCMotorSim m_simDriveModel = frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                frc::DCMotor::KrakenX60(1),
                0.001_kg_sq_m,
                6.75
            ),
            frc::DCMotor::KrakenX60(1)
        );

        frc::sim::DCMotorSim m_simTurnModel = frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                frc::DCMotor::KrakenX44(1),
                0.001_kg_sq_m,
                1.0
            ),
            frc::DCMotor::KrakenX44(1)
        );
};
