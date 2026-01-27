#pragma once

#pragma region Includes
#include <thread>
#include <iostream>

#include <units/angle.h>
#include <units/voltage.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <ctre/phoenix6/TalonFX.hpp>
#pragma endregion

#pragma region TalonFXConfiguration
void TalonFXConfiguration(ctre::phoenix6::hardware::TalonFX *motor,
                          units::ampere_t currentLimit,
                          bool   breakMode,
                          bool   continuousWrap,
                          double P,
                          double I,
                          double D,
                          double S,
                          double V,
                          double A,
                          units::turns_per_second_t         velocityLimit,
                          units::turns_per_second_squared_t accelerationLimit);
#pragma endregion