#pragma once

#include <frc/kinematics/SwerveModuleState.h>

#include <wpi/array.h>
#include <wpi/sendable/Sendable.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/GenericEntry.h>

class SwerveHeading : public wpi::Sendable {
  public:
    void SetHeading( double heading );
    double GetHeading( void );

  private:
    void InitSendable(wpi::SendableBuilder& builder);

    double m_heading;
};

class SwerveModuleDisplay {
  public:
    SwerveModuleDisplay( std::string tab, std::string name, std::string layout="", int row=0, int col=0 );
    void SetHeading( double heading );
    double GetHeading( void );
    void SetSpeed( double speed );
    double GetSpeed( void );

   frc::ShuffleboardLayout & GetLayout( void );
  private:
    SwerveHeading m_heading;
    std::string m_tab;
    std::string m_name;
    std::string m_layout;
    double m_speed;
    nt::GenericEntry *m_speed_entry;
};

class SwerveStatusDisplay {
  public:
    SwerveStatusDisplay( std::string tab, std::string name );
    void SetState( const wpi::array< frc::SwerveModuleState, 4 > &moduleStates );

  private:
    SwerveModuleDisplay FrontLeftDisp;
    SwerveModuleDisplay FrontRightDisp;
    SwerveModuleDisplay RearLeftDisp;
    SwerveModuleDisplay RearRightDisp;
};

