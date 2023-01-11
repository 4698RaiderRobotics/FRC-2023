
#include "SwerveModuleDisplay.h"

#include <wpi/sendable/SendableBuilder.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

void SwerveHeading::SetHeading( double heading ) {
  m_heading = heading;
}

double SwerveHeading::GetHeading( void ) {
  return m_heading;
}

void SwerveHeading::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Gyro");
  builder.AddDoubleProperty(
      "Value", [=, this] { return GetHeading(); }, nullptr);
}

SwerveModuleDisplay::SwerveModuleDisplay( std::string tab, std::string name, std::string layout, int row, int col ) {
    m_tab = tab;
    m_name = name;
    m_layout = layout;

    wpi::StringMap<nt::Value> layout_properties{
        std::make_pair("Number of rows", nt::Value::MakeDouble(4)),
        std::make_pair("Number of columns", nt::Value::MakeDouble(2))        
    };
    wpi::StringMap<nt::Value> angle_properties{
        std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))        
    };
    wpi::StringMap<nt::Value> speed_properties{
        std::make_pair("min", nt::Value::MakeDouble(-1)),
        std::make_pair("max", nt::Value::MakeDouble(1))        
    };

    frc::ShuffleboardLayout *swerve_layout;
    if( m_layout.size() > 0 ) {
        swerve_layout = &frc::Shuffleboard::GetTab(m_tab)
            .GetLayout(m_layout, frc::BuiltInLayouts::kGrid)
            .WithProperties( layout_properties )
            .WithSize(4,6);
    } else {
        swerve_layout = &frc::Shuffleboard::GetTab(m_tab)
            .GetLayout(m_name, frc::BuiltInLayouts::kList)
            .WithSize(2, 3);
    }

    int rowidx = 2*(row-1);
    swerve_layout->Add(name + " Steering Angle", m_heading)
        .WithProperties(angle_properties)
        .WithPosition(col-1, rowidx);
    m_speed_entry = swerve_layout->Add(name + " Speed", m_speed)
        .WithWidget(frc::BuiltInWidgets::kNumberSlider)
        .WithProperties(speed_properties)
        .WithPosition(col-1, rowidx + 1)
        .GetEntry();    
}
void SwerveModuleDisplay::SetHeading( double heading ) {
    m_heading.SetHeading( heading );
}

double SwerveModuleDisplay::GetHeading( void ) {
    return m_heading.GetHeading();
}

void SwerveModuleDisplay::SetSpeed( double speed ) {
    m_speed = speed;
    m_speed_entry->SetDouble( m_speed );
}

double SwerveModuleDisplay::GetSpeed( void ) {
    return m_speed;
}

frc::ShuffleboardLayout & SwerveModuleDisplay::GetLayout( void ) {
    return frc::Shuffleboard::GetTab(m_tab)
        .GetLayout(m_name, frc::BuiltInLayouts::kList);
}


SwerveStatusDisplay::SwerveStatusDisplay( std::string tab, std::string name ) :
      FrontLeftDisp( tab, "Front Left Wheel", name, 1, 1), 
      FrontRightDisp( tab, "Front Right Wheel", name, 1, 2 ),
      RearLeftDisp( tab, "Rear Left Wheel", name, 2, 1), 
      RearRightDisp( tab, "Rear Right Wheel", name, 2, 2 ) 
{}


void SwerveStatusDisplay::SetState( const wpi::array< frc::SwerveModuleState, 4 > &moduleStates ) {

    auto [fl, fr, bl, br] = moduleStates;

    FrontLeftDisp.SetSpeed( fl.speed.value() );
    FrontLeftDisp.SetHeading( fl.angle.Degrees().value() );

    FrontRightDisp.SetSpeed( fr.speed.value() );
    FrontRightDisp.SetHeading( fr.angle.Degrees().value() );

    RearLeftDisp.SetSpeed( bl.speed.value() );
    RearLeftDisp.SetHeading( bl.angle.Degrees().value() );

    RearRightDisp.SetSpeed( br.speed.value() );
    RearRightDisp.SetHeading( br.angle.Degrees().value() );
}
