// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>

class DataLogger {
    private:
        // This class is a singleton.
        static DataLogger *singleton;

        // Constructor is private
        DataLogger() {}
    public:

        // delete copy constructor
        DataLogger(const DataLogger& obj) = delete; 

        static DataLogger& GetInstance() {
            // If there is no instance of class
            // then we can create an instance.
            if (singleton == nullptr)  {
                singleton = new DataLogger();
                singleton->log = &frc::DataLogManager::GetLog();
            }
            
            return *singleton;
        }

        void Send( std::string_view s, double val );

        void Send( std::string_view s, std::string_view val );

        void Send( std::string_view s, bool val );
 
        void LogMetadata( void );

    private:
        wpi::log::DataLog *log;

        void SendMetadata( std::string_view s, std::string_view val );

};
