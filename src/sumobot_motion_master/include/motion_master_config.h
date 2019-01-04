#ifndef __MOTION_MASTER_CONFIG
#define __MOTION_MASTER_CONFIG

#include "config.h"

// Filename of the config file this class handles
#define CONFIG_FILEPATH std::string(getenv("HOME")) + "/.config/sumobot/sumobot_motion_master/"

class MotionMasterConfigManager :public ConfigManager {
public:
    MotionMasterConfigManager();
    ~MotionMasterConfigManager();
    
    // Re-read the JSON file
    void reload_config_file();
    // Write currently stored data to the JSON file
    void write_config_file();

    // Get the live data
    Json::Value *get_config_data() { return &m_config_data; };
private:
    // If the file doesn't exist then fill it with values
    void create_new_file();
};

#endif // __MOTION_MASTER_CONFIG
