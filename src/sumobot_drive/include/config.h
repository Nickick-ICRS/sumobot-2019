#ifndef __CONFIG_H
#define __CONFIG_H

#include "json/json.h"

#include <ios>
#include <fstream>

// Filename of the config file this class handles
#define CONFIG_FILEPATH std::string(getenv("HOME")) + "/.config/sumobot/sumobot_drive/"
#define CONFIG_FILENAME std::string(CONFIG_FILEPATH) + "config.json"

class ConfigManager {
public:
    ConfigManager();
    ~ConfigManager();

    // Re-read the JSON file
    void reload_config_file();
    // Write currently stored data to the JSON file
    void write_config_file();
    
    // Get the live data
    Json::Value *get_config_data() { return &m_config_data; };
private:
    // Store the data read from the file
    Json::Value m_config_data;

    // If the file doesn't exist then fill it with values
    void create_new_file();
};

#endif // __CONFIG_H
