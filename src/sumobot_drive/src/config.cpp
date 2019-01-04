/********************************************************************
 ********************************************************************
 * Copyright 2019 Nick Hafner                                       *
                                                                    *
 * Permission is hereby granted, free of charge, to any person      *
 * obtaining a copy of this software and associated documentation   *
 * files (the "Software"), to deal in the Software without          *
 * restriction, including without limitation the rights to use,     *
 * copy, modify, merge, publish, distribute, sublicense, and/or     *
 * sell copies of the Software, and to permit persons to whom the   *
 * Software is furnished to do so, subject to the following         *
 * conditions:                                                      *
                                                                    *
 * The above copyright notice and this permission notice shall be   *
 * included in all copies or substantial portions of the Software.  *
                                                                    *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,  *
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES  *
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND         *
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT      *
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,     *
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR    *
 * OTHER DEALINGS IN THE SOFTWARE.                                  *
 ********************************************************************
 ********************************************************************/

#include "config.h"

// List of all available config file fields
std::vector<std::string> g_available_config_keys = {
    "node_name",
    "topic_name",
    "pwm_frequency",
    "left_motor_pwm_pin",
    "left_motor_direction_pin",
    "right_motor_pwm_pin",
    "right_motor_direction_pin"
};

// Map of default values for the above keys
std::map<std::string, Json::Value> g_default_config_values = {
    {"node_name", Json::Value("sumobot_drive")},
    {"topic_name", Json::Value("motor_control")},
    {"pwm_frequency", Json::Value(10000)},
    {"left_motor_pwm_pin", Json::Value(10)},
    {"left_motor_direction_pin", Json::Value(11)},
    {"right_motor_pwm_pin", Json::Value(12)},
    {"right_motor_direction_pin", Json::Value(13)}
};

ConfigManager::ConfigManager() {
    reload_config_file();
}

ConfigManager::~ConfigManager() {
    // dtor
}

void ConfigManager::reload_config_file() {
    // Make the config directory if it doesn't exist
    system((std::string("mkdir -p ") + CONFIG_FILEPATH).c_str());
    
    // Open config file
    std::ifstream ifs(CONFIG_FILENAME);
    
    if(ifs.is_open()) {
        // Delete stored data
        m_config_data.clear();
        
        // Store new data
        ifs >> m_config_data;
        ifs.close();
    }
    
    // Ensure that all config values have been read
    // If some are missing, create a new config file with those values
    for(char i = 0; i < g_available_config_keys.size(); i++) {
        if(!m_config_data.isMember(g_available_config_keys[i])) {
            create_new_file();
            return;
        }
    }
}    

void ConfigManager::write_config_file() {
    // Delete file if it already exists
    std::ofstream ofs(CONFIG_FILENAME);
    
    // Store new JSON file
    if(ofs.is_open()) {
        ofs << m_config_data;

        ofs.close();
    }
    else {
        std::cout << "Failed to open " << CONFIG_FILENAME << std::endl;
    }
}

void ConfigManager::create_new_file() {
    // Fill missing values with defaults
    for(char i = 0; i < g_available_config_keys.size(); i++) {
        if(!m_config_data.isMember(g_available_config_keys[i])) {
            m_config_data[g_available_config_keys[i]] = 
                g_default_config_values[g_available_config_keys[i]];
        }
    }
    write_config_file();
}
