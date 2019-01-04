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

#include "motion_master_config.h"

// List of all available config file fields
std::vector<std::string> g_available_config_keys = {
    "left_deadband",
    "right_deadband",
    "max_acceleration",
    "left_multiplier",
    "right_multiplier",
    "motor_zero_power",
    "timeout_s",
    "node_name"
};

// Map of default values for the above keys
std::map<std::string, Json::Value> g_default_config_values = {
    {"left_deadband", Json::Value(0.1)},
    {"right_deadband", Json::Value(0.1)},
    {"max_acceleration", Json::Value(500)},
    {"left_multiplier", Json::Value(1.0)},
    {"right_multiplier", Json::Value(1.0)},
    {"motor_zero_power", Json::Value(10)},
    {"timeout_s", Json::Value(30)},
    {"node_name", Json::Value("sumobot_motion_master")}
};

MotionMasterConfigManager::MotionMasterConfigManager() :ConfigManager() { 
    m_filepath = CONFIG_FILEPATH;
    m_filename = "config.json";
    m_error_message = m_filepath + m_filename +
                      " does not exist. Did you start" +
                      " the correct node beforehand?";
    reload_config_file();
} 

MotionMasterConfigManager::~MotionMasterConfigManager() {
    // dtor
}

void MotionMasterConfigManager::reload_config_file() {
    // Make the config directory if it doesn't exist
    system((std::string("mkdir -p ") + m_filepath).c_str());

    // Normal reload case
    try {
        ConfigManager::reload_config_file();
    }
    catch(const std::runtime_error& e) {
        if(e.what() == m_error_message) {
            // It's fine, this is the message that would normally be thrown
        }
        else {
            // Other error, keep raising
            throw;
        }
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

void MotionMasterConfigManager::write_config_file() {
    // Delete file if it already exists
    std::ofstream ofs(m_filepath + m_filename);

    // Store new JSON file
    if(ofs.is_open()) {
        ofs << m_config_data;

        ofs.close();
    }
    else {
        std::cout << "Failed to open " << m_filepath
                  << m_filename << std::endl;
    }
}

void MotionMasterConfigManager::create_new_file() {
    // Fill missing values with defaults
    for(char i = 0; i < g_available_config_keys.size(); i++) {
        if(!m_config_data.isMember(g_available_config_keys[i])) {
            m_config_data[g_available_config_keys[i]] =
                g_default_config_values[g_available_config_keys[i]];
        }
    }
    write_config_file();
}
