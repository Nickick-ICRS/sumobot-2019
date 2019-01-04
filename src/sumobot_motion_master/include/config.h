#ifndef __CONFIG_H
#define __CONFIG_H

#include "json/json.h"

#include <iostream>
#include <fstream>
#include <stdexcept>

class ConfigManager { 
public:
    ConfigManager(std::string filepath, std::string filename);
    virtual ~ConfigManager();
    
    // Re-read the JSON file
    virtual void reload_config_file();
    
    // Get the live data
    Json::Value *get_config_data() { return &m_config_data; };
protected:
    // Only useable by inherited class
    ConfigManager();
    // Filename and filepath
    std::string m_filename;
    std::string m_filepath;

    // Error message
    std::string m_error_message;

    // Store the data read from the file
    Json::Value m_config_data;
};

#endif // __CONFIG_H

