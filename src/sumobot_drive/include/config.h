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

#ifndef __CONFIG_H
#define __CONFIG_H

#include "json/json.h"

#include <iostream>
#include <fstream>

// Filename of the config file this class handles
#define CONFIG_FILEPATH std::string("/home/nrh16/.config/sumobot/sumobot_drive/")
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
