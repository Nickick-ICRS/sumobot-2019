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

