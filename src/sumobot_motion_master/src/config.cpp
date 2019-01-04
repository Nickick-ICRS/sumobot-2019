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

ConfigManager::ConfigManager(std::string filepath, std::string filename)
                            :m_filepath(filepath), m_filename(filename) {
    m_error_message = m_filepath + m_filename + 
                      " does not exist. Did you start" +
                      " the correct node beforehand?";
    reload_config_file();
}

ConfigManager::ConfigManager() {
    // ctor
}

ConfigManager::~ConfigManager() {
    // dtor
}

void ConfigManager::reload_config_file() {
    // Open config file
    std::ifstream ifs(m_filepath + m_filename);
    
    if(ifs.is_open()) {
        // Delete stored data
        m_config_data.clear();

        // Store new data
        ifs >> m_config_data;
        ifs.close();
    }

    // If the file failed to open then the other node hasn't started yet.
    // This is an error, as the other node contains the default config 
    // values. We can't write here as we could cause a race condition.
    else throw std::runtime_error(m_error_message);
}
    
    
