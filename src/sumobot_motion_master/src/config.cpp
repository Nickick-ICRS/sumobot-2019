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
    
    
