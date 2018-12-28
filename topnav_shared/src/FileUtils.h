#ifndef TOPNAV_SHARED_FILEUTILS_H
#define TOPNAV_SHARED_FILEUTILS_H

#include <string>

class FileUtils {
public:
    static char *get_file_path_under_exe_dir(const std::string &fileName);

private:
    static char *get_executable_dir();
};


#endif //TOPNAV_SHARED_FILEUTILS_H
