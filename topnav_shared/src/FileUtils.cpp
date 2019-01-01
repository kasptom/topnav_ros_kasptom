#include <cstring>
#include <climits>
#include <stdio.h>
#include <zconf.h>
#include <libgen.h>
#include "FileUtils.h"

char *FileUtils::get_file_path_under_exe_dir(const std::string &fileName) {
    char* file_path = FileUtils::get_executable_dir();
    strcat(file_path, "/");
    strcat(file_path, fileName.c_str());
    return file_path;
}

char *FileUtils::get_executable_dir() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    const char *path;
    if (count != -1) {
        return dirname(result);
    }
    return nullptr;
}