#include "cps_image_provider.h"
#include "cps_defines.h"
#include "cps_logger.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>

#define MAX_IMG_SRC_LEN  (128)
#define MAX_IMG_NAME_LEN (128)


static char* img_src[MAX_IMG_SRC_LEN+1];

static int find_newest_file(void);

int register_img_src(char* new_img_src)
{
    size_t new_img_src_len = strlen(new_img_src);
    if (new_img_src_len > MAX_IMG_SRC_LEN) {
        cps_log_err("Image source length too long: %d > %d", new_img_src_len, MAX_IMG_SRC_LEN);
        return EXIT_FAILURE;
    }
    cps_log_inf("New image source: %s", new_img_src);
    strncpy(img_src, new_img_src, new_img_src_len);

}

int get_latest_img(void)
{
    
}


static int find_newest_file(char**  newest_file_ptr) {
    struct dirent *entry;
    struct stat file_stat;
    char newest_file[256] = "";
    time_t newest_time = 0;

    DIR *dir = opendir(img_src);
    if (!dir) {
        cps_log_err("Unable to open directory");
        return EXIT_FAILURE;
    }

    while ((entry = readdir(dir)) != NULL) {
        // Skip the current and parent directory entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        char file_path[MAX_IMG_SRC_LEN + MAX_IMG_NAME_LEN];
        snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, entry->d_name);

        // Get file stats
        if (stat(file_path, &file_stat) == 0) {
            // Check if this file is newer than the current newest
            if (difftime(file_stat.st_mtime, newest_time) > 0) {
                newest_time = file_stat.st_mtime;
                strncpy(newest_file, file_path, sizeof(newest_file));
            }
        } else {
            cps_log_err("stat failed");
            return EXIT_FAILURE;
        }
    }

    closedir(dir);

    if (strlen(newest_file) > 0) {
        printf("Newest file: %s\n", newest_file);
        newest_file_ptr = newest
        return EXIT_SUCCESS;
    } else {
        printf("No files found in the directory.\n");
        return EXIT_FAILURE;
    }

    return EXIT_FAILURE;
}


