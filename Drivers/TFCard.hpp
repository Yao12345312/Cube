#pragma once
extern "C"
{
#include "lfs.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
}


class LittleFS
{
public:
    LittleFS(SD_HandleTypeDef* hsd);

    bool mount();
    bool format();
    void unmount();

    lfs_t* handle();

private:
    static int block_read(const struct lfs_config* c,
                          lfs_block_t block,
                          lfs_off_t off,
                          void* buffer,
                          lfs_size_t size);

    static int block_write(const struct lfs_config* c,
                           lfs_block_t block,
                           lfs_off_t off,
                           const void* buffer,
                           lfs_size_t size);

    static int block_erase(const struct lfs_config* c,
                           lfs_block_t block);

    static int block_sync(const struct lfs_config* c);

private:
    SD_HandleTypeDef* m_hsd;
    lfs_t m_lfs;
    lfs_config m_cfg;

    osMutexId_t m_mutex;
};
