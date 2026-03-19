#include "TFCard.hpp"
#include <cstring>
#include "cmsis_os2.h"

#define LFS_BLOCK_SIZE    512
#define LFS_BLOCK_COUNT   (32 * 1024)

/* ================= Cache 操作（STM32H7 必须） ================= */

static inline void cache_clean(const void* addr, size_t size)
{
    uintptr_t a = (uintptr_t)addr & ~31UL;
    SCB_CleanDCache_by_Addr((uint32_t*)a, size + 32);
}

static inline void cache_invalidate(const void* addr, size_t size)
{
    uintptr_t a = (uintptr_t)addr & ~31UL;
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, size + 32);
}

/* ================= LittleFS ================= */

LittleFS::LittleFS(SD_HandleTypeDef* hsd)
    : m_hsd(hsd)
{
    /* CMSIS-RTOS V2 Mutex */
    m_mutex = osMutexNew(nullptr);

    memset(&m_cfg, 0, sizeof(m_cfg));
    m_cfg.context = this;

    /* LittleFS 回调 */
    m_cfg.read  = block_read;
    m_cfg.prog  = block_write;
    m_cfg.erase = block_erase;
    m_cfg.sync  = block_sync;

    /* 文件系统参数 */
    m_cfg.read_size      = LFS_BLOCK_SIZE;
    m_cfg.prog_size      = LFS_BLOCK_SIZE;
    m_cfg.block_size     = LFS_BLOCK_SIZE;
    m_cfg.block_count    = LFS_BLOCK_COUNT;
    m_cfg.cache_size     = LFS_BLOCK_SIZE;
    m_cfg.lookahead_size = 32;
    m_cfg.block_cycles   = 500;
}

bool LittleFS::mount()
{
    osMutexAcquire(m_mutex, osWaitForever);

    int ret = lfs_mount(&m_lfs, &m_cfg);
    if (ret)
    {
        lfs_format(&m_lfs, &m_cfg);
        ret = lfs_mount(&m_lfs, &m_cfg);
    }

    osMutexRelease(m_mutex);
    return ret == 0;
}

bool LittleFS::format()
{
    osMutexAcquire(m_mutex, osWaitForever);
    int ret = lfs_format(&m_lfs, &m_cfg);
    osMutexRelease(m_mutex);
    return ret == 0;
}

void LittleFS::unmount()
{
    osMutexAcquire(m_mutex, osWaitForever);
    lfs_unmount(&m_lfs);
    osMutexRelease(m_mutex);
}

lfs_t* LittleFS::handle()
{
    return &m_lfs;
}

/* ================= block device ================= */

int LittleFS::block_read(const lfs_config* c,
                         lfs_block_t block,
                         lfs_off_t off,
                         void* buffer,
                         lfs_size_t size)
{
    auto self = static_cast<LittleFS*>(c->context);

    uint32_t sector =
        block * (c->block_size / LFS_BLOCK_SIZE) +
        off   / LFS_BLOCK_SIZE;

    uint32_t count = size / LFS_BLOCK_SIZE;

    cache_invalidate(buffer, size);

    if (HAL_SD_ReadBlocks_DMA(self->m_hsd,
                              (uint8_t*)buffer,
                              sector,
                              count) != HAL_OK)
        return LFS_ERR_IO;

    while (HAL_SD_GetCardState(self->m_hsd) != HAL_SD_CARD_TRANSFER);

    cache_invalidate(buffer, size);
    return 0;
}

int LittleFS::block_write(const lfs_config* c,
                          lfs_block_t block,
                          lfs_off_t off,
                          const void* buffer,
                          lfs_size_t size)
{
    auto self = static_cast<LittleFS*>(c->context);

    uint32_t sector =
        block * (c->block_size / LFS_BLOCK_SIZE) +
        off   / LFS_BLOCK_SIZE;

    uint32_t count = size / LFS_BLOCK_SIZE;

    cache_clean(buffer, size);

    if (HAL_SD_WriteBlocks_DMA(self->m_hsd,
                               (uint8_t*)buffer,
                               sector,
                               count) != HAL_OK)
        return LFS_ERR_IO;

    while (HAL_SD_GetCardState(self->m_hsd) != HAL_SD_CARD_TRANSFER);

    return 0;
}

int LittleFS::block_erase(const lfs_config*, lfs_block_t)
{
    return 0;
}

int LittleFS::block_sync(const lfs_config*)
{
    return 0;
}
