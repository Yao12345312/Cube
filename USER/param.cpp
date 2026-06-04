#include "param.hpp"
#include "stm32h7xx_hal.h"
#include <cstring>

/* ============================================================
   Flash存储地址: Bank2, Sector7 (128KB)
   STM32H743: 双Bank各1MB, Bank2完全空闲给参数存储用
   起始地址 0x081E0000
   ============================================================ */
#define PARAM_FLASH_ADDR      0x081E0000UL  //扇区7
#define PARAM_FLASH_SECTOR    FLASH_SECTOR_7
#define PARAM_FLASH_BANK      FLASH_BANK_2

/* Flash数据格式魔数: "FPMC" (Flash ParaM Config) */
#define PARAM_FLASH_MAGIC     0x46504D43UL
#define PARAM_FLASH_VERSION   1U

/* Flash布局 (所有字段小端序):
   [Header: 16B]
     magic(4) + version(4) + block_count(4) + data_size(4)
   [Block 0]
     tag(4) + size(4) + data[size] + pad_to_4B
   [Block 1] ...
   [Footer: 4B]
     checksum (所有32bit字的异或校验, 覆盖Header到最后一个Block末尾)
   总大小向上对齐到32B (FlashWord = 256bit) */

/* ---------- 内部状态 ---------- */
static FlashParam_Block_t s_blocks[FLASH_PARAM_MAX_BLOCKS];
static uint32_t           s_block_count = 0;


/* 计算32bit字的异或校验和 */
static uint32_t calc_checksum(const uint32_t *data, uint32_t word_count)
{
    uint32_t ck = 0;
    for (uint32_t i = 0; i < word_count; i++) {
        ck ^= data[i];
    }
    return ck;
}

/* 4字节对齐向上取整 */
static inline uint32_t align4(uint32_t n) {
    return (n + 3U) & ~3U;
}

/* Flash编程: 按FlashWord(256bit=32B)写入 */
static HAL_StatusTypeDef flash_program(uint32_t addr, const uint8_t *buf, uint32_t byte_len)
{
    uint32_t word_buf[8];  /* 1 FlashWord = 8 x 32bit */
    uint32_t words = (byte_len + 31U) / 32U;

    for (uint32_t w = 0; w < words; w++) {
        memset(word_buf, 0xFF, sizeof(word_buf));
        uint32_t remain = byte_len - w * 32U;
        uint32_t copy_n = (remain < 32U) ? remain : 32U;
        memcpy(word_buf, &buf[w * 32U], copy_n);
        HAL_StatusTypeDef s = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)word_buf);
        if (s != HAL_OK) return s;
        addr += 32U;
    }
    return HAL_OK;
}

/* ---------- 对外接口 ---------- */

/* 注册参数块 */
int FlashParam_Register(uint32_t tag, void *data, uint32_t size)
{
    if (s_block_count >= FLASH_PARAM_MAX_BLOCKS) return -1;
    if (tag == 0xFFFFFFFFUL)   return -1;   /* 与Flash擦除态冲突 */
    if (data == NULL || size == 0) return -1;

    s_blocks[s_block_count].tag  = tag;
    s_blocks[s_block_count].data = data;
    s_blocks[s_block_count].size = size;
    s_block_count++;
    return 0;
}

/* 从Flash加载所有已注册参数 */
int FlashParam_Load(void)
{
    if (s_block_count == 0) return -1;

    const uint8_t *pflash = (const uint8_t *)PARAM_FLASH_ADDR;

    /* 读取Header */
    uint32_t magic      = *(const uint32_t *)(pflash + 0);
    uint32_t version    = *(const uint32_t *)(pflash + 4);
    uint32_t block_cnt  = *(const uint32_t *)(pflash + 8);
    uint32_t data_size  = *(const uint32_t *)(pflash + 12);

    (void)version;

    if (magic != PARAM_FLASH_MAGIC) return -1;
    if (block_cnt > FLASH_PARAM_MAX_BLOCKS) return -1;

    /* 校验Footer checksum */
    uint32_t total_words = (16U + data_size) / 4U;
    uint32_t flash_ck    = *(const uint32_t *)(pflash + 16U + data_size);
    uint32_t calc_ck     = calc_checksum((const uint32_t *)pflash, total_words);
    if (calc_ck != flash_ck) return -1;

    /* 遍历Block, 按tag匹配注册表并拷贝数据 */
    const uint8_t *ptr = pflash + 16U;
    for (uint32_t i = 0; i < block_cnt; i++) {
        uint32_t tag  = *(const uint32_t *)(ptr);
        uint32_t size = *(const uint32_t *)(ptr + 4);
        const uint8_t *data = ptr + 8;

        /* 在注册表中查找匹配的tag */
        for (uint32_t j = 0; j < s_block_count; j++) {
            if (s_blocks[j].tag == tag) {
                uint32_t copy_n = (size < s_blocks[j].size) ? size : s_blocks[j].size;
                memcpy(s_blocks[j].data, data, copy_n);
                break;
            }
        }
        ptr += 8U + align4(size);
    }
    return 0;
}

/* 保存所有已注册参数到Flash */
int FlashParam_Save(void)
{
    if (s_block_count == 0) return -1;

    /* ---------- 在RAM中构建Flash镜像 ---------- */
    static uint8_t buf[FLASH_PARAM_BUF_SIZE];
    memset(buf, 0xFF, sizeof(buf));

    uint32_t offset = 0;

    /* Header: magic(4) + version(4) + block_count(4) + data_size(4) */
    uint32_t header[4];
    header[0] = PARAM_FLASH_MAGIC;
    header[1] = PARAM_FLASH_VERSION;
    header[2] = s_block_count;
    /* header[3] = data_size, 稍后回填 */

    offset = 16U;  /* 跳过Header */

    /* 写入所有Block */
    for (uint32_t i = 0; i < s_block_count; i++) {
        uint32_t tag  = s_blocks[i].tag;
        uint32_t size = s_blocks[i].size;
        uint32_t padded_size = align4(size);

        if (offset + 8U + padded_size + 4U > FLASH_PARAM_BUF_SIZE) return -1;

        memcpy(&buf[offset],     &tag,  4);
        memcpy(&buf[offset + 4], &size, 4);
        memcpy(&buf[offset + 8], s_blocks[i].data, size);
        offset += 8U + padded_size;
    }

    /* 回填Header中的data_size (Block区域总长度, 不含Header和Footer) */
    uint32_t data_size = offset - 16U;
    header[3] = data_size;
    memcpy(&buf[0], header, 16);

    /* Footer: checksum (覆盖Header + 所有Block) */
    uint32_t ck = calc_checksum((const uint32_t *)buf, offset / 4U);
    memcpy(&buf[offset], &ck, 4);
    offset += 4U;

    /* 向上对齐到FlashWord (32B) */
    uint32_t total_len = (offset + 31U) & ~31U;

    /* ---------- Flash写入 ---------- */
    __disable_irq();

    if (HAL_FLASHEx_Unlock_Bank2() != HAL_OK) {
        __enable_irq();
        return -1;
    }

    /* 擦除Sector */
    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks        = PARAM_FLASH_BANK;
    erase_init.Sector       = PARAM_FLASH_SECTOR;
    erase_init.NbSectors    = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status != HAL_OK) {
        HAL_FLASHEx_Lock_Bank2();
        __enable_irq();
        return -1;
    }

    /* 按FlashWord编程 */
    status = flash_program(PARAM_FLASH_ADDR, buf, total_len);

    HAL_FLASHEx_Lock_Bank2();
    __enable_irq();
    return (status == HAL_OK) ? 0 : -1;
}

/* 擦除Flash中的参数 */
int FlashParam_Erase(void)
{
    __disable_irq();

    if (HAL_FLASHEx_Unlock_Bank2() != HAL_OK) {
        __enable_irq();
        return -1;
    }

    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks        = PARAM_FLASH_BANK;
    erase_init.Sector       = PARAM_FLASH_SECTOR;
    erase_init.NbSectors    = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASHEx_Lock_Bank2();
    __enable_irq();
    return (status == HAL_OK) ? 0 : -1;
}
