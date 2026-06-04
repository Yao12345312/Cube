#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   通用Flash参数存储模块 (Bank2 Sector7, 128KB)

   使用方法:
     1. 注册: FlashParam_Register(tag, &your_struct, sizeof(your_struct))
     2. 加载: FlashParam_Load()    -- 启动时调用, 自动匹配tag并拷贝
     3. 保存: FlashParam_Save()    -- 修改参数后调用

   示例:
     // 注册需要持久化的参数块
     FlashParam_Register(0x0001, (void *)K_lqr, sizeof(K_lqr));
     FlashParam_Register(0x0002, (void *)K_ff,  sizeof(K_ff));
     FlashParam_Register(0x0003, &my_cfg,       sizeof(my_cfg));

     // 启动时从Flash加载所有已注册参数
     FlashParam_Load();

     // 修改参数后写回Flash
     FlashParam_Save();
   ============================================================ */

#define FLASH_PARAM_MAX_BLOCKS  8U    /* 最多支持的参数块数量 */
#define FLASH_PARAM_BUF_SIZE    1024U /* 内部序列化缓冲区(字节) */

/* 参数块描述符 */
typedef struct {
    uint32_t tag;        /* 参数唯一标识, 用户自定义(不可为0xFFFFFFFF) */
    void    *data;       /* 指向RAM中的参数数据 */
    uint32_t size;       /* 数据大小(字节) */
} FlashParam_Block_t;

/* 注册参数块, 必须在Load/Save之前调用 */
int FlashParam_Register(uint32_t tag, void *data, uint32_t size);

/* 从Flash加载所有已注册参数(自动按tag匹配) */
int FlashParam_Load(void);

/* 保存所有已注册参数到Flash */
int FlashParam_Save(void);

/* 擦除Flash中的参数(下次启动恢复默认值) */
int FlashParam_Erase(void);

#ifdef __cplusplus
}
#endif
