#include <stdio.h>
#include <string.h>
#include "cal_store.h"
#include "sst25.h"

extern void mock_reset(void);
extern unsigned char *mock_raw(void);
extern int g_write_budget, g_erase_budget, g_power_lost;

static int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("  FAIL: " __VA_ARGS__); printf("\n"); fails++; } }while(0)

static void poke(uint32_t addr, const void *data, uint32_t n){ memcpy(mock_raw()+addr, data, n); }

int main(void)
{
    GrayBaseline_t v1 = { .front_edge=1200, .rear_edge=1350, .ground={3000, 2900} };
    GrayBaseline_t v2 = { .front_edge=1111, .rear_edge=2222, .ground={3333, 4000} };
    GrayBaseline_t got;

    printf("=== 1. CRC-32/ISO-HDLC 自检 ===\n");
    CHECK(Cal_Crc32SelfTest(), "crc32(\"123456789\") 不是 0xCBF43926");
    printf("  自检 %s\n", Cal_Crc32SelfTest() ? "通过" : "失败");

    printf("=== 2. 空 flash: Load 必须返回 NOT_FOUND ===\n");
    mock_reset();
    CHECK(CalStore_Load(&got) == CAL_NOT_FOUND, "空片未返回 CAL_NOT_FOUND (得到 %s)", Cal_StatusStr(Cal_LastStatus()));
    printf("  %s\n", Cal_StatusStr(Cal_LastStatus()));

    printf("=== 3. 存一次再读回 ===\n");
    mock_reset();
    CHECK(CalStore_Save(&v1) == CAL_OK, "首次保存失败");
    CHECK(CalStore_Load(&got) == CAL_OK, "首次读回失败");
    CHECK(got.front_edge==1200 && got.rear_edge==1350 && got.ground[0]==3000 && got.ground[1]==2900, "读回数值不符");
    printf("  存到槽 A(0x1FF000), 读回 F=%u R=%u G=%u/%u\n", got.front_edge,got.rear_edge,got.ground[0],got.ground[1]);

    printf("=== 4. A/B 交替 + seq 递增 ===\n");
    CHECK(CalStore_Save(&v2) == CAL_OK, "第二次保存失败");
    {
        unsigned char *f = mock_raw();
        uint32_t seqA, seqB;
        memcpy(&seqA, f+0x1FF000+0x08, 4);
        memcpy(&seqB, f+0x1FE000+0x08, 4);
        CHECK(seqA==1 && seqB==2, "seq 不对 (A=%u B=%u)", seqA, seqB);
        CHECK(CalStore_Load(&got)==CAL_OK && got.front_edge==1111, "第二次读回应取 B 的新值");
        printf("  seq: A=%u B=%u, 读回取新的那个 ✓\n", seqA, seqB);
    }

    printf("=== 5. 【关键】写入过程中断电 —— 每个断点都必须还有一份完整数据 ===\n");
    {
        int bad = 0;
        for (int cut = 0; cut <= 24; cut++) {
            mock_reset();
            if (CalStore_Save(&v1) != CAL_OK) { printf("  准备阶段失败\n"); bad++; break; }
            g_write_budget = cut;
            (void)CalStore_Save(&v2);
            g_write_budget = -1;
            {
                Cal_Status_t s = CalStore_Load(&got);
                uint16_t want = (cut >= 24) ? 1111 : 1200;   /* 24 = 写完了, 应得新值 */
                if (s != CAL_OK || got.front_edge != want) {
                    printf("  FAIL: 在写第 %d 字节处断电后数据不对 (期望 %u 实得 %u, %s)\n",
                           cut, want, got.front_edge, Cal_StatusStr(s));
                    bad++;
                }
            }
        }
        printf("  24 个断点全部测过: %s\n", bad ? "有丢失!" : "旧数据始终完好 ✓");
        fails += bad;
    }

    printf("=== 6. 擦除 B 槽过程中断电 ===\n");
    {
        mock_reset();
        CalStore_Save(&v1);
        g_erase_budget = 0;
        (void)CalStore_Save(&v2);
        g_erase_budget = -1;
        CHECK(CalStore_Load(&got) == CAL_OK && got.front_edge == 1200, "擦除中断电后旧值丢失");
        printf("  %s ✓\n", CalStore_Load(&got)==CAL_OK ? "旧值完好" : "丢失");
    }

    printf("=== 7. 数值合理性门禁(CRC 对但数值退化必须被拒) ===\n");
    {
        unsigned char rec[24]; uint32_t magic=CAL_REC_MAGIC, seq=1;
        uint16_t ver=CAL_REC_VERSION, len=CAL_PAYLOAD_LEN;
        uint16_t fe=1200, re_=1350, g0=0, g1=2900; uint32_t crc;
        mock_reset();
        memset(rec,0xFF,24);
        memcpy(rec+0,&magic,4); memcpy(rec+4,&ver,2); memcpy(rec+6,&len,2); memcpy(rec+8,&seq,4);
        memcpy(rec+12,&fe,2); memcpy(rec+14,&re_,2); memcpy(rec+16,&g0,2); memcpy(rec+18,&g1,2);
        crc = Cal_Crc32(rec, 20); memcpy(rec+20,&crc,4);
        poke(CAL_SLOT_A_ADDR, rec, 24);
        {
            Cal_Status_t s = CalStore_Load(&got);
            CHECK(s != CAL_OK, "ground=0 的退化记录被当成有效了!");
            printf("  被正确拒绝: %s\n", Cal_StatusStr(s));
        }
    }

    printf("=== 7b. 保存端也要过同一道门(否则写进去却永远读不出来) ===\n");
    {
        GrayBaseline_t badv = { .front_edge=1200, .rear_edge=1350, .ground={0, 2900} };
        mock_reset();
        CHECK(CalStore_Save(&badv) != CAL_OK, "退化值竟然保存成功了");
        printf("  保存被拒: %s ✓\n", Cal_StatusStr(Cal_LastStatus()));
    }

    printf("=== 8. 扇区分类守卫 ===\n");
    {
        mock_reset();
        CHECK(Cal_ClassifySector(0x000000)==SEC_ERASED, "空扇区应判 ERASED");
        CalStore_Save(&v1);
        CHECK(Cal_ClassifySector(CAL_SLOT_A_ADDR)==SEC_OURS, "我们的记录应判 OURS");
        {   /* 撕毁的记录: 只写了前 6 字节 */
            unsigned char junk[6]={0x43,0x41,0x4C,0x31,0x01,0x00};
            poke(0x001000, junk, 6);
            CHECK(Cal_ClassifySector(0x001000)==SEC_OURS, "撕毁的记录必须仍判 OURS");
        }
        {   /* FAT 引导扇区 */
            unsigned char boot[512]; memset(boot,0x00,512); boot[510]=0x55; boot[511]=0xAA;
            poke(0x002000, boot, 512);
            CHECK(Cal_ClassifySector(0x002000)==SEC_FOREIGN_FS, "FAT 引导签名应判 FILESYSTEM");
        }
        {   /* STM32 固件镜像头 */
            unsigned char img[64]; uint32_t sp=0x20005000, pc=0x0800B0C1;
            memset(img,0xFF,64); memcpy(img,&sp,4); memcpy(img+4,&pc,4);
            poke(0x003000, img, 64);
            CHECK(Cal_ClassifySector(0x003000)==SEC_FOREIGN_IMAGE, "STM32 镜像头应判 FW IMAGE");
        }
        printf("  ERASED / OURS(含撕毁) / FILESYSTEM / FW IMAGE 四类判定全部正确\n");
    }

    printf("=== 9. 【回归】中断的擦除留下的位翻转残留, 必须仍可擦写 ===\n");
    {
        mock_reset();
        CalStore_Save(&v1);
        CalStore_Save(&v2);
        {
            unsigned char *f = mock_raw();
            unsigned char junk[4] = {0x63, 0x61, 0x6C, 0xFF};
            memcpy(f + CAL_SLOT_A_ADDR, junk, 4);
            memset(f + CAL_SLOT_A_ADDR + 4, 0xFF, 20);
        }
        {
            SecClass_t c = Cal_ClassifySector(CAL_SLOT_A_ADDR);
            GrayBaseline_t v3 = { .front_edge=1500, .rear_edge=1600, .ground={2000, 2100} };
            Cal_Status_t s3;
            CHECK(c == SEC_OURS, "位翻转残留被判成 %s, 会导致标定永久死锁", Cal_SecClassStr(c));
            s3 = CalStore_Save(&v3);
            CHECK(s3 == CAL_OK, "位翻转残留的槽再存失败: %s", Cal_StatusStr(s3));
            CHECK(CalStore_Load(&got) == CAL_OK && got.front_edge == 1500, "重存后读回不对");
            printf("  分类=%s, 重新保存=%s, 读回 F=%u OK\n",
                   Cal_SecClassStr(c), Cal_StatusStr(s3), got.front_edge);
        }
    }

    printf("\n=== 结论: %s (%d 项失败) ===\n", fails? "有失败项" : "全部通过", fails);
    return fails != 0;
}
