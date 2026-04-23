/*
  Copyright 2009-2010, jimmikaelkael
  Licenced under Academic Free License version 3.0
  Review Open PS2 Loader README & LICENSE files for further details.
*/

#include "internal.h"
#include "../../isofs/zso.h"

#define MODNAME "cdvd_driver"
IRX_ID(MODNAME, 1, 1);

//------------------ Patch Zone ----------------------
struct CDVDMAN_SETTINGS_TYPE cdvdman_settings = {
    CDVDMAN_SETTINGS_DEFAULT_COMMON,
    CDVDMAN_SETTINGS_DEFAULT_DEVICE_SETTINGS};

//----------------------------------------------------
extern struct irx_export_table _exp_cdvdman;
extern struct irx_export_table _exp_cdvdstm;
extern struct irx_export_table _exp_smsutils;
extern struct irx_export_table _exp_oplutils;
#ifdef __USE_DEV9
extern struct irx_export_table _exp_dev9;
#endif

// reader function interface, raw reader impementation by default
int DeviceReadSectorsCached(u64 sector, void *buffer, unsigned int count);
int (*DeviceReadSectorsPtr)(u64 sector, void *buffer, unsigned int count) = &DeviceReadSectors;

// internal functions prototypes
static void oplShutdown(int poff);
static int cdvdman_writeSCmd(u8 cmd, const void *in, u16 in_size, void *out, u16 out_size);
static unsigned int event_alarm_cb(void *args);
static void cdvdman_signal_read_end(void);
static void cdvdman_signal_read_end_intr(void);
static void cdvdman_startThreads(void);
static void cdvdman_create_semaphores(void);
static int cdvdman_read(u32 lsn, u32 sectors, u16 sector_size, void *buf);

// Sector cache to improve IO
static u8 MAX_SECTOR_CACHE = 0;
static u8 *sector_cache = NULL;
static u64 cur_sector = 0xffffffffffffffff;

struct cdvdman_cb_data
{
    void (*user_cb)(int reason);
    int reason;
};

cdvdman_status_t cdvdman_stat;
static struct cdvdman_cb_data cb_data;

int cdrom_io_sema;
static int cdrom_rthread_sema;
static int cdvdman_scmdsema;
int cdvdman_searchfilesema;
static int cdvdman_ReadingThreadID;

static StmCallback_t Stm0Callback = NULL;
static iop_sys_clock_t gCallbackSysClock;

// buffers
u8 cdvdman_buf[CDVDMAN_BUF_SECTORS * 2048];

#define CDVDMAN_MODULE_VERSION 0x225
static int cdvdman_debug_print_flag = 0;

unsigned char sync_flag;
unsigned char cdvdman_cdinited = 0;
static unsigned int ReadPos = 0; /* Current buffer offset in 2048-byte sectors. */

#ifdef __USE_DEV9
static int POFFThreadID;
#endif

typedef void (*oplShutdownCb_t)(void);
static oplShutdownCb_t vmcShutdownCb = NULL;

// --- [90004 Mechanical Tracking Variables] ---
static u32 last_requested_lsn = 0;
static int is_init = 1;

void initCache()
{
    u8 cache_size = cdvdman_settings.common.zso_cache;
    if (cache_size && sector_cache == NULL) {
        sector_cache = AllocSysMemory(ALLOC_FIRST, cache_size * 2048, NULL);
        if (sector_cache)
            MAX_SECTOR_CACHE = cache_size;
    }
}

void oplRegisterShutdownCallback(oplShutdownCb_t cb)
{
    vmcShutdownCb = cb;
}

static void oplShutdown(int poff)
{
    u32 stat;

    DeviceLock();
    if (vmcShutdownCb != NULL)
        vmcShutdownCb();
    DeviceUnmount();
    if (poff) {
        DeviceStop();
#ifdef __USE_DEV9
        Dev9CardStop();
#endif
        sceCdPowerOff(&stat);
    }
}

//-------------------------------------------------------------------------
#ifdef __USE_DEV9
static void cdvdman_poff_thread(void *arg)
{
    SleepThread();

    oplShutdown(1);
}
#endif

void cdvdman_init(void)
{
#ifdef __USE_DEV9
    iop_thread_t ThreadData;
#endif

    if (!cdvdman_cdinited) {
        cdvdman_stat.err = SCECdErNO;

        cdvdman_fs_init();

#ifdef __USE_DEV9
        if (cdvdman_settings.common.flags & IOPCORE_ENABLE_POFF) {
            ThreadData.attr = TH_C;
            ThreadData.option = 0xABCD0001;
            ThreadData.priority = 1;
            ThreadData.stacksize = 0x1000;
            ThreadData.thread = &cdvdman_poff_thread;
            StartThread(POFFThreadID = CreateThread(&ThreadData), NULL);
        }
#endif

        cdvdman_cdinited = 1;
    }
}

int sceCdInit(int init_mode)
{
    cdvdman_init();
    return 1;
}

//-------------------------------------------------------------------------
static unsigned int cdvdemu_read_end_cb(void *arg)
{
    iSetEventFlag(cdvdman_stat.intr_ef, 0x1000);
    return 0;
}

void *ziso_alloc(u32 size)
{
    return AllocSysMemory(0, size, NULL);
}

int DeviceReadSectorsCached(u64 lsn, void *buffer, unsigned int sectors)
{
    if (sectors < MAX_SECTOR_CACHE) {
        if (cur_sector == 0xffffffffffffffff || lsn < cur_sector || (lsn + sectors) - cur_sector > MAX_SECTOR_CACHE) {
            DeviceReadSectors(lsn, sector_cache, MAX_SECTOR_CACHE);
            cur_sector = lsn;
        }
        int pos = lsn - cur_sector;
        memcpy(buffer, &(sector_cache[pos * 2048]), 2048 * sectors);
        return SCECdErNO;
    }
    int res = DeviceReadSectors(lsn, buffer, sectors);
    return res;
}

int read_raw_data(u8 *addr, u32 size, u32 offset, u32 shift)
{
    u32 o_size = size;
    u64 lba = offset / (2048 >> shift);
    u32 pos = (offset << shift) & 2047;

    int (*ReadSectors)(u64 lsn, void *buffer, unsigned int sectors) = (addr == (u8 *)ziso_idx_cache) ? &DeviceReadSectors : &DeviceReadSectorsCached;

    if (pos) {
        int r = MIN(size, (2048 - pos));
        ReadSectors(lba, ziso_tmp_buf, 1);
        memcpy(addr, ziso_tmp_buf + pos, r);
        size -= r;
        lba++;
        addr += r;
    }

    u32 n_blocks = size / 2048;
    if (size % 2048)
        n_blocks++;
    if (n_blocks > 1) {
        int r = 2048 * (n_blocks - 1);
        ReadSectors(lba, addr, n_blocks - 1);
        size -= r;
        addr += r;
        lba += n_blocks - 1;
    }

    if (size) {
        ReadSectors(lba, ziso_tmp_buf, 1);
        memcpy(addr, ziso_tmp_buf, size);
        size = 0;
    }

    return o_size - size;
}

int DeviceReadSectorsCompressed(u64 lsn, void *addr, unsigned int count)
{
    return (ziso_read_sector(addr, (u32)lsn, count) == count) ? SCECdErNO : SCECdErEOM;
}

static int probed = 0;
static int ProbeZSO(u8 *buffer)
{
    if (DeviceReadSectors(0, buffer, 1) != SCECdErNO)
        return 0;
    probed = 1;
    if (*(u32 *)buffer == ZSO_MAGIC) {
        ziso_init((ZISO_header *)buffer, *(u32 *)(buffer + sizeof(ZISO_header)));
        initCache();
        DeviceReadSectorsPtr = &DeviceReadSectorsCompressed;
    }
    return 1;
}

static int cdvdman_read_sectors(u32 lsn, unsigned int sectors, void *buf)
{
    unsigned int remaining;
    void *ptr;
    int endOfMedia = 0;

    DPRINTF("cdvdman_read lsn=%lu sectors=%u buf=%p\n", lsn, sectors, buf);

    // --- 1. SPIN-UP DELAY (Initial Load) ---
    if (is_init) {
        DelayThread(400000); // 400ms simulate motor start
        is_init = 0;
    }

    // --- 2. SEEK LATENCY (Mechanical Head Movement) ---
    // Simulate seek if the jump is non-sequential
    if (lsn > (last_requested_lsn + 20) || lsn < last_requested_lsn) {
        DelayThread(120000); // 120ms seek
    }

    if (mediaLsnCount) {
        if (lsn >= mediaLsnCount) {
            cdvdman_stat.err = SCECdErIPI;
            return 1;
        }
        if ((lsn + sectors) > mediaLsnCount) {
            endOfMedia = 1;
            sectors = mediaLsnCount - lsn;
        }
    }

    if (probed == 0) {
        if (!ProbeZSO(buf))
            return 1;
    }

    cdvdman_stat.err = SCECdErNO;
    for (ptr = buf, remaining = sectors; remaining > 0;) {
        unsigned int SectorsToRead = remaining;

        if (cdvdman_settings.common.flags & IOPCORE_COMPAT_ACCU_READS) {
            iop_sys_clock_t TargetTime;
            if (SectorsToRead > 8)
                SectorsToRead = 8;
            TargetTime.hi = 0;
            TargetTime.lo = (cdvdman_settings.common.media == 0x12 ? 81920 : 33512) * SectorsToRead;
            ClearEventFlag(cdvdman_stat.intr_ef, ~0x1000);
            SetAlarm(&TargetTime, &cdvdemu_read_end_cb, NULL);
        }

        // --- 3. FETCH DATA (THE SMB READ) ---
        cdvdman_stat.err = DeviceReadSectorsPtr(lsn, ptr, SectorsToRead);
        
        if (cdvdman_stat.err != SCECdErNO) {
            if (cdvdman_settings.common.flags & IOPCORE_COMPAT_ACCU_READS)
                CancelAlarm(&cdvdemu_read_end_cb, NULL);
            break;
        }

        // --- 4. ROTATIONAL RHYTHM ---
        // 10.5ms per sector. Forces game to wait for physical "rotation" without timing out SMB.
        DelayThread(10500 * SectorsToRead);

        if (lsn < 13) {
            u32 j;
            u8 *logo = (u8 *)ptr;
            static u8 key = 0;
            if (lsn == 0) 
                key = logo[0];
            if (key != 0) {
                for (j = 0; j < (SectorsToRead * 2048); j++) {
                    logo[j] ^= key;
                    logo[j] = (logo[j] << 3) | (logo[j] >> 5);
                }
            }
        }

        ptr = (void *)((u8 *)ptr + (SectorsToRead * 2048));
        remaining -= SectorsToRead;
        lsn += SectorsToRead;
        ReadPos += SectorsToRead * 2048;

        if (cdvdman_settings.common.flags & IOPCORE_COMPAT_ACCU_READS) {
            WaitEventFlag(cdvdman_stat.intr_ef, 0x1000, WEF_AND, NULL);
        }
    }

    last_requested_lsn = lsn;

    if (endOfMedia) {
        cdvdman_stat.err = SCECdErEOM;
    }

    return (cdvdman_stat.err == SCECdErNO ? 0 : 1);
}

static int cdvdman_read(u32 lsn, u32 sectors, u16 sector_size, void *buf)
{
    cdvdman_stat.status = SCECdStatRead;
    buf = (void *)PHYSADDR(buf);
    u32 offset = 0;
    if (sector_size == 2340)
        offset = 12;
    if ((u32)(buf)&3 || (sector_size != 2048)) {
        WaitSema(cdvdman_searchfilesema);
        u32 nsectors, nbytes;
        u32 rpos = lsn;
        while (sectors > 0) {
            nsectors = sectors;
            if (nsectors > CDVDMAN_BUF_SECTORS)
                nsectors = CDVDMAN_BUF_SECTORS;
            if (sector_size != 2048)
                nsectors = 1;
            cdvdman_read_sectors(rpos, nsectors, cdvdman_buf);
            rpos += nsectors;
            sectors -= nsectors;
            nbytes = nsectors * sector_size;
            memcpy((void *)((u32)buf + offset), cdvdman_buf, nbytes);
            if (sector_size == 2340) {
                u8 *header = (u8 *)buf;
                sceCdlLOCCD p;
                sceCdIntToPos(rpos - 1, &p);
                header[0] = p.minute;
                header[1] = p.second;
                header[2] = p.sector;
                header[3] = 0;
                header[4] = header[8] = 0;
                header[5] = header[9] = 0;
                header[6] = header[10] = 0x8;
                header[7] = header[11] = 0;
            }
            buf = (void *)((u8 *)buf + nbytes);
        }
        SignalSema(cdvdman_searchfilesema);
    } else
        cdvdman_read_sectors(lsn, sectors, buf);
    ReadPos = 0;
    cdvdman_stat.status = SCECdStatPause;
    return 1;
}

u32 sceCdGetReadPos(void)
{
    return ReadPos;
}

static int cdvdman_common_lock(int IntrContext)
{
    if (sync_flag)
        return 0;
    if (IntrContext)
        iClearEventFlag(cdvdman_stat.intr_ef, ~1);
    else
        ClearEventFlag(cdvdman_stat.intr_ef, ~1);
    sync_flag = 1;
    return 1;
}

int cdvdman_AsyncRead(u32 lsn, u32 sectors, u16 sector_size, void *buf)
{
    int IsIntrContext, OldState;
    IsIntrContext = QueryIntrContext();
    CpuSuspendIntr(&OldState);
    if (!cdvdman_common_lock(IsIntrContext)) {
        CpuResumeIntr(OldState);
        return 0;
    }
    cdvdman_stat.cdread_lba = lsn;
    cdvdman_stat.cdread_sectors = sectors;
    cdvdman_stat.sector_size = sector_size;
    cdvdman_stat.cdread_buf = buf;
    CpuResumeIntr(OldState);
    if (IsIntrContext)
        iSignalSema(cdrom_rthread_sema);
    else
        SignalSema(cdrom_rthread_sema);
    return 1;
}

int cdvdman_SyncRead(u32 lsn, u32 sectors, u16 sector_size, void *buf)
{
    int IsIntrContext, OldState;
    IsIntrContext = QueryIntrContext();
    CpuSuspendIntr(&OldState);
    if (!cdvdman_common_lock(IsIntrContext)) {
        CpuResumeIntr(OldState);
        return 0;
    }
    CpuResumeIntr(OldState);
    cdvdman_read(lsn, sectors, sector_size, buf);
    cdvdman_cb_event(SCECdFuncRead);
    sync_flag = 0;
    SetEventFlag(cdvdman_stat.intr_ef, 9);
    return 1;
}

static void cdvdman_initDiskType(void)
{
    cdvdman_stat.err = SCECdErNO;
    cdvdman_stat.disc_type_reg = (int)cdvdman_settings.common.media;
}

u32 sceCdPosToInt(sceCdlLOCCD *p)
{
    register u32 result;
    result = ((u32)p->minute >> 4) * 10 + ((u32)p->minute & 0xF);
    result *= 60;
    result += ((u32)p->second >> 4) * 10 + ((u32)p->second & 0xF);
    result *= 75;
    result += ((u32)p->sector >> 4) * 10 + ((u32)p->sector & 0xF);
    result -= 150;
    return result;
}

sceCdlLOCCD *sceCdIntToPos(u32 i, sceCdlLOCCD *p)
{
    register u32 sc, se, mi;
    i += 150;
    se = i / 75;
    sc = i - se * 75;
    mi = se / 60;
    se = se - mi * 60;
    p->sector = (sc - (sc / 10) * 10) + (sc / 10) * 16;
    p->second = (se / 10) * 16 + se - (se / 10) * 10;
    p->minute = (mi / 10) * 16 + mi - (mi / 10) * 10;
    return p;
}

sceCdCBFunc sceCdCallback(sceCdCBFunc func)
{
    int oldstate;
    void *old_cb;
    if (sceCdSync(1))
        return NULL;
    CpuSuspendIntr(&oldstate);
    old_cb = cb_data.user_cb;
    cb_data.user_cb = func;
    CpuResumeIntr(oldstate);
    return old_cb;
}

int sceCdSC(int code, int *param)
{
    int result;
    switch (code) {
        case CDSC_GET_INTRFLAG:
            result = cdvdman_stat.intr_ef;
            break;
        case CDSC_IO_SEMA:
            if (*param) {
                WaitSema(cdrom_io_sema);
            } else
                SignalSema(cdrom_io_sema);
            result = *param;
            break;
        case CDSC_GET_VERSION:
            result = CDVDMAN_MODULE_VERSION;
            break;
        case CDSC_GET_DEBUG_STATUS:
            *param = (int)&cdvdman_debug_print_flag;
            result = 0xFF;
            break;
        case CDSC_SET_ERROR:
            result = cdvdman_stat.err = *param;
            break;
        case CDSC_OPL_SHUTDOWN:
            oplShutdown(*param);
            result = 1;
            break;
        default:
            result = 1;
    }
    return result;
}

static int cdvdman_writeSCmd(u8 cmd, const void *in, u16 in_size, void *out, u16 out_size)
{
    int i;
    u8 *p;
    WaitSema(cdvdman_scmdsema);
    if (CDVDreg_SDATAIN & 0x80) {
        SignalSema(cdvdman_scmdsema);
        return 0;
    }
    if (!(CDVDreg_SDATAIN & 0x40)) {
        do {
            (void)CDVDreg_SDATAOUT;
        } while (!(CDVDreg_SDATAIN & 0x40));
    }
    if (in_size > 0) {
        for (i = 0; i < in_size; i++) {
            p = (void *)((const u8 *)in + i);
            CDVDreg_SDATAIN = *p;
        }
    }
    CDVDreg_SCOMMAND = cmd;
    (void)CDVDreg_SCOMMAND;
    while (CDVDreg_SDATAIN & 0x80) {
        ;
    }
    i = 0;
    if (!(CDVDreg_SDATAIN & 0x40)) {
        do {
            if (i >= out_size) {
                break;
            }
            p = (void *)((u8 *)out + i);
            *p = CDVDreg_SDATAOUT;
            i++;
        } while (!(CDVDreg_SDATAIN & 0x40));
    }
    if (!(CDVDreg_SDATAIN & 0x40)) {
        do {
            (void)CDVDreg_SDATAOUT;
        } while (!(CDVDreg_SDATAIN & 0x40));
    }
    SignalSema(cdvdman_scmdsema);
    return 1;
}

int cdvdman_sendSCmd(u8 cmd, const void *in, u16 in_size, void *out, u16 out_size)
{
    int r, retryCount = 0;
retry:
    r = cdvdman_writeSCmd(cmd & 0xff, in, in_size, out, out_size);
    if (r == 0) {
        DelayThread(2000);
        if (++retryCount <= 2500)
            goto retry;
    }
    DelayThread(2000);
    return 1;
}

void cdvdman_cb_event(int reason)
{
    if (cb_data.user_cb != NULL) {
        cb_data.reason = reason;
        if (QueryIntrContext())
            iSetAlarm(&gCallbackSysClock, &event_alarm_cb, &cb_data);
        else
            SetAlarm(&gCallbackSysClock, &event_alarm_cb, &cb_data);
    } else {
        cdvdman_signal_read_end();
    }
}

static unsigned int event_alarm_cb(void *args)
{
    struct cdvdman_cb_data *cb_data = args;
    cdvdman_signal_read_end_intr();
    if (cb_data->user_cb != NULL)
        cb_data->user_cb(cb_data->reason);
    return 0;
}

static void cdvdman_signal_read_end(void)
{
    sync_flag = 0;
    SetEventFlag(cdvdman_stat.intr_ef, 9);
}

static void cdvdman_signal_read_end_intr(void)
{
    sync_flag = 0;
    iSetEventFlag(cdvdman_stat.intr_ef, 9);
}

static void cdvdman_cdread_Thread(void *args)
{
    while (1) {
        WaitSema(cdrom_rthread_sema);
        cdvdman_read(cdvdman_stat.cdread_lba, cdvdman_stat.cdread_sectors, cdvdman_stat.sector_size, cdvdman_stat.cdread_buf);
        if (Stm0Callback != NULL) {
            cdvdman_signal_read_end();
            if (Stm0Callback != NULL)
                Stm0Callback();
        } else
            cdvdman_cb_event(SCECdFuncRead);
    }
}

static void cdvdman_startThreads(void)
{
    iop_thread_t thread_param;
    cdvdman_stat.status = SCECdStatPause;
    cdvdman_stat.err = SCECdErNO;
    thread_param.thread = &cdvdman_cdread_Thread;
    thread_param.stacksize = 0x1000;
    thread_param.priority = 0x0f;
    thread_param.attr = TH_C;
    thread_param.option = 0xABCD0000;
    cdvdman_ReadingThreadID = CreateThread(&thread_param);
    StartThread(cdvdman_ReadingThreadID, NULL);
}

static void cdvdman_create_semaphores(void)
{
    iop_sema_t smp;
    smp.initial = 1;
    smp.max = 1;
    smp.attr = 0;
    smp.option = 0;
    cdvdman_scmdsema = CreateSema(&smp);
    smp.initial = 0;
    cdrom_rthread_sema = CreateSema(&smp);
}

static int intrh_cdrom(void *common)
{
    if (CDVDreg_PWOFF & CDL_DATA_RDY)
        CDVDreg_PWOFF = CDL_DATA_RDY;
    if (CDVDreg_PWOFF & CDL_DATA_END) {
        if (cdvdman_settings.common.flags & IOPCORE_ENABLE_POFF) {
            CDVDreg_PWOFF = CDL_DATA_END;
        }
        iSetEventFlag(cdvdman_stat.intr_ef, 0x14);
#ifdef __USE_DEV9
        if (cdvdman_settings.common.flags & IOPCORE_ENABLE_POFF) {
            iWakeupThread(POFFThreadID);
        }
#endif
    } else
        CDVDreg_PWOFF = CDL_DATA_COMPLETE;
    return 1;
}

static inline void InstallIntrHandler(void)
{
    RegisterIntrHandler(IOP_IRQ_CDVD, 1, &intrh_cdrom, NULL);
    EnableIntr(IOP_IRQ_CDVD);
    if (CDVDreg_PWOFF & CDL_DATA_END)
        CDVDreg_PWOFF = CDL_DATA_END;
    if (CDVDreg_PWOFF & CDL_DATA_RDY)
        CDVDreg_PWOFF = CDL_DATA_RDY;
}

int _start(int argc, char **argv)
{
    RegisterLibraryEntries(&_exp_cdvdman);
    RegisterLibraryEntries(&_exp_cdvdstm);
    RegisterLibraryEntries(&_exp_smsutils);
#ifdef __USE_DEV9
    RegisterLibraryEntries(&_exp_dev9);
    dev9d_init();
#endif
    DeviceInit();
    RegisterLibraryEntries(&_exp_oplutils);
    USec2SysClock((cdvdman_settings.common.flags & IOPCORE_COMPAT_ACCU_READS) ? 5000 : 0, &gCallbackSysClock);
    cdvdman_create_semaphores();
    cdvdman_startThreads();
    cdvdman_initdev();
    InstallIntrHandler();
    hookMODLOAD();
    cdvdman_initDiskType();
    return MODULE_RESIDENT_END;
}

void SetStm0Callback(StmCallback_t callback)
{
    Stm0Callback = callback;
}

int _shutdown(void)
{
    DeviceDeinit();
    return 0;
}