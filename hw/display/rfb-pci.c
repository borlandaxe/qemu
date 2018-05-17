
//#include "hw/pci/pci.h"
//
//

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qapi/visitor.h"

#include "ui/console.h"
#include "vga_int.h"

typedef struct PCIVGAState {
    PCIDevice dev;
    VGACommonState vga;
    uint32_t flags;
    MemoryRegion mmio;
    MemoryRegion mrs[3];
} PCIVGAState;


#define TYPE_PCI_VGA "pci-vga"
#define PCI_VGA(obj) OBJECT_CHECK(PCIVGAState, (obj), TYPE_PCI_VGA)

#define RFB(obj)        OBJECT_CHECK(RfbState, obj, "rfb")
#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;
	MemoryRegion vram; 

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define RFB_STATUS_COMPUTING    0x01
#define RFB_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define RFB_DMA_RUN             0x1
#define RFB_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define RFB_DMA_FROM_PCI       0
# define RFB_DMA_TO_PCI         1
#define RFB_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[1024*1024];
    uint64_t dma_mask;
}RfbState;

//RfbState *grfb  = NULL;
MemoryRegion *gpvram; 

static bool rfb_msi_enabled(RfbState *rfb)
{
    return msi_enabled(&rfb->pdev);
}

static void rfb_raise_irq(RfbState *rfb, uint32_t val)
{
    rfb->irq_status |= val;
    if (rfb->irq_status) {
        if (rfb_msi_enabled(rfb)) {
            msi_notify(&rfb->pdev, 0);
        } else {
            pci_set_irq(&rfb->pdev, 1);
        }
    }
}

static void rfb_lower_irq(RfbState *rfb, uint32_t val)
{
    rfb->irq_status &= ~val;

    if (!rfb->irq_status && !rfb_msi_enabled(rfb)) {
        pci_set_irq(&rfb->pdev, 0);
    }
}

static bool within(uint32_t addr, uint32_t start, uint32_t end)
{
    return start <= addr && addr < end;
}

static void rfb_check_range(uint32_t addr, uint32_t size1, uint32_t start,
                uint32_t size2)
{
    uint32_t end1 = addr + size1;
    uint32_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("RFB: DMA range 0x%.8x-0x%.8x out of bounds (0x%.8x-0x%.8x)!",
            addr, end1 - 1, start, end2 - 1);
}

static dma_addr_t rfb_clamp_addr(const RfbState *rfb, dma_addr_t addr)
{
    dma_addr_t res = addr & rfb->dma_mask;

    if (addr != res) {
        printf("RFB: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void rfb_dma_timer(void *opaque)
{
    RfbState *rfb = opaque;
    bool raise_irq = false;

    if (!(rfb->dma.cmd & RFB_DMA_RUN)) {
        return;
    }

    if (RFB_DMA_DIR(rfb->dma.cmd) == RFB_DMA_FROM_PCI) {
        uint32_t dst = rfb->dma.dst;
        rfb_check_range(dst, rfb->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&rfb->pdev, rfb_clamp_addr(rfb, rfb->dma.src),
                rfb->dma_buf + dst, rfb->dma.cnt);
    } else {
        uint32_t src = rfb->dma.src;
        rfb_check_range(src, rfb->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&rfb->pdev, rfb_clamp_addr(rfb, rfb->dma.dst),
                rfb->dma_buf + src, rfb->dma.cnt);
    }

    rfb->dma.cmd &= ~RFB_DMA_RUN;
    if (rfb->dma.cmd & RFB_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        rfb_raise_irq(rfb, DMA_IRQ);
    }
}

static void dma_rw(RfbState *rfb, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (rfb->dma.cmd & RFB_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&rfb->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static uint64_t rfb_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    RfbState *rfb = opaque;
    uint64_t val = ~0ULL;

    if (size != 4) {
        return val;
    }

    switch (addr) {
    case 0x00:
        val = 0x010000edu;
        break;
    case 0x04:
        val = rfb->addr4;
        break;
    case 0x08:
        qemu_mutex_lock(&rfb->thr_mutex);
        val = rfb->fact;
        qemu_mutex_unlock(&rfb->thr_mutex);
        break;
    case 0x20:
        val = atomic_read(&rfb->status);
        break;
    case 0x24:
        val = rfb->irq_status;
        break;
    case 0x80:
        dma_rw(rfb, false, &val, &rfb->dma.src, false);
        break;
    case 0x88:
        dma_rw(rfb, false, &val, &rfb->dma.dst, false);
        break;
    case 0x90:
        dma_rw(rfb, false, &val, &rfb->dma.cnt, false);
        break;
    case 0x98:
        dma_rw(rfb, false, &val, &rfb->dma.cmd, false);
        break;
    }

    return val;
}

static void rfb_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    RfbState *rfb = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return;
    }

    switch (addr) {
    case 0x04:
        rfb->addr4 = ~val;
        break;
    case 0x08:
        if (atomic_read(&rfb->status) & RFB_STATUS_COMPUTING) {
            break;
        }
        /* RFB_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&rfb->thr_mutex);
        rfb->fact = val;
        atomic_or(&rfb->status, RFB_STATUS_COMPUTING);
        qemu_cond_signal(&rfb->thr_cond);
        qemu_mutex_unlock(&rfb->thr_mutex);
        break;
    case 0x20:
        if (val & RFB_STATUS_IRQFACT) {
            atomic_or(&rfb->status, RFB_STATUS_IRQFACT);
        } else {
            atomic_and(&rfb->status, ~RFB_STATUS_IRQFACT);
        }
        break;
    case 0x60:
        rfb_raise_irq(rfb, val);
        break;
    case 0x64:
        rfb_lower_irq(rfb, val);
        break;
    case 0x80:
        dma_rw(rfb, true, &val, &rfb->dma.src, false);
        break;
    case 0x88:
        dma_rw(rfb, true, &val, &rfb->dma.dst, false);
        break;
    case 0x90:
        dma_rw(rfb, true, &val, &rfb->dma.cnt, false);
        break;
    case 0x98:
        if (!(val & RFB_DMA_RUN)) {
            break;
        }
        dma_rw(rfb, true, &val, &rfb->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps rfb_mmio_ops = {
    .read = rfb_mmio_read,
    .write = rfb_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void *rfb_fact_thread(void *opaque)
{
    RfbState *rfb = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&rfb->thr_mutex);
        while ((atomic_read(&rfb->status) & RFB_STATUS_COMPUTING) == 0 &&
                        !rfb->stopping) {
            qemu_cond_wait(&rfb->thr_cond, &rfb->thr_mutex);
        }

        if (rfb->stopping) {
            qemu_mutex_unlock(&rfb->thr_mutex);
            break;
        }

        val = rfb->fact;
        qemu_mutex_unlock(&rfb->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&rfb->thr_mutex);
        rfb->fact = ret;
        qemu_mutex_unlock(&rfb->thr_mutex);
        atomic_and(&rfb->status, ~RFB_STATUS_COMPUTING);

        if (atomic_read(&rfb->status) & RFB_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            rfb_raise_irq(rfb, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_rfb_realize(PCIDevice *pdev, Error **errp)
{
    //PCIVGAState *d = PCI_VGA(pdev);
    //VGACommonState *s = &d->vga;

	RfbState *rfb = DO_UPCAST(RfbState, pdev, pdev);
    uint8_t *pci_conf = pdev->config;

	printf("pci_rfb_realize\n");

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&rfb->dma_timer, QEMU_CLOCK_VIRTUAL, rfb_dma_timer, rfb);

    qemu_mutex_init(&rfb->thr_mutex);
    qemu_cond_init(&rfb->thr_cond);
    qemu_thread_create(&rfb->thread, "rfb", rfb_fact_thread,
                       rfb, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&rfb->mmio, OBJECT(rfb), &rfb_mmio_ops, rfb,
                    "rfb-mmio", 1 << 20);
    memory_region_init_ram_nomigrate(&rfb->vram, OBJECT(pdev), "rfb.vram", 0x10<<20, &error_fatal);
	gpvram = &rfb->vram;
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &rfb->mmio);
    
	pci_register_bar(pdev, 1, PCI_BASE_ADDRESS_MEM_PREFETCH, &rfb->vram);
}

static void pci_rfb_uninit(PCIDevice *pdev)
{
    RfbState *rfb = DO_UPCAST(RfbState, pdev, pdev);

    qemu_mutex_lock(&rfb->thr_mutex);
    rfb->stopping = true;
    qemu_mutex_unlock(&rfb->thr_mutex);
    qemu_cond_signal(&rfb->thr_cond);
    qemu_thread_join(&rfb->thread);

    qemu_cond_destroy(&rfb->thr_cond);
    qemu_mutex_destroy(&rfb->thr_mutex);

    timer_del(&rfb->dma_timer);
}

static void rfb_obj_uint64(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    uint64_t *val = opaque;

    visit_type_uint64(v, name, val, errp);
}

static void rfb_instance_init(Object *obj)
{
	printf("%s %d %s stub...\n", __FILE__, __LINE__, __func__);
    RfbState *rfb = RFB(obj);

    rfb->dma_mask = (1UL << 28) - 1;
    object_property_add(obj, "dma_mask", "uint64", rfb_obj_uint64,
                    rfb_obj_uint64, NULL, &rfb->dma_mask, NULL);
	printf("%s %d %s stub...\n", __FILE__, __LINE__, __func__);
}

static void rfb_class_init(ObjectClass *class, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_rfb_realize;
    k->exit = pci_rfb_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x11e9;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
}

static void pci_rfb_register_types(void)
{
    static const TypeInfo rfb_info = {
        .name          = "rfb",
//        .parent        = TYPE_PCI_DEVICE,
        .parent        = TYPE_PCI_VGA,
        .instance_size = sizeof(RfbState),
        .instance_init = rfb_instance_init,
        .class_init    = rfb_class_init,
    };

    type_register_static(&rfb_info);
}

type_init(pci_rfb_register_types)
