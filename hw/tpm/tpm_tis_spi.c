#include "qemu/osdep.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/acpi/tpm.h"
#include "tpm_prop.h"
#include "tpm_tis.h"
#include "qom/object.h"
#include "hw/ssi/ssi.h"

#define TPM_TIS_SPI_ADDR_BYTES 3

typedef enum {
    TIS_SPI_PKT_STATE_DEACTIVATED = 0,
    TIS_SPI_PKT_STATE_START,
    TIS_SPI_PKT_STATE_ADDRESS,
    TIS_SPI_PKT_STATE_DATA_WR,
    TIS_SPI_PKT_STATE_DATA_RD,
    TIS_SPI_PKT_STATE_DONE,
} TisSpiPktState;

union TpmTisRWSizeByte {
    uint8_t byte;
    struct {
        uint8_t data_expected_size:6;
        uint8_t resv:1;
        uint8_t rwflag:1;
    }
}

union TpmTisSpiHwAddr {
    hwaddr addr;
    uint8_t bytes[sizeof(hwaddr)];
}

union TpmTisSpiData {
    uint32_t data;
    uint8_t bytes[sizeof(uint32_t)];
}

struct TPMSpiState {
    /*< private >*/
    SSIPeripheral parent_obj;

    /*< public >*/
    TPMState tpm_state; /* not a QOM object */
    TisSpiPktState tis_spi_state;

    union TpmTisRWSizeByte first_byte;
    union TpmTisSpiHwAddr addr;
    union TpmTisSpiData data;

    uint8_t data_idx;
    uint8_t addr_idx;
};

OBJECT_DECLARE_SIMPLE_TYPE(TPMSpiState, TPM_TIS_SPI)

static uint32_t tpm_tis_spi_transfer8_ex(SSIPeripheral *ss, uint32_t tx)
{
    TPMSpiState *tts = TPM_TIS_SPI(ss);
    SSIBus *ssibus = (SSIBus *)qdev_get_parent_bus(DEVICE(s));

    uint8_t prev_state = s->tis_spi_state;
    uint32_t r = tpm_tis_spi_transfer8(ss, tx);
    uint8_t curr_state = s->tis_spi_state;

    if (ssibus->preread &&
       /* cmd state has changed into DATA reading state */
       prev_state != TIS_SPI_PKT_STATE_DATA_RD &&
       curr_state == TIS_SPI_PKT_STATE_DATA_RD) {
        r = tpm_tis_spi_transfer8(ss, 0xFF);
    }

    return r;
}

static uint32_t tpm_tis_spi_transfer8(SSIPeripheral *ss, uint32_t tx)
{
    TPMSpiState *tts = TPM_TIS_SPI(ss);
    uint32_t r = 1;

    switch (s->tis_spi_state) {
    case TIS_SPI_PKT_STATE_START:
        tts->first_byte.byte = (uint8_t)tx;
        tts->data_idx = 0;
        tts->addr_idx = TPM_TIS_SPI_ADDR_BYTES;
        tts->tpm_state = TIS_SPI_PKT_STATE_ADDRESS;
        break;
    case TIS_SPI_PKT_STATE_ADDRESS:
        assert(tts->addr_idx > 0);
        tts->addr.bytes[--tts->addr_idx] = (uint8_t)tx;

        if (tts->addr_idx == 0) {
            if (tts->first_byte.rwflag == TIS_SPI_PKT_STATE_DATA_WR) {
                tts->tpm_state = TIS_SPI_PKT_STATE_DATA_WR;
            } else { /* read and get the data ready */
                tts->data = (uint32_t)tpm_tis_mmio_read(
                                      &tts->tpm_state,
                                      tts->addr.addr,
                                      tts->first_byte.data_expected_size);
                tts->tpm_state = TIS_SPI_PKT_STATE_DATA_RD;
            }
        }
        break;
    case TIS_SPI_PKT_STATE_DATA_WR:
        tts->data.bytes[tts->data_idx++] = (uint8_t)tx;
        if (tts->data_idx == tts->first_byte.data_epected_sized) {
            tpm_tis_mmio_write(&tts->tpm_state,
                               tts->addr.addr,
                               tts->data.data,
                               tts->first_byte.data_expected_size);
             tts->spi_state = TIS_SPI_PKT_STATE_DONE;
        }
        break;
    case TIS_SPI_PKT_STATE_DATA_RD:
        r = tts->data.bytes[tts->data_idx++];
        if (tts->data_idx == tts->first_byte.data_expected_size) {
            tts->spi_state = TIS_SPI_PKT_STATE_DONE;
        }
        break;
    default:
        tts->tpm_state = TIS_SPI_PKT_STATE_DEACTIVATED;
        r = uint32_t (-1);
    }

    return r;
}

static int tpm_tis_spi_cs(SSIPeripheral *ss, bool select)
{
    TPMStateSpi *tts = TPM_TIS_SPI(ss);

    if (select) { /* cs de-assert */
        tts->spi_state = TIS_SPI_PKT_STATE_DEACTIVATED;
    } else {
        tts->spi_state = TIS_SPI_PKT_STATE_START;
        tts->first_byte = 0;
        tts->addr = 0;
        tts->data = 0;
    }
    return 0;
}

static int tpm_tis_pre_save_spi(void *opaque)
{
    TPMSpiState *sbdev = opaque;

    return tpm_tis_pre_save(&sbdev->state);
}

static const VMStateDescription vmstate_tpm_tis_spi = {
    .name = "tpm-tis-spi",
    .version_id = 0,
    .pre_save  = tpm_tis_pre_save_spi,
    .fields = (VMStateField[]) {
        VMSTATE_BUFFER(state.buffer, TPMSpiState),
        VMSTATE_UINT16(state.rw_offset, TPMSpiState),
        VMSTATE_UINT8(state.active_locty, TPMSpiState),
        VMSTATE_UINT8(state.aborting_locty, TPMSpiState),
        VMSTATE_UINT8(state.next_locty, TPMSpiState),

        VMSTATE_STRUCT_ARRAY(state.loc, TPMSpiState, TPM_TIS_NUM_LOCALITIES,
                             0, vmstate_locty, TPMLocality),

        VMSTATE_END_OF_LIST()
    }
};

static void tpm_tis_spi_request_completed(TPMIf *ti, int ret)
{
    TPMSpiState *sbdev = TPM_TIS_SPI(ti);
    TPMState *s = &sbdev->state;

    tpm_tis_request_completed(s, ret);
}

static enum TPMVersion tpm_tis_spi_get_tpm_version(TPMIf *ti)
{
    TPMSpiState *sbdev = TPM_TIS_SPI(ti);
    TPMState *s = &sbdev->state;

    return tpm_tis_get_tpm_version(s);
}

static void tpm_tis_spi_reset(DeviceState *dev)
{
    TPMSpiState *sbdev = TPM_TIS_SPI(dev);
    TPMState *s = &sbdev->state;

    return tpm_tis_reset(s);
}

static Property tpm_tis_spi_properties[] = {
    DEFINE_PROP_UINT32("irq", TPMSpiState, state.irq_num, TPM_TIS_IRQ),
    DEFINE_PROP_TPMBE("tpmdev", TPMSpiState, state.be_driver),
    DEFINE_PROP_BOOL("ppi", TPMSpiState, state.ppi_enabled, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void tpm_tis_spi_initfn(Object *obj)
{
    TPMSpiState *sbdev = TPM_TIS_SPI(obj);
    TPMState *s = &sbdev->state;
}

static void tpm_tis_spi_realizefn(DeviceState *dev, Error **errp)
{
    TPMSpiState *sbdev = TPM_TIS_SPI(dev);
    TPMState *s = &sbdev->state;

    if (!tpm_find()) {
        error_setg(errp, "at most one TPM device is permitted");
        return;
    }

    if (!s->be_driver) {
        error_setg(errp, "'tpmdev' property is required");
        return;
    }
}

static void tpm_tis_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    TPMIfClass *tc = TPM_IF_CLASS(klass);

    device_class_set_props(dc, tpm_tis_spi_properties);

    k->transfer = tpm_tis_spi_transfer8;
    k->set_cs = tpm_tis_spi_cs;
    dc->vmsd  = &vmstate_tpm_tis_spi;
    tc->model = TPM_MODEL_TPM_TIS;
    dc->realize = tpm_tis_spi_realizefn;
    dc->reset = tpm_tis_spi_reset;
    tc->request_completed = tpm_tis_spi_request_completed;
    tc->get_version = tpm_tis_spi_get_tpm_version;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo tpm_tis_spi_info = {
    .name = TYPE_TPM_TIS_SPI,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(TPMSpiState),
    .instance_init = tpm_tis_spi_initfn,
    .class_init  = tpm_tis_spi_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_TPM_IF },
        { }
    }
};

static void tpm_tis_spi_register(void)
{
    type_register_static(&tpm_tis_spi_info);
}

type_init(tpm_tis_spi_register)
