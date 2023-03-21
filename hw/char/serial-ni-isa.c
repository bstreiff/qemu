/*
 * QEMU 16550A UART emulation
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2008 Citrix Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/acpi/acpi_aml_interface.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(ISASerialState, ISA_SERIAL)

struct ISASerialState {
    ISADevice parent_obj;

    uint32_t index;
    uint32_t iobase;
    uint32_t isairq;
    uint32_t pmr;
    uint32_t pcr;

    char *transceiver;

    SerialState state;
};

#define NI16550_TFS 0x0C
#define NI16550_RFS 0x0D
#define NI16550_PMR 0x0E
#define NI16550_PMR_CAP_MASK    0x03
#define NI16550_PMR_CAP_RS232   0x01
#define NI16550_PMR_CAP_RS485   0x02
#define NI16550_PMR_CAP_DUAL    0x03
#define NI16550_PMR_MODE_MASK   0x10
#define NI16550_PMR_MODE_RS232  0x00
#define NI16550_PMR_MODE_RS485  0x10
#define NI16550_PCR 0x0F

static void ni16550_ioport_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
    ISASerialState *s = opaque;

    assert(size == 1 && addr < 16);
    if (addr < 8) {
        /* standard 16550 register */
        serial_io_ops.write(&s->state, addr, val, size);
        return;
    }

    switch (addr) {
    default:
        return;
    case NI16550_PMR:
        /* only the mode side is writable */
        s->pmr = (s->pmr & ~NI16550_PMR_MODE_MASK) | (val & NI16550_PMR_MODE_MASK);
        return;
    case NI16550_PCR:
        s->pcr = (uint8_t)val;
        return;
    }
}

static uint64_t ni16550_ioport_read(void *opaque, hwaddr addr, unsigned size)
{
    ISASerialState *s = opaque;

    assert(size == 1 && addr < 16);
    if (addr < 8) {
        /* standard 16550 register */
        return serial_io_ops.read(&s->state, addr, size);
    }

    switch (addr) {
    default:
        return 0;
    case 0x0C: /* TX FIFO Size */
    case 0x0D: /* RX FIFO Size */
        return 128;
    case 0x0E: /* Port Mode Register */
        return s->pmr;
    case 0x0F: /* Port Control Register */
        return s->pcr;
    }
}

const MemoryRegionOps ni16550_io_ops = {
    .read = ni16550_ioport_read,
    .write = ni16550_ioport_write,
    .valid = {
        .unaligned = 1,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#define MAX_NI16550_SERIAL_PORTS 9

/* this mapping comes from sbRIO-96x8 */
static const int isa_serial_io[MAX_NI16550_SERIAL_PORTS] = {
    0x350, 0x360, 0x2E0, 0x2F0, 0x300, 0x310, 0x320, 0x330, 0x340,
};
static const int isa_serial_irq[MAX_NI16550_SERIAL_PORTS] = {
//    1, 3, 6, 7, 15, 10, 11, 9, 12
    3, 4, 3, 4, 3, 4, 3, 4, 3
};

static void serial_isa_realizefn(DeviceState *dev, Error **errp)
{
    static int autoindex = 0;
    ISADevice *isadev = ISA_DEVICE(dev);
    ISASerialState *isa = ISA_SERIAL(dev);
    SerialState *s = &isa->state;

    if (isa->index == -1) {
        isa->index = autoindex;
        autoindex++;
    }

    if (isa->transceiver == NULL) {
        /* default assumption is RS-485 */
        isa->pmr = NI16550_PMR_CAP_RS485|NI16550_PMR_MODE_RS485;
    } else {
        if (strcmp(isa->transceiver, "RS-232") == 0) {
            isa->pmr = NI16550_PMR_CAP_RS232|NI16550_PMR_MODE_RS232;
        } else if (strcmp(isa->transceiver, "RS-485") == 0) {
            isa->pmr = NI16550_PMR_CAP_RS485|NI16550_PMR_MODE_RS485;
        } else {
            error_setg(errp, "Unknown transceiver type %s, must be RS-232/RS-485/dual.",
                       isa->transceiver);
        }
    }

    if (isa->iobase == -1) {
        if (isa->index >= MAX_NI16550_SERIAL_PORTS) {
            error_setg(errp, "Autoconfig of NI16550 index %d requires iobase to be specified.",
                       isa->index);
            return;
        }

        isa->iobase = isa_serial_io[isa->index];
    }

    if (isa->isairq == -1) {
        /* try to find an IRQ that matches up with the iobase */
        for (int i = 0; i < MAX_NI16550_SERIAL_PORTS; ++i) {
            if (isa->iobase == isa_serial_io[i]) {
                isa->isairq = isa_serial_irq[i];
                break;
            }
        }
        if (isa->isairq == -1) {
            error_setg(errp, "Autoconfig of NI16550 index %d requires irq to be specified.",
                       isa->index);
            return;
        }
    }

    /* initialize the serial subdevice */
    s->irq = isa_get_irq(isadev, isa->isairq);
    qdev_realize(DEVICE(s), NULL, errp);
    qdev_set_legacy_instance_id(dev, isa->iobase, 3);

    memory_region_init_io(&s->io, OBJECT(isa), &ni16550_io_ops, isa, "serial", 16);
    isa_register_ioport(isadev, &s->io, isa->iobase);
}

static void serial_isa_build_aml(AcpiDevAmlIf *adev, Aml *scope)
{
    ISASerialState *isa = ISA_SERIAL(adev);
    Aml *dev;
    Aml *crs;
    Aml *clkfreq;
    Aml *prescaler;
    Aml *transceiver;
    Aml *fpga;
    Aml *props;
    Aml *dsd;

    crs = aml_resource_template();
    aml_append(crs, aml_io(AML_DECODE16, isa->iobase, isa->iobase, 0x00, 0x10));
    aml_append(crs, aml_irq_no_flags(isa->isairq));

    dev = aml_device("UAR%d", isa->index + 1);
    aml_append(dev, aml_name_decl("_HID", aml_eisaid("NIC7A69")));
    aml_append(dev, aml_name_decl("_UID", aml_int(isa->index + 1)));
    aml_append(dev, aml_name_decl("_DDN", aml_string("COM%d", isa->index + 1)));
    aml_append(dev, aml_name_decl("_STA", aml_int(0xf)));
    aml_append(dev, aml_name_decl("_CRS", crs));

    clkfreq = aml_package(0x2);
    aml_append(clkfreq, aml_string("clock-frequency"));
    aml_append(clkfreq, aml_int(29629629));
    prescaler = aml_package(0x2);
    aml_append(prescaler, aml_string("clock-prescaler"));
    aml_append(prescaler, aml_int(0));
    transceiver = aml_package(0x2);
    aml_append(transceiver, aml_string("transceiver"));
    if ((isa->pmr & NI16550_PMR_CAP_MASK) == NI16550_PMR_CAP_RS232) {
        aml_append(transceiver, aml_string("RS-232"));
    } else {
        aml_append(transceiver, aml_string("RS-485"));
    }
    fpga = aml_package(0x2);
    aml_append(fpga, aml_string("fpga"));
    aml_append(fpga, aml_int(0));
    props = aml_package(0x4);
    aml_append(props, clkfreq);
    aml_append(props, prescaler);
    aml_append(props, transceiver);
    aml_append(props, fpga);
    dsd = aml_package(0x2);
    aml_append(dsd, aml_touuid("DAFFD814-6EBA-4D8C-8A91-BC9BBF4AA301"));
    aml_append(dsd, props);
    aml_append(dev, aml_name_decl("_DSD", dsd));

    aml_append(scope, dev);
}

static const VMStateDescription vmstate_isa_serial = {
    .name = "serial-ni",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(state, ISASerialState, 0, vmstate_serial, SerialState),
        VMSTATE_END_OF_LIST()
    }
};

static Property serial_isa_properties[] = {
    DEFINE_PROP_UINT32("index",  ISASerialState, index,   -1),
    DEFINE_PROP_UINT32("iobase", ISASerialState, iobase,  -1),
    DEFINE_PROP_UINT32("irq",    ISASerialState, isairq,  -1),
    DEFINE_PROP_STRING("transceiver", ISASerialState, transceiver),
    DEFINE_PROP_END_OF_LIST(),
};

static void serial_isa_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AcpiDevAmlIfClass *adevc = ACPI_DEV_AML_IF_CLASS(klass);

    dc->realize = serial_isa_realizefn;
    dc->vmsd = &vmstate_isa_serial;
    adevc->build_dev_aml = serial_isa_build_aml;
    device_class_set_props(dc, serial_isa_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static void serial_isa_initfn(Object *o)
{
    ISASerialState *self = ISA_SERIAL(o);

    object_initialize_child(o, "serial", &self->state, TYPE_SERIAL);

    qdev_alias_all_properties(DEVICE(&self->state), o);
}

static const TypeInfo serial_isa_info = {
    .name          = "isa-serial-ni",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISASerialState),
    .instance_init = serial_isa_initfn,
    .class_init    = serial_isa_class_initfn,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_ACPI_DEV_AML_IF },
        { },
    },
};

static void serial_register_types(void)
{
    type_register_static(&serial_isa_info);
}

type_init(serial_register_types)
