// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * ASMedia xHCI firmware loader
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/acpi.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include "xhci.h"
#include "xhci-trace.h"
#include "xhci-pci.h"

/* Configuration space registers */
#define ASMT_CFG_CONTROL		0xe0
#define ASMT_CFG_CONTROL_WRITE		BIT(1)
#define ASMT_CFG_CONTROL_READ		BIT(0)

#define ASMT_CFG_SRAM_ADDR		0xe2

#define ASMT_CFG_SRAM_ACCESS		0xef
#define ASMT_CFG_SRAM_ACCESS_READ	BIT(6)
#define ASMT_CFG_SRAM_ACCESS_ENABLE	BIT(7)

#define ASMT_CFG_DATA_READ0		0xf0
#define ASMT_CFG_DATA_READ1		0xf4

#define ASMT_CFG_DATA_WRITE0		0xf8
#define ASMT_CFG_DATA_WRITE1		0xfc

#define ASMT_CMD_GET_FWVER		0x8000060840
#define ASMT_FWVER_ROM			0x010250090816

/* BAR0 registers */
#define ASMT_REG_ADDR			0x3000

#define ASMT_REG_DATA			0x3004

#define ASMT_REG_STATUS			0x3009
#define ASMT_REG_STATUS_BUSY		BIT(7)

#define ASMT_REG_WDATA			0x3010
#define ASMT_REG_RDATA			0x3018

#define TIMEOUT_USEC			10000
#define RESET_TIMEOUT_USEC		500000

static int asmedia_mbox_tx(struct pci_dev *pdev, u64 data)
{
	u8 op;
	int i;

	for (i = 0; i < TIMEOUT_USEC; i++) {
		pci_read_config_byte(pdev, ASMT_CFG_CONTROL, &op);
		if (!(op & ASMT_CFG_CONTROL_WRITE))
			break;
		udelay(1);
	}

	if (op & ASMT_CFG_CONTROL_WRITE) {
		dev_err(&pdev->dev,
			"Timed out on mailbox tx: 0x%llx\n",
			data);
		return -ETIMEDOUT;
	}

	pci_write_config_dword(pdev, ASMT_CFG_DATA_WRITE0, data);
	pci_write_config_dword(pdev, ASMT_CFG_DATA_WRITE1, data >> 32);
	pci_write_config_byte(pdev, ASMT_CFG_CONTROL,
			      ASMT_CFG_CONTROL_WRITE);

	return 0;
}

static int asmedia_mbox_rx(struct pci_dev *pdev, u64 *data)
{
	u8 op;
	u32 low, high;
	int i;

	for (i = 0; i < TIMEOUT_USEC; i++) {
		pci_read_config_byte(pdev, ASMT_CFG_CONTROL, &op);
		if (op & ASMT_CFG_CONTROL_READ)
			break;
		udelay(1);
	}

	if (!(op & ASMT_CFG_CONTROL_READ)) {
		dev_err(&pdev->dev, "Timed out on mailbox rx\n");
		return -ETIMEDOUT;
	}

	pci_read_config_dword(pdev, ASMT_CFG_DATA_READ0, &low);
	pci_read_config_dword(pdev, ASMT_CFG_DATA_READ1, &high);
	pci_write_config_byte(pdev, ASMT_CFG_CONTROL,
			      ASMT_CFG_CONTROL_READ);

	*data = ((u64)high << 32) | low;
	return 0;
}

static int asmedia_get_fw_version(struct pci_dev *pdev, u64 *version)
{
	int err = 0;
	u64 cmd;

	err = asmedia_mbox_tx(pdev, ASMT_CMD_GET_FWVER);
	if (err)
		return err;
	err = asmedia_mbox_tx(pdev, 0);
	if (err)
		return err;

	err = asmedia_mbox_rx(pdev, &cmd);
	if (err)
		return err;
	err = asmedia_mbox_rx(pdev, version);
	if (err)
		return err;

	if (cmd != ASMT_CMD_GET_FWVER) {
		dev_err(&pdev->dev, "Unexpected reply command 0x%llx\n", cmd);
		return -EIO;
	}

	return 0;
}

static bool asmedia_check_firmware(struct pci_dev *pdev)
{
	u64 fwver;
	int ret;

	ret = asmedia_get_fw_version(pdev, &fwver);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Firmware version: 0x%llx\n", fwver);

	return fwver != ASMT_FWVER_ROM;
}

static int asmedia_wait_reset(struct pci_dev *pdev)
{
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);
	struct xhci_cap_regs __iomem *cap = hcd->regs;
	struct xhci_op_regs __iomem *op;
	u32 val;
	int ret;

	op = hcd->regs + HC_LENGTH(readl(&cap->hc_capbase));

	ret = readl_poll_timeout(&op->command,
				 val, !(val & CMD_RESET),
				 1000, RESET_TIMEOUT_USEC);

	if (!ret)
		return 0;

	dev_err(hcd->self.controller, "Reset timed out, trying to kick it\n");

	pci_write_config_byte(pdev, ASMT_CFG_SRAM_ACCESS,
			      ASMT_CFG_SRAM_ACCESS_ENABLE);

	pci_write_config_byte(pdev, ASMT_CFG_SRAM_ACCESS, 0);

	ret = readl_poll_timeout(&op->command,
				 val, !(val & CMD_RESET),
				 1000, RESET_TIMEOUT_USEC);

	if (ret)
		dev_err(hcd->self.controller, "Reset timed out, giving up\n");

	return ret;
}

static void asmedia_write_reg(struct usb_hcd *hcd, u16 addr, u8 data) {
	void __iomem *regs = hcd->regs;
	u8 status;
	int ret;

	writew_relaxed(addr, regs + ASMT_REG_ADDR);

	ret = readb_poll_timeout(regs + ASMT_REG_STATUS,
				 status, !(status & ASMT_REG_STATUS_BUSY),
				 1000, TIMEOUT_USEC);

	if (ret)
		dev_err(hcd->self.controller,
			"Write addr timed out ([%04x] = %02x)\n",
			addr, data);

	writeb_relaxed(data, regs + ASMT_REG_DATA);

	ret = readb_poll_timeout(regs + ASMT_REG_STATUS,
				 status, !(status & ASMT_REG_STATUS_BUSY),
				 1000, TIMEOUT_USEC);

	if (ret)
		dev_err(hcd->self.controller,
			"Write data timed out ([%04x] = %02x)\n",
			addr, data);
}

static int asmedia_load_fw(struct pci_dev *pdev, const struct firmware *fw)
{
	struct usb_hcd *hcd;
	void __iomem *regs;
	const u16 *fw_data = (const u16 *)fw->data;
	u32 data;
	size_t index = 0, addr = 0;
	size_t words = fw->size >> 1;
	int ret;

	hcd = dev_get_drvdata(&pdev->dev);
	regs = hcd->regs;

	asmedia_write_reg(hcd, 0x5040, 2);
	asmedia_write_reg(hcd, 0x5042, 1);

	ret = asmedia_wait_reset(pdev);
	if (ret) {
		dev_err(hcd->self.controller, "Failed pre-upload reset\n");
		return ret;
	}

	asmedia_write_reg(hcd, 0x500e, 1);

	pci_write_config_byte(pdev, ASMT_CFG_SRAM_ACCESS,
			      ASMT_CFG_SRAM_ACCESS_ENABLE);

	/* The firmware upload is interleaved in 0x4000 word blocks */
	addr = index = 0;
	while (index < words) {
		data = fw_data[index];
		if ((index | 0x4000) < words)
			data |= fw_data[index | 0x4000] << 16;

		pci_write_config_word(pdev, ASMT_CFG_SRAM_ADDR,
				      addr);

		writel_relaxed(data, regs + ASMT_REG_WDATA);

		if (++index & 0x4000)
			index += 0x4000;
		addr += 2;
	}

	asmedia_write_reg(hcd, 0x5040, 3);

	pci_write_config_byte(pdev, ASMT_CFG_SRAM_ACCESS, 0);

	asmedia_write_reg(hcd, 0x500e, 0);

	ret = asmedia_wait_reset(pdev);
	if (ret) {
		dev_err(hcd->self.controller, "Failed post-upload reset\n");
		return ret;
	}

	return 0;
}

int asmedia_xhci_check_request_fw(struct pci_dev *pdev,
				  const struct pci_device_id *id)
{
	struct xhci_driver_data *driver_data =
			(struct xhci_driver_data *)id->driver_data;
	const char *fw_name = driver_data->firmware;
	const struct firmware *fw;
	int ret;

	/* Check if device has firmware, if so skip everything */
	ret = asmedia_check_firmware(pdev);
	if (ret < 0)
		return ret;
	else if (ret == 1)
		return 0;

	pci_dev_get(pdev);
	ret = request_firmware(&fw, fw_name, &pdev->dev);
	pci_dev_put(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Could not load firmware %s: %d\n",
			fw_name, ret);
		return ret;
	}

	ret = asmedia_load_fw(pdev, fw);
	if (ret) {
		dev_err(&pdev->dev, "Firmware upload failed: %d\n", ret);
		goto err;
	}

	ret = asmedia_check_firmware(pdev);
	if (ret < 0) {
		goto err;
	} else if (ret != 1) {
		dev_err(&pdev->dev, "Firmware version is too old after upload\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

err:
	release_firmware(fw);
	return ret;
}
