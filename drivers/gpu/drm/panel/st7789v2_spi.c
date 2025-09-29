// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix ST7789V
 * display controller in SPI mode.
 *
 * Copyright 2025 Aleksandr Skupov <skupov93@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>


#define ST7789_CMD_PORCTRL      0xB2 //  Porch Setting
#define ST7789_CMD_GCTRL        0xB7 // Gate Control
#define ST7789_CMD_VCOMS        0xBB // VCOM Setting
#define ST7789_CMD_LCMCTRL      0xC0 // LCM Control
#define ST7789_CMD_VDVVRHEN     0xC2 // VDV and VRH Command Enable
#define ST7789_CMD_VRHS         0xC3 // VRH Set
#define ST7789_CMD_VDVS         0xC4 // VDV Set
#define ST7789_CMD_FRCTRL2      0xC6 // Frame Rate Control in Normal Mode
#define ST7789_CMD_PWCTRL1      0xD0 // Power Control 1
#define ST7789_CMD_PVGAMCTRL    0xE0 // Positive Voltage Gamma Control
#define ST7789_CMD_NVGAMCTRL    0xE1 // Negative Voltage Gamma Control


#define ST7789_MY	BIT(7)
#define ST7789_MX	BIT(6)
#define ST7789_MV	BIT(5)
#define ST7789_ML   BIT(4)
#define ST7789_BRG	BIT(3)

struct st7789v_cfg {
	const struct drm_display_mode mode;
	unsigned int left_offset;
	unsigned int top_offset;
	unsigned int write_only:1;
	unsigned int rgb:1;		/* RGB (vs. BGR) */
};


struct st7789v_priv {
	struct mipi_dbi_dev dbidev;	/* Must be first for .release() */
	const struct st7789v_cfg *cfg;
};

// Инициализация дисплея
static void st7789v_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state) {

	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_plane *plane = &pipe->plane;
	struct drm_device *drm = crtc->dev;

    struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct st7789v_priv *priv = container_of(dbidev, struct st7789v_priv,
						 dbidev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;
	u8 addr_mode;

    DRM_DEBUG_KMS("\n");

    if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

    ret = mipi_dbi_poweron_reset(dbidev);
	if (ret)
		goto out_exit;

	msleep(150);

    mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(500);

    mipi_dbi_command(dbi, ST7789_CMD_PORCTRL, 0x0C, 0x0C, 0x00, 0x33, 0x33);
    mipi_dbi_command(dbi, ST7789_CMD_GCTRL, 0x35);

    mipi_dbi_command(dbi, ST7789_CMD_VCOMS, 0x19); // VCOM setting 0.725v (Default: 0.75v -> 0x20)
	mipi_dbi_command(dbi, ST7789_CMD_LCMCTRL, 0x2C); // LCM Control
	mipi_dbi_command(dbi, ST7789_CMD_VDVVRHEN, 0x01); // VDV and VRH Command Enable
	mipi_dbi_command(dbi, ST7789_CMD_VRHS, 0x12); // VRH set+-4.45v (Default: +-4.1v -> 0x0B)
	mipi_dbi_command(dbi, ST7789_CMD_VDVS, 0x20); // VDV set
	mipi_dbi_command(dbi, ST7789_CMD_FRCTRL2, 0x0F); // Frame Rate Control in Normal Mode Default value (60HZ)
	mipi_dbi_command(dbi, ST7789_CMD_PWCTRL1, 0xA4, 0xA1); // Power control

	mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE); // Inversion ON

	dev_info(drm->dev, "Before set rotation ver 2 %d\n", dbidev->rotation);

	unsigned int top_offset = 80;
    
    u8 row_start_high = (top_offset >> 8) & 0xFF;
    u8 row_start_low = top_offset & 0xFF;
    u8 row_end_high = ((240 + top_offset - 1) >> 8) & 0xFF;
    u8 row_end_low = (240 + top_offset - 1) & 0xFF;
    
    mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
                     row_start_high, row_start_low,
                     row_end_high, row_end_low);
    switch (dbidev->rotation) {
	default:
		addr_mode = ST7789_MX | ST7789_MY;
		break;
	case 90:
		addr_mode = ST7789_MX | ST7789_MV;
		break;
	case 180:
		addr_mode = 0;
		break;
	case 270:
		addr_mode = ST7789_MY | ST7789_MV;
		break;
	}

	if (priv->cfg->rgb)
		addr_mode |= ST7789_BRG;
	
    mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT,
			 MIPI_DCS_PIXEL_FMT_16BIT);
	
    /* Division line */
	mipi_dbi_command(dbi, ST7789_CMD_PVGAMCTRL, 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23);
	mipi_dbi_command(dbi, ST7789_CMD_NVGAMCTRL, 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	msleep(100);

	mipi_dbi_command(dbi, MIPI_DCS_ENTER_NORMAL_MODE);

	msleep(20);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	DRM_MIPI_DBI_SIMPLE_DISPLAY_PIPE_FUNCS(st7789v_pipe_enable),
};


static const struct st7789v_cfg dwn_li24240_cfg = {
	.mode		= { DRM_SIMPLE_MODE(240, 240, 27, 27) },
	/* Cannot read from Adafruit 1.8" display via SPI */
	.top_offset = 80,
	.left_offset = 0,
	.write_only	= true,
    .rgb = false,
};

DEFINE_DRM_GEM_DMA_FOPS(st7789v_fops);

static const struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v2",
	.desc			= "Sitronix ST7789V2",
	.major			= 1,
	.minor			= 0,
};

static int st7789fb_probe(struct spi_device *spi) {
    struct device *dev = &spi->dev;
    const struct st7789v_cfg *cfg;
    struct mipi_dbi_dev *dbidev;
	struct st7789v_priv *priv;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

    cfg = device_get_match_data(&spi->dev);
	if (!cfg)
		cfg = (void *)spi_get_device_id(spi)->driver_data;

    priv = devm_drm_dev_alloc(dev, &st7789v_driver,
				  struct st7789v_priv, dbidev.drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	dbidev = &priv->dbidev;
	priv->cfg = cfg;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

    dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

    device_property_read_u32(dev, "rotation", &rotation);

    ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	if (cfg->write_only)
		dbi->read_commands = NULL;

	dbidev->left_offset = cfg->left_offset;
	dbidev->top_offset = cfg->top_offset;

	ret = mipi_dbi_dev_init(dbidev, &st7789v_pipe_funcs, &cfg->mode,
                    rotation);

    if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_client_setup(drm, NULL);

    return 0;
}

static void st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static const struct of_device_id st7789_dt_ids[] = {
    { .compatible = "sitronix,st7789v2", .data = &dwn_li24240_cfg},
    { }
};

static struct spi_driver st7789v_spi_driver = {
    .driver = {
        .name = "st7789v2",
        .of_match_table = st7789_dt_ids,
    },
    .probe = st7789fb_probe,
    .remove = st7789v_remove,
    .shutdown = st7789v_shutdown
};
module_spi_driver(st7789v_spi_driver);

MODULE_AUTHOR("Skupov Aleksandr");
MODULE_DESCRIPTION("ST7789 SPI Framebuffer Driver");
MODULE_LICENSE("GPL");