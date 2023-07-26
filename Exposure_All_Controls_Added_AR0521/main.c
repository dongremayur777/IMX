#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-subdev.h>
//#include "imx_ap1302.h"
#include "sensor_tbls.h"
#include "otp_flash.h"




#define AP1302_REG_16BIT(n) ((2 << 24) | (n))

#define AP1302_AE_CTRL			AP1302_REG_16BIT(0x5002)
#define AP1302_AE_CTRL_STATS_SEL		BIT(11)
#define AP1302_AE_CTRL_IMM				BIT(10)
#define AP1302_AE_CTRL_ROUND_ISO		BIT(9)
#define AP1302_AE_CTRL_UROI_FACE		BIT(7)
#define AP1302_AE_CTRL_UROI_LOCK		BIT(6)
#define AP1302_AE_CTRL_UROI_BOUND		BIT(5)
#define AP1302_AE_CTRL_IMM1				BIT(4)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN	(0U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_EXP_TIME	(1U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_GAIN		(2U << 0)
#define AP1302_AE_CTRL_MANUAL_BV_ISO		(3U << 0)
#define AP1302_AE_CTRL_AUTO_BV_EXP_TIME		(9U << 0)
#define AP1302_AE_CTRL_AUTO_BV_GAIN			(10U << 0)
#define AP1302_AE_CTRL_AUTO_BV_ISO			(11U << 0)
#define AP1302_AE_CTRL_FULL_AUTO			(12U << 0)
#define AP1302_AE_CTRL_MODE_MASK		0x000f
#define AP1302_AE_MET				AP1302_REG_16BIT(0x503E)
#define AP1302_DZ_TGT_FCT			AP1302_REG_16BIT(0x1010)
#define AP1302_SFX_MODE				AP1302_REG_16BIT(0x1016)
#define AP1302_SFX_MODE_SFX_NORMAL		(0U << 0)
#define AP1302_SFX_MODE_SFX_ALIEN		(1U << 0)
#define AP1302_SFX_MODE_SFX_ANTIQUE		(2U << 0)
#define AP1302_SFX_MODE_SFX_BW			(3U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS		(4U << 0)
#define AP1302_SFX_MODE_SFX_EMBOSS_COLORED	(5U << 0)
#define AP1302_SFX_MODE_SFX_GRAYSCALE		(6U << 0)
#define AP1302_SFX_MODE_SFX_NEGATIVE		(7U << 0)
#define AP1302_SFX_MODE_SFX_BLUISH		(8U << 0)
#define AP1302_SFX_MODE_SFX_GREENISH		(9U << 0)
#define AP1302_SFX_MODE_SFX_REDISH		(10U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE1		(11U << 0)
#define AP1302_SFX_MODE_SFX_POSTERIZE2		(12U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA1		(13U << 0)
#define AP1302_SFX_MODE_SFX_SEPIA2		(14U << 0)
#define AP1302_SFX_MODE_SFX_SKETCH		(15U << 0)
#define AP1302_SFX_MODE_SFX_SOLARIZE		(16U << 0)
#define AP1302_SFX_MODE_SFX_FOGGY		(17U << 0)
#define AP1302_BUBBLE_OUT_FMT			AP1302_REG_16BIT(0x1164)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_BUBBLE_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_BUBBLE_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_ATOMIC				AP1302_REG_16BIT(0x1184)
#define AP1302_ATOMIC_MODE			BIT(2)
#define AP1302_ATOMIC_FINISH			BIT(1)
#define AP1302_ATOMIC_RECORD			BIT(0)
#define AP1302_PREVIEW_WIDTH			AP1302_REG_16BIT(0x2000)
#define AP1302_PREVIEW_HEIGHT			AP1302_REG_16BIT(0x2002)
#define AP1302_PREVIEW_ROI_X0			AP1302_REG_16BIT(0x2004)
#define AP1302_PREVIEW_ROI_Y0			AP1302_REG_16BIT(0x2006)
#define AP1302_PREVIEW_ROI_X1			AP1302_REG_16BIT(0x2008)
#define AP1302_PREVIEW_ROI_Y1			AP1302_REG_16BIT(0x200a)
#define AP1302_PREVIEW_OUT_FMT			AP1302_REG_16BIT(0x2012)
#define AP1302_PREVIEW_OUT_FMT_IPIPE_BYPASS	BIT(13)
#define AP1302_PREVIEW_OUT_FMT_SS		BIT(12)
#define AP1302_PREVIEW_OUT_FMT_FAKE_EN		BIT(11)
#define AP1302_PREVIEW_OUT_FMT_ST_EN		BIT(10)
#define AP1302_PREVIEW_OUT_FMT_IIS_NONE		(0U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_POST_VIEW	(1U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_VIDEO	(2U << 8)
#define AP1302_PREVIEW_OUT_FMT_IIS_BUBBLE	(3U << 8)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_422	(0U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_JPEG_420	(1U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV		(3U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RGB		(4U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_YUV_JFIF	(5U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW8		(8U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW10		(9U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW12		(10U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_RAW16		(11U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG8		(12U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG10		(13U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG12		(14U << 4)
#define AP1302_PREVIEW_OUT_FMT_FT_DNG16		(15U << 4)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_ROTATE	BIT(2)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_SCAN	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_JFIF	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_JPEG_EXIF	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_888	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_565	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555M	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RGB_555L	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_422	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_420	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_YUV_400	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_SENSOR	(0U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CAPTURE	(1U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CP	(2U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_BPC	(3U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_IHDR	(4U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PP	(5U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_DENSH	(6U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_PM	(7U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_GC	(8U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CURVE	(9U << 0)
#define AP1302_PREVIEW_OUT_FMT_FST_RAW_CCONV	(10U << 0)
#define AP1302_PREVIEW_S1_SENSOR_MODE		AP1302_REG_16BIT(0x202e)
#define AP1302_PREVIEW_HINF_CTRL		AP1302_REG_16BIT(0x2030)
#define AP1302_PREVIEW_HINF_CTRL_BT656_LE	BIT(15)
#define AP1302_PREVIEW_HINF_CTRL_BT656_16BIT	BIT(14)
#define AP1302_PREVIEW_HINF_CTRL_MUX_DELAY(n)	((n) << 8)
#define AP1302_PREVIEW_HINF_CTRL_LV_POL		BIT(7)
#define AP1302_PREVIEW_HINF_CTRL_FV_POL		BIT(6)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_CONT_CLK	BIT(5)
#define AP1302_PREVIEW_HINF_CTRL_SPOOF		BIT(4)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_MODE	BIT(3)
#define AP1302_PREVIEW_HINF_CTRL_MIPI_LANES(n)	((n) << 0)
#define AP1302_AE_MANUAL_GAIN		AP1302_REG_16BIT(0x5006)
#define AP1302_AE_BV_OFF			AP1302_REG_16BIT(0x5014)
#define AP1302_AE_MET				AP1302_REG_16BIT(0x503E)
#define AP1302_AWB_CTRL				AP1302_REG_16BIT(0x5100)
#define AP1302_AWB_CTRL_RECALC			BIT(13)
#define AP1302_AWB_CTRL_POSTGAIN		BIT(12)
#define AP1302_AWB_CTRL_UNGAIN			BIT(11)
#define AP1302_AWB_CTRL_CLIP			BIT(10)
#define AP1302_AWB_CTRL_SKY			BIT(9)
#define AP1302_AWB_CTRL_FLASH			BIT(8)
#define AP1302_AWB_CTRL_FACE_OFF		(0U << 6)
#define AP1302_AWB_CTRL_FACE_IGNORE		(1U << 6)
#define AP1302_AWB_CTRL_FACE_CONSTRAINED	(2U << 6)
#define AP1302_AWB_CTRL_FACE_ONLY		(3U << 6)
#define AP1302_AWB_CTRL_IMM			BIT(5)
#define AP1302_AWB_CTRL_IMM1			BIT(4)
#define AP1302_AWB_CTRL_MODE_OFF		(0U << 0)
#define AP1302_AWB_CTRL_MODE_HORIZON		(1U << 0)
#define AP1302_AWB_CTRL_MODE_A			(2U << 0)
#define AP1302_AWB_CTRL_MODE_CWF		(3U << 0)
#define AP1302_AWB_CTRL_MODE_D50		(4U << 0)
#define AP1302_AWB_CTRL_MODE_D65		(5U << 0)
#define AP1302_AWB_CTRL_MODE_D75		(6U << 0)
#define AP1302_AWB_CTRL_MODE_MANUAL		(7U << 0)
#define AP1302_AWB_CTRL_MODE_MEASURE		(8U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO		(15U << 0)
#define AP1302_AWB_CTRL_MODE_MASK		0x000f
#define AP1302_FLICK_CTRL			AP1302_REG_16BIT(0x5440)
#define AP1302_FLICK_CTRL_FREQ(n)		((n) << 8)
#define AP1302_FLICK_CTRL_ETC_IHDR_UP		BIT(6)
#define AP1302_FLICK_CTRL_ETC_DIS		BIT(5)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define AP1302_FLICK_CTRL_FRC_EN		BIT(2)
#define AP1302_FLICK_CTRL_MODE_DISABLED		(0U << 0)
#define AP1302_FLICK_CTRL_MODE_MANUAL		(1U << 0)
#define AP1302_FLICK_CTRL_MODE_AUTO		(2U << 0)
#define AP1302_SCENE_CTRL			AP1302_REG_16BIT(0x5454)
#define AP1302_SCENE_CTRL_MODE_NORMAL		(0U << 0)
#define AP1302_SCENE_CTRL_MODE_PORTRAIT		(1U << 0)
#define AP1302_SCENE_CTRL_MODE_LANDSCAPE	(2U << 0)
#define AP1302_SCENE_CTRL_MODE_SPORT		(3U << 0)
#define AP1302_SCENE_CTRL_MODE_CLOSE_UP		(4U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT		(5U << 0)
#define AP1302_SCENE_CTRL_MODE_TWILIGHT		(6U << 0)
#define AP1302_SCENE_CTRL_MODE_BACKLIGHT	(7U << 0)
#define AP1302_SCENE_CTRL_MODE_HIGH_SENSITIVE	(8U << 0)
#define AP1302_SCENE_CTRL_MODE_NIGHT_PORTRAIT	(9U << 0)
#define AP1302_SCENE_CTRL_MODE_BEACH		(10U << 0)
#define AP1302_SCENE_CTRL_MODE_DOCUMENT		(11U << 0)
#define AP1302_SCENE_CTRL_MODE_PARTY		(12U << 0)
#define AP1302_SCENE_CTRL_MODE_FIREWORKS	(13U << 0)
#define AP1302_SCENE_CTRL_MODE_SUNSET		(14U << 0)
#define AP1302_SCENE_CTRL_MODE_AUTO		(0xffU << 0)

#define AP1302_BOOTDATA_STAGE			AP1302_REG_16BIT(0x6002)
#define AP1302_WARNING(n)			AP1302_REG_16BIT(0x6004  (n) * 2)
#define AP1302_SENSOR_SELECT			AP1302_REG_16BIT(0x600c)
#define AP1302_SENSOR_SELECT_TP_MODE(n)		((n) << 8)
#define AP1302_SENSOR_SELECT_PATTERN_ON		BIT(7)
#define AP1302_SENSOR_SELECT_MODE_3D_ON		BIT(6)
#define AP1302_SENSOR_SELECT_CLOCK		BIT(5)
#define AP1302_SENSOR_SELECT_SINF_MIPI		BIT(4)
#define AP1302_SENSOR_SELECT_YUV		BIT(2)
#define AP1302_SENSOR_SELECT_SENSOR_TP		(0U << 0)
#define AP1302_SENSOR_SELECT_SENSOR(n)		(((n)  1) << 0)
#define AP1302_SYS_START			AP1302_REG_16BIT(0x601a)
#define AP1302_SYS_START_PLL_LOCK		BIT(15)
#define AP1302_SYS_START_LOAD_OTP		BIT(12)
#define AP1302_SYS_START_RESTART_ERROR		BIT(11)
#define AP1302_SYS_START_STALL_STATUS		BIT(9)
#define AP1302_SYS_START_STALL_EN		BIT(8)
#define AP1302_SYS_START_STALL_MODE_FRAME	(0U << 6)
#define AP1302_SYS_START_STALL_MODE_DISABLED	(1U << 6)
#define AP1302_SYS_START_STALL_MODE_POWER_DOWN	(2U << 6)
#define AP1302_SYS_START_GO			BIT(4)
#define AP1302_SYS_START_PATCH_FUN		BIT(1)
#define AP1302_SYS_START_PLL_INIT		BIT(0)
#define AP1302_DMA_SRC				AP1302_REG_32BIT(0x60a0)
#define AP1302_DMA_DST				AP1302_REG_32BIT(0x60a4)
#define AP1302_DMA_SIP_SIPM(n)			((n) << 26)
#define AP1302_DMA_SIP_DATA_16_BIT		BIT(25)
#define AP1302_DMA_SIP_ADDR_16_BIT		BIT(24)
#define AP1302_DMA_SIP_ID(n)			((n) << 17)
#define AP1302_DMA_SIP_REG(n)			((n) << 0)
#define AP1302_DMA_SIZE				AP1302_REG_32BIT(0x60a8)
#define AP1302_DMA_CTRL				AP1302_REG_16BIT(0x60ac)
#define AP1302_DMA_CTRL_SCH_NORMAL		(0 << 12)
#define AP1302_DMA_CTRL_SCH_NEXT		(1 << 12)
#define AP1302_DMA_CTRL_SCH_NOW			(2 << 12)
#define AP1302_DMA_CTRL_DST_REG			(0 << 8)
#define AP1302_DMA_CTRL_DST_SRAM		(1 << 8)
#define AP1302_DMA_CTRL_DST_SPI			(2 << 8)
#define AP1302_DMA_CTRL_DST_SIP			(3 << 8)
#define AP1302_DMA_CTRL_SRC_REG			(0 << 4)
#define AP1302_DMA_CTRL_SRC_SRAM		(1 << 4)
#define AP1302_DMA_CTRL_SRC_SPI			(2 << 4)
#define AP1302_DMA_CTRL_SRC_SIP			(3 << 4)
#define AP1302_DMA_CTRL_MODE_32_BIT		BIT(3)
#define AP1302_DMA_CTRL_MODE_MASK		(7 << 0)
#define AP1302_DMA_CTRL_MODE_IDLE		(0 << 0)
#define AP1302_DMA_CTRL_MODE_SET		(1 << 0)
#define AP1302_DMA_CTRL_MODE_COPY		(2 << 0)
#define AP1302_DMA_CTRL_MODE_MAP		(3 << 0)
#define AP1302_DMA_CTRL_MODE_UNPACK		(4 << 0)
#define AP1302_DMA_CTRL_MODE_OTP_READ		(5 << 0)
#define AP1302_DMA_CTRL_MODE_SIP_PROBE		(6 << 0)

#define AP1302_BRIGHTNESS			AP1302_REG_16BIT(0x7000)
#define AP1302_CONTRAST				AP1302_REG_16BIT(0x7002)
#define AP1302_SATURATION			AP1302_REG_16BIT(0x7006)
#define AP1302_GAMMA				AP1302_REG_16BIT(0x700A)

/* Misc Registers */
#define AP1302_REG_ADV_START			0xe000
#define AP1302_ADVANCED_BASE			AP1302_REG_32BIT(0xf038)
#define AP1302_SIP_CRC				AP1302_REG_16BIT(0xf052)

/* Advanced System Registers */
#define AP1302_ADV_IRQ_SYS_INTE			AP1302_REG_32BIT(0x00230000)
#define AP1302_ADV_IRQ_SYS_INTE_TEST_COUNT	BIT(25)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_1		BIT(24)
#define AP1302_ADV_IRQ_SYS_INTE_HINF_0		BIT(23)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI_L	(7U << 20)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_B_MIPI	BIT(19)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI_L	(15U << 14)
#define AP1302_ADV_IRQ_SYS_INTE_SINF_A_MIPI	BIT(13)
#define AP1302_ADV_IRQ_SYS_INTE_SINF		BIT(12)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_S		BIT(11)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_B		BIT(10)
#define AP1302_ADV_IRQ_SYS_INTE_IPIPE_A		BIT(9)
#define AP1302_ADV_IRQ_SYS_INTE_IP		BIT(8)
#define AP1302_ADV_IRQ_SYS_INTE_TIMER		BIT(7)
#define AP1302_ADV_IRQ_SYS_INTE_SIPM		(3U << 6)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_ADR_RANGE	BIT(5)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_DIRECT_WRITE	BIT(4)
#define AP1302_ADV_IRQ_SYS_INTE_SIPS_FIFO_WRITE	BIT(3)
#define AP1302_ADV_IRQ_SYS_INTE_SPI		BIT(2)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_CNT	BIT(1)
#define AP1302_ADV_IRQ_SYS_INTE_GPIO_PIN	BIT(0)

/* Advanced Slave MIPI Registers */
#define AP1302_ADV_SINF_MIPI_INTERNAL_p_LANE_n_STAT(p, n) \
	AP1302_REG_32BIT(0x00420008  (p) * 0x50000  (n) * 0x20)
#define AP1302_LANE_ERR_LP_VAL(n)		(((n) >> 30) & 3)
#define AP1302_LANE_ERR_STATE(n)		(((n) >> 24) & 0xf)
#define AP1302_LANE_ERR				BIT(18)
#define AP1302_LANE_ABORT			BIT(17)
#define AP1302_LANE_LP_VAL(n)			(((n) >> 6) & 3)
#define AP1302_LANE_STATE(n)			((n) & 0xf)
#define AP1302_LANE_STATE_STOP_S		0x0
#define AP1302_LANE_STATE_HS_REQ_S		0x1
#define AP1302_LANE_STATE_LP_REQ_S		0x2
#define AP1302_LANE_STATE_HS_S			0x3
#define AP1302_LANE_STATE_LP_S			0x4
#define AP1302_LANE_STATE_ESC_REQ_S		0x5
#define AP1302_LANE_STATE_TURN_REQ_S		0x6
#define AP1302_LANE_STATE_ESC_S			0x7
#define AP1302_LANE_STATE_ESC_0			0x8
#define AP1302_LANE_STATE_ESC_1			0x9
#define AP1302_LANE_STATE_TURN_S		0xa
#define AP1302_LANE_STATE_TURN_MARK		0xb
#define AP1302_LANE_STATE_ERROR_S		0xc

#define AP1302_ADV_CAPTURE_A_FV_CNT		AP1302_REG_32BIT(0x00490040)

struct sensor {
	struct v4l2_subdev v4l2_subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct i2c_client *i2c_client;
	struct otp_flash *otp_flash_instance;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *host_power_gpio;
	struct gpio_desc *device_power_gpio;
	struct gpio_desc *standby_gpio;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure_ctrl;
	struct v4l2_ctrl *scene_ctrl;
	struct v4l2_ctrl *expo_met_ctrl;
	struct v4l2_ctrl *current_scene_mode ;
	u8 selected_mode;
	u8 selected_sensor;
	bool supports_over_4k_res;
	char *sensor_name;
};


//static int ap1302_set_wb_mode(struct sensor *instance, s32 mode);
//sstatic int ap1302_set_scene_mode(struct sensor *instance, s32 val);
static int sensor_standby(struct i2c_client *client, int enable);

static int sensor_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u8 size)
{
	struct i2c_msg msg[2];
	u8 buf[2];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = size;

	return i2c_transfer(client->adapter, msg, 2);
}



static int sensor_i2c_read_16b(struct i2c_client *client, u16 reg, u16 *value)
{
	u8 v[2] = {0,0};
	int ret;

	ret = sensor_i2c_read(client, reg, v, 2);

	if (unlikely(ret < 0)) {
		dev_err(&client->dev, "i2c transfer error.\n");
		return ret;
	}

	*value = (v[0] << 8) | v[1];
	dev_dbg(&client->dev, "%s() read reg 0x%x, value 0x%x\n",
		 __func__, reg, *value);

	return 0;
}

static int sensor_i2c_write_16b(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int retry_tmp = 0;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);


	while((i2c_transfer(client->adapter, &msg, 1)) < 0)
	{
		retry_tmp++;
		dev_err(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write 16b reg:%x val:%x.\n", reg, val);

		if (retry_tmp > 50)
		{
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}

static int ap1302_read(struct sensor *instance, u16 reg_addr, u32 *val)
{
	u16 reg_value;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, reg_addr, &reg_value);
	if (ret < 0) {
		printk(KERN_ERR "Error reading from register 0x%x: %d\n", reg_addr, ret);
		return ret;
	}

	*val = reg_value;

	// Print the read value for debugging.
	printk(KERN_INFO "Read register 0x%x, value: 0x%x\n", reg_addr, *val);

	return 0; // Return 0 to indicate success. Replace this with appropriate error handling if needed.
}

static int ap1302_write(struct sensor *instance, u16 reg_addr, u32 val)
{
	int ret;

	// Print the value to be written for debugging.
	printk(KERN_INFO "Write to register 0x%x, value: 0x%x\n", reg_addr, val);

	ret = sensor_i2c_write_16b(instance->i2c_client, reg_addr, val);
	if (ret < 0) {
		printk(KERN_ERR "Error writing to register 0x%x: %d\n", reg_addr, ret);
		return ret;
	}

	return 0; // Return 0 to indicate success. Replace this with appropriate error handling if needed.
}

static u16 ap1302_wb_values[] = {
	AP1302_AWB_CTRL_MODE_OFF,	/* V4L2_WHITE_BALANCE_MANUAL */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_AUTO */
	AP1302_AWB_CTRL_MODE_A,		/* V4L2_WHITE_BALANCE_INCANDESCENT */
	AP1302_AWB_CTRL_MODE_D50,	/* V4L2_WHITE_BALANCE_FLUORESCENT */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_FLUORESCENT_H */
	AP1302_AWB_CTRL_MODE_HORIZON,	/* V4L2_WHITE_BALANCE_HORIZON */
	AP1302_AWB_CTRL_MODE_D65,	/* V4L2_WHITE_BALANCE_DAYLIGHT */
	AP1302_AWB_CTRL_MODE_AUTO,	/* V4L2_WHITE_BALANCE_FLASH */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_CLOUDY */
	AP1302_AWB_CTRL_MODE_D75,	/* V4L2_WHITE_BALANCE_SHADE */
};

static int ap1302_set_wb_mode(struct sensor *instance, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(instance, AP1302_AWB_CTRL, &val);
	if (ret)
		return ret;
	val &= ~AP1302_AWB_CTRL_MODE_MASK;
	val |= ap1302_wb_values[mode];

	if (mode == V4L2_WHITE_BALANCE_FLASH)
		val |= AP1302_AWB_CTRL_FLASH;
	else
		val &= ~AP1302_AWB_CTRL_FLASH;

	return ap1302_write(instance, AP1302_AWB_CTRL, val);
}


static int ap1302_set_exposure(struct sensor *instance, s32 mode)
{
	u32 val;
	int ret;

	ret = ap1302_read(instance, AP1302_AE_CTRL, &val);
	if (ret) {
		printk(KERN_ERR "Error reading AE_CTRL register: %d\n", ret);
		return ret;
	}

	// Print the original value of the AE_CTRL register
	printk(KERN_INFO "Original AE_CTRL register value: 0x%x\n", val);

	val &= ~AP1302_AE_CTRL_MODE_MASK;
	val |= mode;

	// Print the new value to be written to the AE_CTRL register
	printk(KERN_INFO "New AE_CTRL register value: 0x%x\n", val);

	ret = ap1302_write(instance, AP1302_AE_CTRL, val);
	if (ret) {
		printk(KERN_ERR "Error writing AE_CTRL register: %d\n", ret);
		return ret;
	}

	// Print success message if the write is successful
	printk(KERN_INFO "Exposure set successfully.\n");

	return 0;
}

static int ap1302_set_exp_met(struct sensor *instance, s32 val)
{


printk(KERN_INFO "Entering exposure metering to \n");
	return ap1302_write(instance, AP1302_AE_MET, val);
}



//#include <linux/kernel.h>

static u16 ap1302_scene_mode_values[] = {	
  AP1302_SCENE_CTRL_MODE_NORMAL,	
  AP1302_SCENE_CTRL_MODE_PORTRAIT,		
  AP1302_SCENE_CTRL_MODE_LANDSCAPE,
  AP1302_SCENE_CTRL_MODE_SPORT	,	
  AP1302_SCENE_CTRL_MODE_CLOSE_UP,	
  AP1302_SCENE_CTRL_MODE_NIGHT	,	
  AP1302_SCENE_CTRL_MODE_TWILIGHT,	
  AP1302_SCENE_CTRL_MODE_BACKLIGHT,	
  AP1302_SCENE_CTRL_MODE_HIGH_SENSITIVE,
  AP1302_SCENE_CTRL_MODE_NIGHT_PORTRAIT,
  AP1302_SCENE_CTRL_MODE_BEACH,		
  AP1302_SCENE_CTRL_MODE_DOCUMENT,		
  AP1302_SCENE_CTRL_MODE_PARTY,		
  AP1302_SCENE_CTRL_MODE_FIREWORKS,	
  AP1302_SCENE_CTRL_MODE_SUNSET,		
  AP1302_SCENE_CTRL_MODE_AUTO,	
};

static int ap1302_set_scene_mode(struct sensor *instance, s32 val)
{
    // Define the register address for the scene control mode
    u16 reg_addr = AP1302_SCENE_CTRL;

    // Check if the provided value (val) is within the valid range of scene modes
    if (val < 0 || val >= ARRAY_SIZE(ap1302_scene_mode_values)) {
        printk(KERN_ERR "Invalid scene mode value: %d\n", val);
        return -EINVAL; // Return an error code to indicate an invalid value
    }

    // Write the scene mode value to the AP1302 camera sensor
    int ret = ap1302_write(instance, reg_addr, ap1302_scene_mode_values[val]);
    if (ret < 0) {
        printk(KERN_ERR "Failed to set scene mode: %d\n", ret);
        return ret; // Return an error code if the write operation fails
    }

    // If needed, update the current scene mode value in the instance structure
    instance->current_scene_mode = val;

    // Print a debug message indicating the successful setting of the scene mode
    printk(KERN_INFO "Scene mode set to: %d\n", val);

    return 0; // Return 0 to indicate success
}

#define SCENE_MODE_PORTRAIT AP1302_SCENE_CTRL_MODE_PORTRAIT
#define SCENE_MODE_LANDSCAPE AP1302_SCENE_CTRL_MODE_LANDSCAPE

// Function to set portrait mode
int set_portrait_mode(struct sensor *instance) {
    return ap1302_set_scene_mode(instance, SCENE_MODE_PORTRAIT);
}

// Function to set landscape mode
int set_landscape_mode(struct sensor *instance) {
    return ap1302_set_scene_mode(instance, SCENE_MODE_LANDSCAPE);
}

static int ap1302_set_gain(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_AE_MANUAL_GAIN, val);
}

static int ap1302_set_contrast(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_CONTRAST, val);
}

static int ap1302_set_brightness(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_BRIGHTNESS, val);
}

static int ap1302_set_saturation(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_SATURATION, val);
}

static int ap1302_set_gamma(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_GAMMA, val);
}

static int ap1302_set_zoom(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_DZ_TGT_FCT, val);
}

static u16 ap1302_sfx_values[] = {
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_NONE */
	AP1302_SFX_MODE_SFX_BW,		/* V4L2_COLORFX_BW */
	AP1302_SFX_MODE_SFX_SEPIA1,	/* V4L2_COLORFX_SEPIA */
	AP1302_SFX_MODE_SFX_NEGATIVE,	/* V4L2_COLORFX_NEGATIVE */
	AP1302_SFX_MODE_SFX_EMBOSS,	/* V4L2_COLORFX_EMBOSS */
	AP1302_SFX_MODE_SFX_SKETCH,	/* V4L2_COLORFX_SKETCH */
	AP1302_SFX_MODE_SFX_BLUISH,	/* V4L2_COLORFX_SKY_BLUE */
	AP1302_SFX_MODE_SFX_GREENISH,	/* V4L2_COLORFX_GRASS_GREEN */
	AP1302_SFX_MODE_SFX_REDISH,	/* V4L2_COLORFX_SKIN_WHITEN */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_VIVID */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_AQUA */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_ART_FREEZE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SILHOUETTE */
	AP1302_SFX_MODE_SFX_SOLARIZE, /* V4L2_COLORFX_SOLARIZATION */
	AP1302_SFX_MODE_SFX_ANTIQUE, /* V4L2_COLORFX_ANTIQUE */
	AP1302_SFX_MODE_SFX_NORMAL,	/* V4L2_COLORFX_SET_CBCR */
};

static int ap1302_set_special_effect(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_SFX_MODE, ap1302_sfx_values[val]);
}

static const u16 ap1302_flicker_values[] = {
	AP1302_FLICK_CTRL_MODE_DISABLED,
	AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL,
	AP1302_FLICK_CTRL_MODE_AUTO,
};


static int ap1302_set_flicker_freq(struct sensor *instance, s32 val)
{
	return ap1302_write(instance, AP1302_FLICK_CTRL,
			    ap1302_flicker_values[val]);
}


static int ap1302_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor *instance = container_of(ctrl->handler, struct sensor, ctrls);

switch (ctrl->id) {
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return ap1302_set_wb_mode(instance, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ap1302_set_exposure(instance, ctrl->val);

	case V4L2_CID_EXPOSURE_METERING:
		return ap1302_set_exp_met(instance, ctrl->val);

	case V4L2_CID_GAIN:
		return ap1302_set_gain(instance, ctrl->val);

	case V4L2_CID_GAMMA:
		return ap1302_set_gamma(instance, ctrl->val);

	case V4L2_CID_CONTRAST:
		return ap1302_set_contrast(instance, ctrl->val);

	case V4L2_CID_BRIGHTNESS:
		return ap1302_set_brightness(instance, ctrl->val);

	case V4L2_CID_SATURATION:
		return ap1302_set_saturation(instance, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ap1302_set_zoom(instance, ctrl->val);

	case V4L2_CID_COLORFX:
		return ap1302_set_special_effect(instance, ctrl->val);

        case V4L2_CID_SCENE_MODE:
            // Check the value of the V4L2_CID_SCENE_MODE control
            switch (ctrl->val) {
                case SCENE_MODE_PORTRAIT:
                    return set_portrait_mode(instance);

                case SCENE_MODE_LANDSCAPE:
                    return set_landscape_mode(instance);

                default:
                    return ap1302_set_scene_mode(instance, ctrl->val);
            }
	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ap1302_set_flicker_freq(instance, ctrl->val);

	default:
		return -EINVAL;
	}
}


static const struct v4l2_ctrl_ops ap1302_ctrl_ops = {
	.s_ctrl = ap1302_s_ctrl,
};


static const struct v4l2_ctrl_config ap1302_ctrls[] = {
	{
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.min = 0,
		.max = 9,
		.def = 1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0100,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x1000,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x100,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x100,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0100,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x1000,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xC,
		.step = 1,
		.def = 0xC,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Exposure Metering",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x3,
		.step = 1,
		.def = 0x1,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0100,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.min = 0x0100,
		.max = 0x1000,
		.step = 1,
		.def = 0x0100,
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.min = 0,
		.max = 15,
		.def = 0,
		.menu_skip_mask = BIT(15) | BIT(12) | BIT(11) | BIT(10) | BIT(9),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_SCENE_MODE,
		.min = 0,
		.max = 14,
		.def = 0,
	//	.menu_skip_mask = BIT(5) | BIT(4),
	}, {
		.ops = &ap1302_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.min = 0,
		.max = 3,
		.def = 3,
	},
};

static int sensor_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len)
{
	struct i2c_msg msg;
	int retry_tmp = 0;

	if (len == 0) {
		return 0;
	}

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = len;

	while((i2c_transfer(client->adapter, &msg, 1)) < 0)
	{
		retry_tmp++;
		dev_err(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write bust buf:%x.\n", client->addr);

		if (retry_tmp > 50)
		{
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}

static int ops_power(struct v4l2_subdev *sub_dev, int on)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, on);
	return 0;
}

static int ops_init(struct v4l2_subdev *sub_dev, u32 val)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, val);
	return 0;
}

static int ops_load_fw(struct v4l2_subdev *sub_dev)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s()\n", __func__);
	return 0;
}

static int ops_reset(struct v4l2_subdev *sub_dev, u32 val)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, val);
	return 0;
}

static int ops_get_frame_interval(struct v4l2_subdev *sub_dev,
				  struct v4l2_subdev_frame_interval *fi)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int ops_set_frame_interval(struct v4l2_subdev *sub_dev,
				  struct v4l2_subdev_frame_interval *fi)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int ops_set_stream(struct v4l2_subdev *sub_dev, int enable)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int ret = 0;

	dev_dbg(sub_dev->dev, "%s() enable [%x]\n", __func__, enable);

	if (instance->selected_mode >= ap1302_sensor_table[instance->selected_sensor].res_list_size)
		return -EINVAL;

	if (enable == 0) {
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
		//VIDEO_WIDTH
		sensor_i2c_write_16b(instance->i2c_client, 0x2000, 1280);//1280
		//VIDEO_HEIGHT
		sensor_i2c_write_16b(instance->i2c_client, 0x2002, 720);//720
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
		ret = sensor_standby(instance->i2c_client, 1);
	} else {
		ret = sensor_standby(instance->i2c_client, 0);
		if (ret == 0) {
			int fps = ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].framerates;
			dev_dbg(sub_dev->dev, "%s() width=%d, height=%d\n", __func__, 
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width, 
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			pr_err("FPS_BMIT 1 %d \n",fps);
			sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
			//VIDEO_WIDTH
			sensor_i2c_write_16b(instance->i2c_client, 0x2000,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width);
			//VIDEO_HEIGHT
			pr_err("FPS_BMIT 2 %d \n",fps);
			sensor_i2c_write_16b(instance->i2c_client, 0x2002,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			sensor_i2c_write_16b(instance->i2c_client, 0x2020, fps << 8); //VIDEO_MAX_FPS
			sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
		}
	}

	return ret;
}

static int ops_enum_mbus_code(struct v4l2_subdev *sub_dev,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;

	return 0;
}

static int ops_get_fmt(struct v4l2_subdev *sub_dev,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sub_dev,
						 sd_state,
						 format->pad);
	else
		fmt = &instance->fmt;

	memmove(mbus_fmt, fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int ops_set_fmt(struct v4l2_subdev *sub_dev,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int i;

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	for(i = 0 ; i < ap1302_sensor_table[instance->selected_sensor].res_list_size ; i++)
	{
		if (mbus_fmt->width == ap1302_sensor_table[instance->selected_sensor].res_list[i].width &&
				mbus_fmt->height == ap1302_sensor_table[instance->selected_sensor].res_list[i].height)
			break;
	}

	if (i >= ap1302_sensor_table[instance->selected_sensor].res_list_size)
	{
		return -EINVAL;
	}
	instance->selected_mode = i;
	dev_dbg(sub_dev->dev, "%s() selected mode index [%d]\n", __func__,
		instance->selected_mode);

	mbus_fmt->width = ap1302_sensor_table[instance->selected_sensor].res_list[i].width;
	mbus_fmt->height = ap1302_sensor_table[instance->selected_sensor].res_list[i].height;
	mbus_fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
	mbus_fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(mbus_fmt->colorspace);
	mbus_fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mbus_fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(mbus_fmt->colorspace);
	memset(mbus_fmt->reserved, 0, sizeof(mbus_fmt->reserved));

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sub_dev, sd_state, 0);
	else
		fmt = &instance->fmt;

	memmove(fmt, mbus_fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int ops_enum_frame_size(struct v4l2_subdev *sub_dev,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	dev_dbg(sub_dev->dev, "%s() %x %x %x\n", __func__,
		fse->pad, fse->code, fse->index);

	if ((fse->pad != 0) ||
	    (fse->index >= ap1302_sensor_table[instance->selected_sensor].res_list_size))
		return -EINVAL;

	if(!instance->supports_over_4k_res &&
	    ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].width > 4096)
		return -EINVAL;

	fse->min_width = fse->max_width = ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].width;
	fse->min_height = fse->max_height = ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].height;

	return 0;
}

static int ops_enum_frame_interval(struct v4l2_subdev *sub_dev,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int i;
	dev_dbg(sub_dev->dev, "%s() %x %x %x\n", __func__,
				fie->pad, fie->code, fie->index);

	if ((fie->pad != 0) ||
	    (fie->index != 0))
		return -EINVAL;

	fie->interval.numerator = 1;

	for(i = 0 ; i < ap1302_sensor_table[instance->selected_sensor].res_list_size ; i++) {
		if(fie->width == ap1302_sensor_table[instance->selected_sensor].res_list[i].width &&
			fie->height == ap1302_sensor_table[instance->selected_sensor].res_list[i].height) {
				fie->interval.denominator = ap1302_sensor_table[instance->selected_sensor].res_list[i].framerates;
				break;
			}
	}

	return 0;
}

static int ops_media_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_subdev_core_ops sensor_v4l2_subdev_core_ops = {
	.s_power = ops_power,
	.init = ops_init,
	//.s_ctrl = v4l2_subdev_s_ctrl,
	.load_fw = ops_load_fw,
	.reset = ops_reset,
};
static const struct v4l2_subdev_video_ops sensor_v4l2_subdev_video_ops = {
	.g_frame_interval = ops_get_frame_interval,
	.s_frame_interval = ops_set_frame_interval,
	.s_stream = ops_set_stream,
};
static const struct v4l2_subdev_pad_ops sensor_v4l2_subdev_pad_ops = {
	.enum_mbus_code = ops_enum_mbus_code,
	.get_fmt = ops_get_fmt,
	.set_fmt = ops_set_fmt,
	.enum_frame_size = ops_enum_frame_size,
	.enum_frame_interval = ops_enum_frame_interval,
};

static const struct v4l2_subdev_ops sensor_subdev_ops = {
	.core = &sensor_v4l2_subdev_core_ops,
	.video = &sensor_v4l2_subdev_video_ops,
	.pad = &sensor_v4l2_subdev_pad_ops,
};

static const struct media_entity_operations sensor_media_entity_ops = {
	.link_setup = ops_media_link_setup,
};

static int check_sensor_chip_id(struct i2c_client *client, u16* chip_id)
{
	int timeout;

	for (timeout = 0 ; timeout < 100 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x60AC, chip_id);
		if ((*chip_id & 0x7) == 0)
			break;
	}
	if (timeout >= 100) {
		dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, *chip_id);
		return -EINVAL;
	}
	sensor_i2c_write_16b(client, 0x60AA, 0x0002); // DMA_SIZE
	sensor_i2c_write_16b(client, 0x60A0, 0x0320); // DMA_SRC_0
	sensor_i2c_write_16b(client, 0x60A2, 0x3000); // DMA_SRC_1
	sensor_i2c_write_16b(client, 0x60A4, 0x0000); // DMA_DST_0
	sensor_i2c_write_16b(client, 0x60A6, 0x60A4); // DMA_DST_1
	sensor_i2c_write_16b(client, 0x60AC, 0x0032); // DMA_CTRL
	for (timeout = 0 ; timeout < 100 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x60AC, chip_id);
		if ((*chip_id & 0x7) == 0)
			break;
	}
	if (timeout >= 100) {
		dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, *chip_id);
		return -EINVAL;
	}
	sensor_i2c_read_16b(client, 0x60A4, chip_id);

	return 0;
}

static int set_standby_mode_rel419(struct i2c_client *client, int enable)
{
	u16 v = 0;
	int timeout;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	if (enable == 1) {
		sensor_i2c_write_16b(client, 0xf056, 0x0000);
		sensor_i2c_write_16b(client, 0x601a, 0x8140);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if (timeout < 500) {
			if(check_sensor_chip_id(client, &v) == 0) {
				if (v == 0x356) {
					dev_dbg(&client->dev, "sensor check: v=0x%x\nbypass standby and set fw stall gate frames.\n", v);
					return 0;
				}
			}
			// Reset ADV_GPIO in Advanced Registers
			sensor_i2c_write_16b(client, 0xF038, 0x002A);
			sensor_i2c_write_16b(client, 0xF03A, 0x0000);
			sensor_i2c_write_16b(client, 0xE002, 0x0490);
			sensor_i2c_write_16b(client, 0xFFFE, 1);
			msleep(100);
		} else {
			dev_err(&client->dev, "timeout: line[%d]\n", __LINE__);
			return -EINVAL;
		}
	} else {
		sensor_i2c_write_16b(client, 0xFFFE, 0);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0, &v);
		  	if (v != 0)
		 	  break;
		}
		if (timeout >= 500) {
		 dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
		 return -EINVAL;
		}
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			if(check_sensor_chip_id(client, &v) == 0) {
				if (v == 0x356) {
					dev_dbg(&client->dev, "sensor check: v=0x%x\nrecover status from fw stall gate frames.\n", v);
					sensor_i2c_write_16b(client, 0x601a, 0x8340);
					msleep(10);
					break;
				}
			}
		}

		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if ( (v & 0x200) != 0x200 ) {
			dev_dbg(&client->dev, "stop waking up: camera is working.\n");
			return 0;
		}

		sensor_i2c_write_16b(client, 0x601a, 0x241);
		usleep_range(1000, 2000);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 1) == 0)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			pr_err("timout morethan rc1\n");
			return -EINVAL;
		}

		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x8000) == 0x8000)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			pr_err("timout morethan rc2\n");
			return -EINVAL;
		}

		sensor_i2c_write_16b(client, 0x601a, 0x8250);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x8040) == 0x8040)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			pr_err("timout morethan rc3\n");
			return -EINVAL;
		}
		sensor_i2c_write_16b(client, 0xF056, 0x0000);

		dev_dbg(&client->dev, "sensor wake up\n");
	}

	return 0;
}

static int sensor_standby(struct i2c_client *client, int enable)
{
	u16 v = 0;
	int timeout;
	u16 checksum = 0;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	for (timeout = 0 ; timeout < 50 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x6134, &checksum);
		if (checksum == 0xFFFF)
			break;
	}
	if(checksum != 0xFFFF){
		return set_standby_mode_rel419(client, enable); // standby for rel419
	}

	if (enable == 1) {
		sensor_i2c_write_16b(client, 0x601a, 0x0180);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if (timeout < 500) {
			msleep(100);
		} else {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			pr_err("timout morethan 500\n");
			return -EINVAL;
		}
	} else {
		sensor_i2c_write_16b(client, 0x601a, 0x0380);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			pr_err("timout morethan 100\n");
			return -EINVAL;
		}
		dev_dbg(&client->dev, "sensor wake up\n");
	}

	return 0;
}

static int sensor_power_on(struct sensor *instance)
{
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);
	//gpiod_set_value_cansleep(instance->standby_gpio, 1);
	gpiod_set_value_cansleep(instance->host_power_gpio, 1);
	gpiod_set_value_cansleep(instance->device_power_gpio, 1);
	usleep_range(500, 5000);
	gpiod_set_value_cansleep(instance->reset_gpio, 1);
	msleep(10);

	return 0;
}

static int sensor_power_off(struct sensor *instance)
{
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);
	gpiod_set_value_cansleep(instance->standby_gpio, 0);
	gpiod_set_value_cansleep(instance->reset_gpio, 0);
	usleep_range(50, 500);
	gpiod_set_value_cansleep(instance->device_power_gpio, 0);
	gpiod_set_value_cansleep(instance->host_power_gpio, 0);
	msleep(10);

	return 0;
}

static int sensor_try_on(struct sensor *instance)
{
	u16 v;
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);

	sensor_power_off(instance);

	sensor_power_on(instance);

	if (sensor_i2c_read_16b(instance->i2c_client, 0, &v) != 0) {
		dev_err(&instance->i2c_client->dev, "%s() try on failed\n",
			__func__);
		sensor_power_off(instance);
		return -EINVAL;
	}

	return 0;
}

static int sensor_load_bootdata(struct sensor *instance)
{
	struct device *dev = &instance->i2c_client->dev;
	int index = 0;
	size_t len = BOOT_DATA_WRITE_LEN;
	u16 otp_data;
	u16 *bootdata_temp_area;
	u16 checksum;

	bootdata_temp_area = devm_kzalloc(dev,
					  BOOT_DATA_WRITE_LEN + 2,
					  GFP_KERNEL);
	if (bootdata_temp_area == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -EINVAL;
	}

	checksum = ap1302_otp_flash_get_checksum(instance->otp_flash_instance);

	while(!(len < BOOT_DATA_WRITE_LEN)) {
		bootdata_temp_area[0] = cpu_to_be16(BOOT_DATA_START_REG);
		len = ap1302_otp_flash_read(instance->otp_flash_instance,
					    (u8 *)(&bootdata_temp_area[1]),
					    index, BOOT_DATA_WRITE_LEN);
		dev_dbg(dev, "index: 0x%04x, len [%zu]\n", index, len);
		sensor_i2c_write_bust(instance->i2c_client,
				      (u8 *)bootdata_temp_area,
				      len + 2);
		index += len;
	}

	sensor_i2c_write_16b(instance->i2c_client, 0x6002, 0xffff);
	devm_kfree(dev, bootdata_temp_area);

	index = 0;
	otp_data = 0;
	while(otp_data != checksum && index < 20) {
		msleep(10);
		sensor_i2c_read_16b(instance->i2c_client, 0x6134, &otp_data);
		index ++;
	}
	if (unlikely(index == 20)) {
		if (likely(otp_data == 0))
			dev_err(dev, "failed try to read checksum\n");
		else
			dev_err(dev, "bootdata checksum missmatch\n");

		return -EINVAL;
	}

	return 0;
}

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_fwnode_device_properties props;
	struct sensor *instance = NULL;
	struct device *dev = &client->dev;
	struct v4l2_mbus_framefmt *fmt;
	struct header_ver2 *header;
	int data_lanes;
	int continuous_clock;
	int i;
	int ret;
	int retry_f;

	dev_info(&client->dev, "%s() device node: %s\n",
		       __func__, client->dev.of_node->full_name);

	instance = devm_kzalloc(dev, sizeof(struct sensor), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -EINVAL;
	}
	instance->i2c_client = client;
	//instance->dev_regmap_config = &sensor_regmap_config;

	instance->host_power_gpio = devm_gpiod_get(dev, "host-power",
						   GPIOD_OUT_LOW);
	instance->device_power_gpio = devm_gpiod_get(dev, "device-power",
						     GPIOD_OUT_LOW);
	instance->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	instance->standby_gpio = devm_gpiod_get(dev, "standby", GPIOD_OUT_LOW);

	if (IS_ERR(instance->reset_gpio) ||
	    IS_ERR(instance->host_power_gpio) ||
	    IS_ERR(instance->device_power_gpio) ||
	    IS_ERR(instance->standby_gpio) ) {
		dev_err(dev, "get gpio object failed\n");
		return -EPROBE_DEFER;
	}

	data_lanes = 4;
	if (of_property_read_u32(dev->of_node, "data-lanes", &data_lanes) == 0) {
		if ((data_lanes < 1) || (data_lanes > 4)) {
			dev_err(dev, "value of 'data-lanes' property is invaild\n");
			data_lanes = 4;
		}
	}

	continuous_clock = 0;
	if (of_property_read_u32(dev->of_node, "continuous-clock",
				 &continuous_clock) == 0) {
		if (continuous_clock > 1) {
			dev_err(dev, "value of 'continuous-clock' property is invaild\n");
			continuous_clock = 0;
		}
	}

	instance->supports_over_4k_res = of_property_read_bool(dev->of_node, "supports-over-4k-res");

	dev_dbg(dev, "data-lanes [%d] ,continuous-clock [%d], supports-over-4k-res [%d]\n",
		data_lanes, continuous_clock, instance->supports_over_4k_res);

	retry_f = 0x01;
	/*
	bit 0: start bit
	bit 1: sensor_try_on fail
	bit 2: ap1302_otp_flash_init fail
	bit 3: sensor_load_bootdata fail
	bit 4-7: retry count
	*/
	while(retry_f) {
		retry_f &= ~0x01;

		if (sensor_try_on(instance) != 0) {
			retry_f |= 0x02 ;
		}

		instance->otp_flash_instance = ap1302_otp_flash_init(dev);
		if(IS_ERR(instance->otp_flash_instance)) {
			dev_err(dev, "otp flash init failed\n");
			// retry_f |= 0x04 ;
			return -EINVAL;
		}

		header = instance->otp_flash_instance->header_data;
		for(i = 0 ; i < ARRAY_SIZE(ap1302_sensor_table); i++)
		{
			if (strcmp((const char*)header->product_name, ap1302_sensor_table[i].sensor_name) == 0)
				break;
		}
		instance->selected_sensor = i;
		dev_dbg(dev, "selected_sensor:%d, sensor_name:%s\n", i, header->product_name);

		if(sensor_load_bootdata(instance) != 0) {
			dev_err(dev, "load bootdata failed\n");
			retry_f |= 0x08 ;
		}

		if ((retry_f & 0x0F) != 0x00) {
			if (((retry_f & 0x30) >> 4 ) < 3 ) {
				retry_f += 1 << 4;
				retry_f &= ~0x0F;
				dev_err(dev, "Probe retry:%d.\n", ((retry_f & 0x30) >> 4 ));
			}
			else {
				retry_f &= 0x00;
				dev_dbg(dev, "Probe retry failed\n");
				return  -EINVAL;
			}
		}
	}

	fmt = &instance->fmt;
	fmt->width = ap1302_sensor_table[instance->selected_sensor].res_list[0].width;
	fmt->height = ap1302_sensor_table[instance->selected_sensor].res_list[0].height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	pr_err("media_bus inside\n");
	fmt->colorspace =  V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	memset(fmt->reserved, 0, sizeof(fmt->reserved));
	
	//ret = v4l2_fwnode_device_parse(&instance->i2c_client->dev, &props);
	//ret = v4l2_ctrl_new_fwnode_properties(&instance->ctrls, &ap1302_ctrl_ops,
	//				      &props);
					      

	v4l2_i2c_subdev_init(&instance->v4l2_subdev,client, &sensor_subdev_ops);
	instance->v4l2_subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	instance->v4l2_subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	instance->pad.flags= MEDIA_PAD_FL_SOURCE;
	
	
	
	
	instance->v4l2_subdev.entity.ops = &sensor_media_entity_ops;
	
	ret = v4l2_ctrl_handler_init(&instance->ctrls, ARRAY_SIZE(ap1302_ctrls));
for (i = 0; i < ARRAY_SIZE(ap1302_ctrls); i++) {
    struct v4l2_ctrl *ctrl;

    if (ap1302_ctrls[i].id == V4L2_CID_SCENE_MODE) {
        // Handle V4L2_CID_SCENE_MODE separately
        // Here, you can either set a default scene mode or ignore this control
        // based on your camera sensor's behavior.
        // For example, you can set the default scene mode to Landscape:
        ctrl = v4l2_ctrl_new_custom(&instance->ctrls, &ap1302_ctrls[i], NULL);
        if (ctrl == NULL) {
            dev_err(&instance->i2c_client->dev, "v4l2_ctrl_new_custom failed\n");
            return -ENOMEM; // or appropriate error code
        }
        // Set the default scene mode, e.g., Landscape
        ctrl->val = V4L2_SCENE_MODE_LANDSCAPE; // Assuming V4L2_SCENE_MODE_LANDSCAPE is defined in your driver
        ap1302_set_scene_mode(instance, ctrl->val);
    } else {
        // Handle other controls using v4l2_ctrl_new_custom as before
        ctrl = v4l2_ctrl_new_custom(&instance->ctrls, &ap1302_ctrls[i], NULL);
        if (ctrl == NULL) {
            dev_err(&instance->i2c_client->dev, "v4l2_ctrl_new_custom failed\n");
            return -ENOMEM; // or appropriate error code
        }
    }
}

                                  
                                  
	instance->v4l2_subdev.ctrl_handler = &instance->ctrls;
	
	v4l2_ctrl_handler_setup(&instance->ctrls);	
	
	ret = media_entity_pads_init(&instance->v4l2_subdev.entity, 1, &instance->pad);
	
	ret += v4l2_async_register_subdev(&instance->v4l2_subdev);
	if (ret != 0) {
		dev_err(&instance->i2c_client->dev, "v4l2 register failed\n");
		return -EINVAL;
	}

	//set something reference from DevX tool register log
	//cntx select 'Video'
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
	sensor_i2c_write_16b(instance->i2c_client, 0x1000, 0); //CTRL
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
	msleep(1);
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
	//Video output
	sensor_i2c_write_16b(instance->i2c_client, 0x2000, ap1302_sensor_table[instance->selected_sensor].res_list[0].width); //VIDEO_WIDTH
	sensor_i2c_write_16b(instance->i2c_client, 0x2002, ap1302_sensor_table[instance->selected_sensor].res_list[0].height); //VIDEO_HEIGHT
	sensor_i2c_write_16b(instance->i2c_client, 0x2012, 0x50); //VIDEO_OUT_FMT
	//continuous clock, data-lanes
	sensor_i2c_write_16b(instance->i2c_client, 0x2030,
			     0x10 | (continuous_clock << 5) | (data_lanes)); //VIDEO_HINF_CTRL
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC

	//let ap1302 go to standby mode
	ret = sensor_standby(instance->i2c_client, 1);
	pr_err("ret in probe func %d \n",ret);
	if (ret == 0)
		dev_info(&client->dev, "probe success\n");
	else
		dev_err(&client->dev, "probe failed\n");

	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "tevi-ap1302", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static const struct of_device_id sensor_of[] = {
	{ .compatible = "tn,tevi-ap1302" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sensor_of);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(sensor_of),
		.name  = "tevi-ap1302",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

module_i2c_driver(sensor_i2c_driver);

MODULE_AUTHOR("TECHNEXION Inc.");
MODULE_DESCRIPTION("TechNexion driver for TEVI-AR Series");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("Camera");
