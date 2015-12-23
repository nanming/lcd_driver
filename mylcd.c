
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <plat/clock.h>
#include <plat/media.h>
#include <mach/media.h>
#include <mach/map.h>
#include "mylcd.h"
#include <linux/regulator/driver.h>		

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-fb-s5p.h>


static unsigned int pseudo_pal[16];
static struct mylcd_global *mylcd_global;
static struct my_lcd_timing my_lcd_timing= {
    .h_spw = 32,
    .h_bpd = 80,
    .h_fpd = 48,
    .v_spw = 5,
    .v_bpd = 14,
    .v_fpd = 3,
    .rise_vclk = 1,
    .i_hsync = 0,
    .i_vsync = 1,
    .i_vden  = 0,
};

static struct lcd_desc lcd_desc = {
    .height = 800,
    .length = 1280,
    .bpp    = 24,
};


#define EXYNOS4_GPD_0_0_TOUT_0  (0x2)
#define EXYNOS4_GPD_0_1_TOUT_1  (0x2 << 4)
#define EXYNOS4_GPD_0_2_TOUT_2  (0x2 << 8)
#define EXYNOS4_GPD_0_3_TOUT_3  (0x2 << 12)


static inline unsigned int __chan_to_field(unsigned int chan, struct fb_bitfield bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf.length;

	return chan << bf.offset;
}

static int myfb_setcolreg(unsigned int regno, unsigned int red,
			unsigned int green, unsigned int blue,
			unsigned int transp, struct fb_info *fb)
{
	unsigned int *pal = (unsigned int *)fb->pseudo_palette;
	unsigned int val = 0;

	if (regno < 16) {
		/* fake palette of 16 colors */
		val |= __chan_to_field(red, fb->var.red);
		val |= __chan_to_field(green, fb->var.green);
		val |= __chan_to_field(blue, fb->var.blue);
		val |= __chan_to_field(transp, fb->var.transp);
		pal[regno] = val;
	}

	return 0;
}

struct fb_ops mylcd_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= myfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};


static int mylcd_set_fbinfo(struct mylcd_global *mylcd_global)
{
    struct fb_info *fb = mylcd_global->fb_info;
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct fb_var_screeninfo *var = &fb->var;
    struct lcd_desc *lcd_desc = mylcd_global->lcd_desc;
    int height = lcd_desc->height;
    int length = lcd_desc->height;
    int bpp    = lcd_desc->bpp;

	strcpy(fix->id, "mylcd_4412");
    fix->smem_len    = height * length * bpp / 8; 
    fix->type        = FB_TYPE_PACKED_PIXELS;
    fix->visual      = FB_VISUAL_TRUECOLOR;
    fix->line_length = length * bpp / 8;

    var->xres        = length;
    var->yres        = height;
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres;
    var->xoffset      = 0;
    var->yoffset      = 0;
    var->bits_per_pixel = bpp;

	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
    var->transp.offset = 0;
    var->transp.length = 0; /* added for LCD RGB32 */

    var->activate = FB_ACTIVATE_NOW;

    fb->fbops = &mylcd_fb_ops;
    fb->pseudo_palette = &pseudo_pal;


	fb->screen_size =  height * length * bpp / 8;
	fb->screen_base = dma_alloc_writecombine(NULL,
						 PAGE_ALIGN(fix->smem_len),
						 (unsigned int *)
						 &fix->smem_start, GFP_KERNEL);

    return 0;
}

static void myfb_gpio_setup_24bpp(unsigned int start, unsigned int size,
		unsigned int cfg, s5p_gpio_drvstr_t drvstr)
{
	u32 reg;

	s3c_gpio_cfgrange_nopull(start, size, cfg);

	for (; size > 0; size--, start++)
		s5p_gpio_set_drvstr(start, drvstr);

	/* Set FIMD0 bypass */
	reg = __raw_readl(S3C_VA_SYS + 0x0210);
	reg |= (1<<1);
	__raw_writel(reg, S3C_VA_SYS + 0x0210);
}

static int mylcd_bakclight_on(void)
{
    int err;
    /* LCD Backlight control*/ 
    err = gpio_request(EXYNOS4_GPL0(4), "BK_VDD_EN");
    if (err) {
            printk(KERN_ERR "failed to request BK_VDD_EN\n");
            //return err;
    }

    gpio_direction_output(EXYNOS4_GPL0(4), 1);
    s3c_gpio_cfgpin(EXYNOS4_GPL0(4), S3C_GPIO_OUTPUT);
    gpio_free(EXYNOS4_GPL0(4));

    mdelay(5);
    /* LCD Power*/ 
	err = gpio_request(EXYNOS4_GPL1(0), "GPL1_0");
	if (err) {
		printk(KERN_ERR "failed to request GPL1 for "
			"lcd power control\n");
		return err;
	}
	gpio_direction_output(EXYNOS4_GPL1(0), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPL1(0), S3C_GPIO_OUTPUT);
	gpio_free(EXYNOS4_GPL1(0));

   	mdelay(250);
	//PWM
	err = gpio_request(EXYNOS4_GPD0(1), "GPD0_1");
	if (err) {
		printk(KERN_ERR "failed to request GPD0 for "
			"lcd backlight control\n");
		//return err;
	}
#if !defined(CONFIG_BACKLIGHT_PWM)
	gpio_direction_output(EXYNOS4_GPD0(1), 1);
	gpio_free(EXYNOS4_GPD0(1));
#else
	gpio_direction_output(EXYNOS4_GPD0(1), 0);

	s3c_gpio_cfgpin(EXYNOS4_GPD0(1), EXYNOS4_GPD_0_1_TOUT_1);

	gpio_free(EXYNOS4_GPD0(1));
	printk("(%s, %d): LCD_PWM_ON\n", __FUNCTION__, __LINE__);
#endif
	mdelay(5);

#if 1
	err = gpio_request_one(EXYNOS4_GPC0(2), 0, "GPC0_2");
        if (err) {
                printk(KERN_ERR "failed to request GPC0_2 for "
                                "4.3 LCD control\n");
                return err;
        }

        s3c_gpio_setpull(EXYNOS4_GPC0(2), S3C_GPIO_PULL_UP);
        //gpio_set_value(EXYNOS4_GPC0(2), 0);

        //mdelay(10);

        gpio_set_value(EXYNOS4_GPC0(2), 1);

        gpio_free(EXYNOS4_GPC0(2));
#endif		

	err = gpio_request(EXYNOS4_GPC0(2), "VGA_EN");
        if (err) {
                printk(KERN_ERR "failed to request VGA_EN\n");
                return err;
        }

	gpio_direction_output(EXYNOS4_GPC0(2), 1);

	s3c_gpio_cfgpin(EXYNOS4_GPC0(2), S3C_GPIO_OUTPUT);
        gpio_free(EXYNOS4_GPC0(2));

	printk("(%s, %d): VGA_EN_ON\n", __FUNCTION__, __LINE__);
	/* end add */
    return 0;
}

static int mylcd_cfg_gpio(void)
{
	myfb_gpio_setup_24bpp(EXYNOS4_GPF0(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	myfb_gpio_setup_24bpp(EXYNOS4_GPF1(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	myfb_gpio_setup_24bpp(EXYNOS4_GPF2(0), 8, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);
	myfb_gpio_setup_24bpp(EXYNOS4_GPF3(0), 4, S3C_GPIO_SFN(2), S5P_GPIO_DRVSTR_LV4);

    mylcd_bakclight_on();

    return 0;
}

static void mylcd_cfg_timing(struct mylcd_global *mylcd_global)
{
	u32 cfg;
    struct my_lcd_timing *time;

    time = mylcd_global->my_lcd_timing;

	cfg = 0;

	/*cfg |= S3C_VIDTCON0_VBPDE(time->v_bpe - 1);*/
	cfg |= S3C_VIDTCON0_VBPD(time->v_bpd - 1);
	cfg |= S3C_VIDTCON0_VFPD(time->v_fpd - 1);
	cfg |= S3C_VIDTCON0_VSPW(time->v_spw - 1);

	writel(cfg, mylcd_global->regs + S3C_VIDTCON0);

	cfg = 0;

	/*cfg |= S3C_VIDTCON1_VFPDE(time->v_fpe - 1);*/
	cfg |= S3C_VIDTCON1_HBPD(time->h_bpd - 1);
	cfg |= S3C_VIDTCON1_HFPD(time->h_fpd - 1);
	cfg |= S3C_VIDTCON1_HSPW(time->h_spw - 1);

	writel(cfg, mylcd_global->regs + S3C_VIDTCON1);
}

static void mylcd_cfg_size(struct mylcd_global *mylcd_global)
{
    u32 cfg;
    struct lcd_desc *lcd_desc;

    cfg = 0;
    lcd_desc = mylcd_global->lcd_desc;

    cfg  |= S3C_VIDTCON2_HOZVAL(lcd_desc->length - 1);
    cfg  |= S3C_VIDTCON2_LINEVAL(lcd_desc->height - 1);

    writel(cfg, mylcd_global->regs + S3C_VIDTCON2);
}

static void mylcd_display_on(struct mylcd_global *mylcd_global)
{
    u32 cfg;
    
    cfg = readl(mylcd_global->regs + S3C_VIDCON0);

    cfg |= ((1 << 1) | (1 << 0));
    writel(cfg, mylcd_global->regs + S3C_VIDCON0);
}

static void mylcd_cfg_register(struct mylcd_global *mylcd_global)
{
    u32 cfg;

    struct fb_info *fb = mylcd_global->fb_info;
	struct fb_fix_screeninfo *fix = &fb->fix;
	/*struct fb_var_screeninfo *var = &fb->var;*/
    struct my_lcd_timing *time;
    /*struct lcd_desc *lcd_desc;*/
	dma_addr_t start_addr = 0, end_addr = 0;

    /*lcd_desc = mylcd_global->lcd_desc;*/

    time = mylcd_global->my_lcd_timing;

    /*cfg = readl(my_lcd_timing->regs)*/
    cfg = 0;
    cfg &= ~(S3C_VIDCON0_VIDOUT_MASK);
    cfg |= (S3C_VIDCON0_VIDOUT_RGB) | (0 << 18) | (S3C_VIDCON0_PNRMODE_RGB_P) | (S3C_VIDCON0_CLKVALUP_ALWAYS) \
           | (11 << 6) | (S3C_VIDCON0_ENVID_DISABLE) | (S3C_VIDCON0_ENVID_F_DISABLE);

    writel(cfg, mylcd_global->regs + S3C_VIDCON0);

    cfg = readl(mylcd_global->regs + S3C_VIDCON1);

    cfg &= ~(3 << 9);
    cfg |= (time->rise_vclk << 7) | (time->i_hsync << 6) | (time->i_vsync << 5) | (time->i_vden << 4);
    writel(cfg, mylcd_global->regs + S3C_VIDCON1);

    start_addr = fix->smem_start;
    end_addr = fix->smem_start + fb->screen_size;
    writel(start_addr, S3C_VIDADDR_START0(0));
    writel(end_addr, S3C_VIDADDR_END0(0));

#if 0
#define S3C_VIDADDR_START0(x)	(0x00A0 + (x * 0x08))
#define S3C_VIDADDR_START1(x)	(0x00A4 + (x * 0x08))
#define S3C_VIDADDR_END0(x)	(0x00D0 + (x * 0x08))
#define S3C_VIDADDR_END1(x)	(0x00D4 + (x * 0x08))
#
#endif

}

static int mylcd_init(void)
{
    int ret;
    struct clk *clk;

	mylcd_global = kzalloc(sizeof(struct mylcd_global), GFP_KERNEL);
    mylcd_global->my_lcd_timing = &my_lcd_timing;
    mylcd_global->lcd_desc = &lcd_desc;
    
    mylcd_global->fb_info = framebuffer_alloc(0, NULL);
    if (!mylcd_global->fb_info) {
        printk("No enough memory!\n");
        ret = -ENOMEM;
        goto err_allocfb;
    }

    /* Set the fix and var of fb_info*/ 
    ret = mylcd_set_fbinfo(mylcd_global);

    /* Configure the port for LCD func*/
    mylcd_cfg_gpio();

    /* Configure the timing */
    mylcd_cfg_timing(mylcd_global);
    mylcd_cfg_size(mylcd_global);
    mylcd_cfg_register(mylcd_global);

    /* enable the clk of lcd, sclk_fimd is the source of lcd*/ 
    clk = clk_get(NULL, "sclk_fimd");
    clk_set_rate(clk, 800000000);
    clk_enable(clk);
    /*clk_put(clk);*/
    mylcd_global->regs = ioremap(0x11c00000, SZ_32K);
    

    register_framebuffer(mylcd_global->fb_info);
    mylcd_display_on(mylcd_global);

    return 0;

err_allocfb:
    framebuffer_release(mylcd_global->fb_info);
    return ret;
}

static void mylcd_exit(void)
{

}

module_init(mylcd_init);
module_exit(mylcd_exit);
