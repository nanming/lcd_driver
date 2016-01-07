
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
#include <linux/cma.h>

#define DRIVER_NAME "mylcd_4412"
#define S3CFB_NAME		"s3cfb"
#define CMA_REGION_FIMD 	"fimd"

static int mylcd_cfg_register(struct fb_info *info);
static unsigned int pseudo_pal[16];
static struct mylcd_global *mylcd_global;
static struct clk *global_clk;
static struct my_lcd_timing my_lcd_timing = {
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
    .height = 1280,
    .width= 800,
    .bpp    = 24,
    .freq   = 50,
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
    /*.fb_set_par	= mylcd_cfg_register,*/
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
    struct my_lcd_timing *time;

    int height = lcd_desc->height;
    int width = lcd_desc->width;
    int bpp    = lcd_desc->bpp;

    time = mylcd_global->my_lcd_timing;

	strcpy(fix->id, DRIVER_NAME);
    fix->smem_len    = height * width * bpp / 8; 
    fix->type        = FB_TYPE_PACKED_PIXELS;
    fix->visual      = FB_VISUAL_TRUECOLOR;
    fix->line_length = width * bpp / 8;
#if 0
	__u32 left_margin;		/* time from sync to picture	*/
	__u32 right_margin;		/* time from picture to sync	*/
	__u32 upper_margin;		/* time from sync to picture	*/
	__u32 lower_margin;
	__u32 hsync_len;		/* length of horizontal sync	*/
	__u32 vsync_len;		/* length of vertical sync	*/
	var->hsync_len = timing->h_sw;
	var->vsync_len = timing->v_sw;
	var->left_margin = timing->h_bp;
	var->right_margin = timing->h_fp;
	var->upper_margin = timing->v_bp;
	var->lower_margin = timing->v_fp;

#endif

    var->xres        = width;
    var->yres        = height;
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres;
    var->xoffset      = 0;
    var->yoffset      = 0;
    var->bits_per_pixel = bpp;
    var->left_margin  = time->h_bpd;
    var->right_margin = time->h_fpd;
    var->upper_margin = time->v_bpd;
    var->lower_margin = time->v_fpd;
    var->hsync_len    = time->h_spw;
    var->vsync_len    = time->v_spw;
    /*var->sync           =  FB_SYNC_ON_GREEN;*/

	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
    var->transp.offset = 0;
    var->transp.length = 0; /* added for LCD RGB32 */

    var->activate = FB_ACTIVATE_NOW;


	var->pixclock = (lcd_desc->freq *
		(var->left_margin + var->right_margin
		+ var->hsync_len + var->xres) *
		(var->upper_margin + var->lower_margin
		+ var->vsync_len + var->yres));
	var->pixclock = KHZ2PICOS(var->pixclock/1000);

    fb->fbops = &mylcd_fb_ops;
    fb->pseudo_palette = &pseudo_pal;


	fb->screen_size =  height * width * bpp / 8;

	fix->smem_start = (dma_addr_t)cma_alloc
		(mylcd_global->dev, CMA_REGION_FIMD, (size_t)fix->smem_len, 0);
	fb->screen_base = cma_get_virt(fix->smem_start, fix->smem_len, 1);

#if 0
	fb->screen_base = dma_alloc_writecombine(NULL,
						 PAGE_ALIGN(fix->smem_len),
						 (unsigned int *)
						 &fix->smem_start, GFP_KERNEL);
#endif

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

static int mylcd_bakclight_off(void)
{
    int err;
    /* LCD Backlight control*/ 
    err = gpio_request(EXYNOS4_GPL0(4), "BK_VDD_EN");
    if (err) {
            printk(KERN_ERR "failed to request BK_VDD_EN\n");
            //return err;
    }

    gpio_direction_output(EXYNOS4_GPL0(4), 0);
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
	gpio_direction_output(EXYNOS4_GPL1(0), 0);

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
	gpio_direction_output(EXYNOS4_GPD0(1), 0);
	gpio_free(EXYNOS4_GPD0(1));
#else
	gpio_direction_output(EXYNOS4_GPD0(1), 0);

	s3c_gpio_cfgpin(EXYNOS4_GPD0(1), EXYNOS4_GPD_0_1_TOUT_1);

	gpio_free(EXYNOS4_GPD0(1));
	printk("(%s, %d): LCD_PWM_ON\n", __FUNCTION__, __LINE__);
#endif
	mdelay(5);

#if 0
	err = gpio_request_one(EXYNOS4_GPC0(2), 0, "GPC0_2");
        if (err) {
                printk(KERN_ERR "failed to request GPC0_2 for "
                                "4.3 LCD control\n");
                return err;
        }

        s3c_gpio_setpull(EXYNOS4_GPC0(2), S3C_GPIO_PULL_UP);
        //gpio_set_value(EXYNOS4_GPC0(2), 0);

        //mdelay(10);

        gpio_set_value(EXYNOS4_GPC0(2), 0);

        gpio_free(EXYNOS4_GPC0(2));
#endif		

	err = gpio_request(EXYNOS4_GPC0(2), "VGA_EN");
        if (err) {
                printk(KERN_ERR "failed to request VGA_EN\n");
                return err;
        }

	gpio_direction_output(EXYNOS4_GPC0(2), 0);

	s3c_gpio_cfgpin(EXYNOS4_GPC0(2), S3C_GPIO_OUTPUT);
        gpio_free(EXYNOS4_GPC0(2));

	printk("(%s, %d): VGA_EN_OFF\n", __FUNCTION__, __LINE__);
	/* end add */
    return 0;

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

    cfg  |= S3C_VIDTCON2_HOZVAL(lcd_desc->width - 1);
    cfg  |= S3C_VIDTCON2_LINEVAL(lcd_desc->height - 1);

    writel(cfg, mylcd_global->regs + S3C_VIDTCON2);
}

static void mylcd_display_off(struct mylcd_global *mylcd_global)
{
    u32 cfg;
    
    cfg = readl(mylcd_global->regs + S3C_VIDCON0);

    cfg &= S3C_VIDCON0_ENVID_DISABLE;
    writel(cfg, mylcd_global->regs + S3C_VIDCON0);
	/*cfg |= (S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE);*/
    cfg &= S3C_VIDCON0_ENVID_F_DISABLE;
    writel(cfg, mylcd_global->regs + S3C_VIDCON0);
}

static void mylcd_display_on(struct mylcd_global *mylcd_global)
{
    u32 cfg;
    
    cfg = readl(mylcd_global->regs + S3C_VIDCON0);

    /*cfg |= ((1 << 1) | (1 << 0));*/
	cfg |= (S3C_VIDCON0_ENVID_ENABLE | S3C_VIDCON0_ENVID_F_ENABLE);
    writel(cfg, mylcd_global->regs + S3C_VIDCON0);
}

static int mylcd_cfg_register(struct fb_info *info)
{
    u32 cfg;
    u32 shw;

    /*struct fb_info *fb = mylcd_global->fb_info;*/
    struct fb_info *fb = info;
	struct fb_fix_screeninfo *fix = &fb->fix;
	/*struct fb_var_screeninfo *var = &fb->var;*/
    struct my_lcd_timing *time;
    struct lcd_desc *lcd_desc;
	dma_addr_t start_addr = 0, end_addr = 0;

    lcd_desc = mylcd_global->lcd_desc;

    time = mylcd_global->my_lcd_timing;

    shw = readl(mylcd_global->regs + S3C_WINSHMAP);
    shw |= S3C_WINSHMAP_PROTECT(0);
    writel(shw, mylcd_global->regs + S3C_WINSHMAP);

    cfg = readl(mylcd_global->regs + S3C_VIDCON0);
    cfg &= ~(S3C_VIDCON0_VIDOUT_MASK);
    cfg &= ~S3C_VIDCON0_CLKVAL_F(0xff);
	/*cfg |= S3C_VIDCON0_CLKVAL_F(div - 1);*/
    cfg |= ((S3C_VIDCON0_VIDOUT_RGB) | (0 << 18) | (S3C_VIDCON0_PNRMODE_RGB_P) | (S3C_VIDCON0_CLKVALUP_ALWAYS) \
           | (S3C_VIDCON0_CLKVAL_F(12)) | (S3C_VIDCON0_ENVID_DISABLE) | (S3C_VIDCON0_ENVID_F_DISABLE));

    writel(cfg, mylcd_global->regs + S3C_VIDCON0);

    cfg = readl(mylcd_global->regs + S3C_VIDCON1);

    cfg &= ~(3 << 9);
    cfg |= (time->rise_vclk << 7) | (time->i_hsync << 6) | (time->i_vsync << 5) | (time->i_vden << 4);
    writel(cfg, mylcd_global->regs + S3C_VIDCON1);

    start_addr = fix->smem_start;
    end_addr = fix->smem_start + fb->screen_size;
    writel(start_addr, mylcd_global->regs + S3C_VIDADDR_START0(0));
    writel(end_addr, mylcd_global->regs + S3C_VIDADDR_END0(0));

/*#define S3C_VIDADDR_PAGEWIDTH(x)		(((x) & 0x1fff) << 0)*/
    cfg |= (S3C_VIDADDR_OFFSIZE(0) | S3C_VIDADDR_PAGEWIDTH(800 * 24 / 8));
    writel(cfg, mylcd_global->regs + S3C_VIDADDR_SIZE(0));

    cfg = readl(mylcd_global->regs + S3C_WINCON(0));
    cfg &= ~(S3C_WINCON_BPPMODE_MASK);
    cfg |= (S3C_WINCON_DATAPATH_DMA | S3C_WINCON_BPPMODE_24BPP_888 | S3C_WINCON_ENWIN_ENABLE);
    writel(cfg, mylcd_global->regs + S3C_WINCON(0));

    cfg = readl(mylcd_global->regs + S3C_VIDOSD0A);
    cfg &= ~(S3C_VIDOSD_LEFT_X(0) | S3C_VIDOSD_TOP_Y(0));
    writel(cfg, mylcd_global->regs + S3C_VIDOSD0A);

    cfg = readl(mylcd_global->regs + S3C_VIDOSD0B); /*windows0's buffer size*/ 
    cfg &= ~(S3C_VIDOSD_RIGHT_X(0) | S3C_VIDOSD_BOTTOM_Y(0));
    cfg |= ((1279 << 11) | 799);
    writel(cfg, mylcd_global->regs + S3C_VIDOSD0B);

    /*cfg = readl(mylcd_global->regs + S3C_VIDOSD0C);*/
    /*cfg &= ~(S3C_VIDOSD_SIZE(0));*/
    /*cfg = (lcd_desc->height * lcd_desc->length * lcd_desc->bpp / 8);*/
    cfg = (lcd_desc->height * lcd_desc->width); /*windows 0's size*/ 
    writel(cfg, mylcd_global->regs + S3C_VIDOSD0C);

    cfg = readl(mylcd_global->regs + S3C_WINSHMAP);
    cfg |= (S3C_WINSHMAP_CH_ENABLE(0));
    writel(cfg, mylcd_global->regs + S3C_WINSHMAP);

    cfg = readl(mylcd_global->regs + 0x003C);
    cfg &= ~((7 << 16) | (7 << 0));
    cfg |= ((1 << 16) | ( 1 << 0));
    writel(cfg, mylcd_global->regs + 0x003C);


    shw = readl(mylcd_global->regs + S3C_WINSHMAP);
    shw &= ~S3C_WINSHMAP_PROTECT(0);
    writel(shw, mylcd_global->regs + S3C_WINSHMAP);
    return 0;

#if 0
#define S3C_WINCON(x)		(0x0020 + (x * 0x04))
#define S3C_VIDOSD_A(x)		(0x0040 + (x * 0x10))
#define S3C_VIDOSD_B(x)		(0x0044 + (x * 0x10))
#define S3C_VIDOSD_C(x)		(0x0048 + (x * 0x10))
#define S3C_VIDOSD_D(x)		(0x004C + (x * 0x10))
#define S3C_VIDADDR_START0(x)	(0x00A0 + (x * 0x08))
#define S3C_VIDADDR_START1(x)	(0x00A4 + (x * 0x08))
#define S3C_VIDADDR_END0(x)	(0x00D0 + (x * 0x08))
#define S3C_VIDADDR_END1(x)	(0x00D4 + (x * 0x08))
#define S3C_VIDADDR_SIZE(x)	(0x0100 + (x * 0x04))
#define S3C_KEYCON(x)		(0x0140 + ((x - 1) * 0x08))
#define S3C_KEYVAL(x)		(0x0144 + ((x - 1) * 0x08))
#define S3C_WINMAP(x)		(0x0180 + (x * 0x04))

#endif
}

static int __devexit myfb_remove(struct platform_device *pdev)
{
#if 1
    struct fb_info *info = mylcd_global->fb_info;
    struct fb_fix_screeninfo *fix = &info->fix;
	struct clk *lcd_clk = NULL;

	lcd_clk = clk_get(&pdev->dev, "lcd");
	if (IS_ERR(lcd_clk)) {
		printk(KERN_ERR "failed to get ip clk for fimd0\n");
		/*goto err_clk0;*/
	}

	clk_disable(lcd_clk);
	clk_put(lcd_clk);

    clk_disable(global_clk);
    clk_put(global_clk);

    printk("mylcd modules exit routine\n");
    mylcd_display_off(mylcd_global);
    mylcd_bakclight_off();
    unregister_framebuffer(info);
    iounmap(mylcd_global->regs);
    /*cma_free(fix->smem_start);*/
	if (fix->smem_start) {
        iounmap(info->screen_base);
        fix->smem_start = 0;
        fix->smem_len = 0;
    }
    /*dma_free_writecombine(NULL, PAGE_ALIGN(mylcd_global->fb_info->fix.smem_len),*/
            /*(unsigned int *) mylcd_global->fb_info->screen_base, mylcd_global->fb_info->fix.smem_start);*/
    framebuffer_release(info);
    kfree(info);
    kfree(mylcd_global);
#endif
    return 0;
}
static int __devinit myfb_probe(struct platform_device *pdev)
{
#if 1
    int ret;
    struct clk *clk = NULL;
    struct clk *mout_mpll = NULL;
	struct clk *lcd_clk = NULL;
	struct resource *res = NULL;

    printk("My first 4412 LCD driver Test Begin.\n");
	mylcd_global = kzalloc(sizeof(struct mylcd_global), GFP_KERNEL);
    mylcd_global->my_lcd_timing = &my_lcd_timing;
    mylcd_global->lcd_desc = &lcd_desc;
    mylcd_global->dev = &pdev->dev;
    
    /* enable the clk of lcd, sclk_fimd is the source of lcd*/ 
	lcd_clk = clk_get(&pdev->dev, "lcd");
	if (IS_ERR(lcd_clk)) {
        printk("failed to get clk for lcd\n");
        goto err_clk0;
    }

	ret = clk_enable(lcd_clk);
    if (ret < 0) {
        printk("failed to clk_enable for lcd\n");
        goto err_clk0;
    }
    clk_put(lcd_clk);

    clk = clk_get(&pdev->dev, "sclk_fimd");
    global_clk = clk;
	if (IS_ERR(clk)) {
        printk("failed to get clk for sclk_fimd\n");
        goto err_clk1;
    }

    mout_mpll = clk_get(&pdev->dev, "mout_mpll_user");
	if (IS_ERR(mout_mpll)) {
        printk("failed to get clk for lcd\n");
        goto err_clk2;
    }

	ret = clk_set_parent(clk, mout_mpll);
    clk_set_rate(clk, 800000000);
    clk_enable(clk);
	clk_put(mout_mpll);
    /*clk_put(clk);*/

    /* Configure the port for LCD func*/
    mylcd_cfg_gpio();

    mylcd_global->fb_info = framebuffer_alloc(0, mylcd_global->dev);
    if (!mylcd_global->fb_info) {
        printk("No enough memory!\n");
        ret = -ENOMEM;
        goto err_allocfb;
    }

    /* Set the fix and var of fb_info*/ 
    ret = mylcd_set_fbinfo(mylcd_global);

    res = request_mem_region(0x11c00000, SZ_32K, pdev->name);
    if (!res) {
        printk("Not enough memory for ioremap\n");
        return -ENOMEM;
    }
    mylcd_global->regs = ioremap(res->start, res->end - res->start + 1);
    if (!mylcd_global->regs) {
        printk("failed to ioremap io for mylcd\n");
        return -EINVAL;
    }

    /* Configure the timing */
    mylcd_cfg_timing(mylcd_global);
    mylcd_cfg_size(mylcd_global);
    mylcd_cfg_register(mylcd_global->fb_info);

    register_framebuffer(mylcd_global->fb_info);
    mylcd_display_on(mylcd_global);
#endif
    return 0;

err_allocfb:
    framebuffer_release(mylcd_global->fb_info);
err_clk2:
    clk_put(mout_mpll);
err_clk1:
    clk_put(clk);
err_clk0:
    clk_put(lcd_clk);

	return -EINVAL;
}

static struct platform_driver mylcdfb_driver = {
	.probe		= myfb_probe,
    .remove		= myfb_remove,
	/*.suspend	= s3cfb_suspend,*/
	/*.resume		= s3cfb_resume,*/
	.driver		= {
		.name	= S3CFB_NAME,
		.owner	= THIS_MODULE,
	},
};

static int mylcd_register(void)
{
	platform_driver_register(&mylcdfb_driver);
	return 0;
}
static void mylcd_exit(void)
{
    platform_driver_unregister(&mylcdfb_driver);
}

module_init(mylcd_register);
module_exit(mylcd_exit);
MODULE_AUTHOR("nanming <584114384@qq.com>");
MODULE_DESCRIPTION("Display Controller (FIMD) driver");
MODULE_LICENSE("GPL");

