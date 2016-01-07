#ifndef _MYLCD_H
#define _MYLCD_H __FILE__

#include <linux/fb.h>

struct my_lcd_timing {
	int	h_spw;
	int	h_bpd;
	int	h_fpd;
	int	v_spw;
	int	v_bpd;
	int	v_fpd;
    int rise_vclk;
    int i_hsync;  //VIDCON1 Specifies HSYNC pulse polarity. 0:normal, 1:inverted bit[6]
    int i_vsync;  //VIDCON1 Specifies VSYNC pulse polarity. 0:normal, 1:inverted bit[5]
    int i_vden;
};

struct lcd_desc {
    int height;
    int width;
    int bpp;
    int freq;
};

struct mylcd_global {
    void __iomem  *regs;
    struct fb_info *fb_info;
    struct lcd_desc *lcd_desc;
    struct my_lcd_timing *my_lcd_timing;
	struct device		*dev;
};
#endif
