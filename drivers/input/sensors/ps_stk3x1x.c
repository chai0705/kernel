/*
 *  ps_stk3x1x.c - Linux kernel modules for sensortek stk301x, stk321x and stk331x 
 *  proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2015 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include   <linux/fs.h>   
#include  <asm/uaccess.h> 
#include <linux/sensor-dev.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif
 
#define DRIVER_VERSION  "3.10.0_0429"
 
//#define STK_POLL_PS
#define STK_TUNE0
#define STK_DEBUG_PRINTF
 
#include "linux/stk3x1x.h"
 
/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_ALSCTRL_REG 			0x02
#define STK_LEDCTRL_REG 			0x03
#define STK_INT_REG 				0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_OFFSET_REG 	0x15
#define STK_DATA2_OFFSET_REG 	0x16
#define STK_DATA1_IR_REG 		0x17
#define STK_DATA2_IR_REG 		0x18
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80	
 
#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02
#define STK_STATE_EN_PS_MASK	0x01
 
#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUI_MASK			0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01
 
#define STK_INT_ALS				0x08
 
/*****************************************************************************/
#define STK_MAX_MIN_DIFF	200
#define STK_LT_N_CT	100
#define STK_HT_N_CT	150
/*****************************************************************************/
#define STK3310SA_PID		0x17
#define STK3311SA_PID		0x1E
#define STK3311WV_PID	0x1D
/*****************************************************************************/
 
/* INT 0x04 */
#define	PS_INT_DISABLE		0xF8
#define	PS_INT_ENABLE		(1 << 0)
#define	PS_INT_ENABLE_FLGNFH	(2 << 0)
#define	PS_INT_ENABLE_FLGNFL	(3 << 0)
#define	PS_INT_MODE_ENABLE	(4 << 0)
#define	PS_INT_ENABLE_THL	(5 << 0)
#define	PS_INT_ENABLE_THH	(6 << 0)
#define	PS_INT_ENABLE_THHL	(7 << 0)
#define	ALS_INT_DISABLE		(0 << 3)
#define	ALS_INT_ENABLE		(1 << 3)
#define	INT_CTRL_PS_OR_LS	(0 << 7)
#define	INT_CTRL_PS_AND_LS	(1 << 7)
 
/* FLAG 0x10 */
/* FLAG 0x10 */
#define	STK_FLAG_NF	(1 << 0)
#define	STK_FLAG_IR_RDY	(1 << 1)
#define	STK_FLAG_OUI	(1 << 2)
#define	STK_FLAG_PSINT	(1 << 4)
#define	STK_FLAG_ALSINT	(1 << 5)
#define	STK_FLAG_PSDR	(1 << 6)
#define	STK_FLAG_ALSDR	(1 << 7)
 
 
 
#ifdef STK_ALS_FIR
	#define STK_FIR_LEN	8
	#define MAX_FIR_LEN 32
	
struct data_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif
 
struct stk3x1x_data {
	int		int_pin;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
#ifdef CALI_PS_EVERY_TIME	
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif	
	int32_t ps_distance_last;
	bool ps_enabled;
	// bool re_enable_ps;
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
	int stk_max_min_diff;
	int stk_lt_n_ct;
	int stk_ht_n_ct;
#endif	
	atomic_t	recv_reg;
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
};
 
struct stk3x1x_data *ps_data;
 
/*****************************************************************************/
 
static struct stk3x1x_platform_data stk3x1x_pfdata={ 
  .state_reg = 0x0,    /* disable all */ 
  .psctrl_reg = 0x31,    /* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */ 
  .alsctrl_reg = 0x39, 	/* als_persistance=1, als_gain=64X, ALS_IT=100ms */
  .ledctrl_reg = 0xFF,   /* 100mA IRDR, 64/64 LED duty */ 
  .wait_reg = 0x07,    /* 50 ms */   
  .ps_thd_h = 1700, 
  .ps_thd_l = 1500, 
  //.int_pin = sprd_3rdparty_gpio_pls_irq,  
  .transmittance = 500, 
#if defined(CONFIG_SOFIA_3GR_BND_I706)
  .stk_max_min_diff = 15,
  .stk_lt_n_ct = 50,
  .stk_ht_n_ct = 75, 
#else
  .stk_max_min_diff = 10,
  .stk_lt_n_ct = 45,
  .stk_ht_n_ct = 60,
#endif
};
 
 
static int32_t stk3x1x_check_pid(struct i2c_client *client);
static int stk_ps_tune_zero_init(struct i2c_client *client);
 
 
/*****************************************************************************/
 
static int32_t stk3x1x_set_ps_thd_h(struct i2c_client *client, uint16_t thd_h)
{	
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
 
	ret = sensor_write_reg(client, STK_THDH1_PS_REG, val[0]);
	if(ret < 0){
		printk("%s: fail, ret=%d\n", __func__, ret);	
	}
 
	ret = sensor_write_reg(client, STK_THDH2_PS_REG, val[1]);
	if(ret < 0){
		printk("%s: fail, ret=%d\n", __func__, ret);	
	}
	
	return ret;
}
 
 
static int32_t stk3x1x_set_ps_thd_l(struct i2c_client *client, uint16_t thd_l)
{	
	unsigned char val[2];
	int ret;
 
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;	
 
	ret = sensor_write_reg(client, STK_THDL1_PS_REG, val[0]);
	if(ret < 0){
		printk("%s: fail, ret=%d\n", __func__, ret);	
	}
 
	ret = sensor_write_reg(client, STK_THDL2_PS_REG, val[1]);
	if(ret < 0){
		printk("%s: fail, ret=%d\n", __func__, ret);	
	}
	
	return ret;
}
/* 
static uint32_t stk3x1x_get_ps_reading(struct i2c_client *client, u16 *data)
 {	 
	 unsigned char value[2];
	 int err = 0;
 
	 value[0] = sensor_read_reg(client, STK_DATA1_PS_REG);
	 if(value[0] < 0){
		goto EXIT_ERR;
	 }
 
	 value[1] = sensor_read_reg(client, STK_DATA2_PS_REG);
	 if(value[1] < 0){
		goto EXIT_ERR;
	 }
 
	 *data = ((value[0]<<8) | value[1]);
	 return 0; 
 
EXIT_ERR:
	printk("stk3x1x_read_ps fail\n");
	return err;
 }
 */
static int proximity_sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	// u16 ps_code;
#ifdef STK_DEBUG_PRINTF	
	printk("%s init proc = %d\n", __func__, (ps_data->tune_zero_init_proc ? 1 : 0));
#endif	
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	
	sensor->ops->ctrl_data &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK); 
	if(enable)
	{
		sensor->ops->ctrl_data |= STK_STATE_EN_PS_MASK;	
		if(!(sensor->ops->ctrl_data & STK_STATE_EN_ALS_MASK))
			sensor->ops->ctrl_data |= STK_STATE_EN_WAIT_MASK;			
	}
#ifdef STK_DEBUG_PRINTF		
	printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
#endif
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
 
 
	if(enable)
	{
		usleep_range(4000, 5000);
		sensor->ops->report(sensor->client);
		
		// stk3x1x_get_ps_reading(client, &ps_code);
		// stk3x1x_set_ps_thd_h(client, ps_code + STK_HT_N_CT);
		// stk3x1x_set_ps_thd_l(client, ps_code + STK_LT_N_CT);
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: thdh:%d, thdl:%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
#endif	
		
		stk3x1x_set_ps_thd_h(client, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(client, ps_data->ps_thd_l);		
	}
	ps_data->ps_enabled = enable?true:false;
	ps_data->ps_distance_last = 1;	
 
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;	
	ps_data->psi_set = 0;
 
	ps_data->stk_max_min_diff = stk3x1x_pfdata.stk_max_min_diff;
	ps_data->stk_lt_n_ct = stk3x1x_pfdata.stk_lt_n_ct;
	ps_data->stk_ht_n_ct = stk3x1x_pfdata.stk_ht_n_ct;
 
#ifdef STK_DEBUG_PRINTF				
	printk(KERN_INFO "%s: lt:%d ht:%d max diff:%d\n", __func__, ps_data->stk_lt_n_ct, ps_data->stk_ht_n_ct, ps_data->stk_max_min_diff);
#endif	
	return result;
 
}
 
static int32_t stk3x1x_check_pid(struct i2c_client *client)
{
	char value[2] = {0}, pid_msb;
	int result;
	
	ps_data->p_wv_r_bd_with_co = 0;
	
	value[0] = STK_PDT_ID_REG;
	result = sensor_rx_data(client, value, 2);	
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "%s: PID=0x%x, RID=0x%x\n", __func__, value[0], value[1]);
#endif
	ps_data->pid = value[0];
	
	if(value[0] == STK3311WV_PID)
		ps_data->p_wv_r_bd_with_co |= 0b100;
	if(value[1] == 0xC3)
		ps_data->p_wv_r_bd_with_co |= 0b010;
		
	// if(stk3x1x_read_otp25(ps_data) == 1)
	// {
		// ps_data->p_wv_r_bd_with_co |= 0b001;
	// }
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "%s: p_wv_r_bd_with_co = 0x%x\n", __func__, ps_data->p_wv_r_bd_with_co);	
#endif
 
	pid_msb = value[0] & 0xF0;
	switch(pid_msb)
	{
	case 0x10:
	case 0x20:
	case 0x30:
		return 0;
	default:
		printk(KERN_ERR "%s: invalid PID(%#x)\n", __func__, value[0]);	
		return -1;
	}
	return 0;
}
 
/*
static int stk_allreg(struct i2c_client *client)
{
	uint8_t ps_reg[0x22];
	int cnt = 0;	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		if(ps_reg[cnt] < 0)
		{
			printk("%s fail \n", __func__);	
			return -EINVAL;
		}
#ifdef STK_DEBUG_PRINTF			
		printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
#endif
	}	
	return 0;
}
*/
#ifdef STK_TUNE0	
static int stk_ps_val(struct i2c_client *client)
{
	int mode;
	int32_t word_data, lii;	
	unsigned char value[4];
	int ret;
	
	value[0] = 0x20;
	ret = sensor_rx_data(client, value, 4);	
	if(ret)
	{
		printk("%s:line=%d,error=%d\n",__func__,__LINE__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	word_data += ((value[2]<<8) | value[3]);	
	
	mode = (stk3x1x_pfdata.psctrl_reg) & 0x3F;
	if(mode == 0x30)	
		lii = 100;	
	else if (mode == 0x31)
		lii = 200;		
	else if (mode == 0x32)
		lii = 400;		
	else if (mode == 0x33)
		lii = 800;	
	else
	{
		printk(KERN_ERR "%s: unsupported PS_IT(0x%x)\n", __func__, mode);
		return -1;
	}
	
	if(word_data > lii)
	{
		printk(KERN_INFO "%s: word_data=%d, lii=%d\n", __func__, word_data, lii);	
		return 0xFFFF;	
	}
	return 0;
}	
 
static int stk_ps_tune_zero_final(struct i2c_client *client)
{
	int ret;
	int value;
 
	value = 0;
#ifndef STK_POLL_PS	
	value |= 0x01;		
#endif
#ifndef STK_POLL_ALS
	value |= STK_INT_ALS;
#endif
	ret = sensor_write_reg(client, STK_INT_REG, value);
	if(ret <= 0)
		return ret;	
 
	value = sensor_read_reg(client, STK_STATE_REG);
 
	if(!(value & STK_STATE_EN_ALS_MASK))
		value |= STK_STATE_EN_WAIT_MASK;
	if(ps_data->ps_enabled)
		value |= STK_STATE_EN_PS_MASK;
	
	ret = sensor_write_reg(client, STK_STATE_REG, value);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	
	if(ps_data->data_count == -1)
	{
		printk(KERN_INFO "%s: exceed limit\n", __func__);
		ps_data->tune_zero_init_proc = false;
		return 0;
	}
	
	ps_data->psa = ps_data->ps_stat_data[0];
	ps_data->psi = ps_data->ps_stat_data[2];	
	ps_data->ps_thd_h = ps_data->ps_stat_data[1] + ps_data->stk_ht_n_ct;
	ps_data->ps_thd_l = ps_data->ps_stat_data[1] + ps_data->stk_lt_n_ct;			
	stk3x1x_set_ps_thd_h(client, ps_data->ps_thd_h);
	stk3x1x_set_ps_thd_l(client, ps_data->ps_thd_l);	
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "stk %s: set HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);		
#endif
	ps_data->tune_zero_init_proc = false;
	return 0;
}
	
static int32_t stk_tune_zero_get_ps_data(struct i2c_client *client, int ps_adc)
{
	int ret;
	
	ret = stk_ps_val(client);	
	if(ret == 0xFFFF)
	{
		ps_data->data_count = -1;
		stk_ps_tune_zero_final(client);
		return 0;
	}
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, ps_data->data_count, ps_adc);
#endif
	ps_data->ps_stat_data[1] +=  ps_adc;			
	if(ps_adc > ps_data->ps_stat_data[0])
		ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < ps_data->ps_stat_data[2])
		ps_data->ps_stat_data[2] = ps_adc;						
	ps_data->data_count++;	
	
	if(ps_data->data_count == 5)
	{
		ps_data->ps_stat_data[1]  /= ps_data->data_count;			
		stk_ps_tune_zero_final(client);
	}		
	
	return 0;
}
 
static int stk_ps_tune_zero_init(struct i2c_client *client)
{
	//struct sensor_private_data *sensor =
	//    (struct sensor_private_data *) i2c_get_clientdata(client);		
	
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;	
	ps_data->psi_set = 0;	
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	
	ps_data->tune_zero_init_proc = false;	
 
	/*
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	sensor->ops->ctrl_data &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK); 
	sensor->ops->ctrl_data |= STK_STATE_EN_PS_MASK;	
	if(!(sensor->ops->ctrl_data & STK_STATE_EN_ALS_MASK))
		sensor->ops->ctrl_data |= STK_STATE_EN_WAIT_MASK;			
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
#ifdef STK_DEBUG_PRINTF		
	printk("%s:reg=0x%x,reg_ctrl=0x%x\n",__func__,sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
#endif
	result = sensor_write_reg(client, STK_INT_REG, 0);
	if(result)
		printk("%s:fail to active sensor\n",__func__);
	*/
	return 0;	
}
 
static int stk_ps_tune_zero_func_fae(struct i2c_client *client, int word_data)
{
	int ret, diff;
#ifdef STK_DEBUG_PRINTF	
	//int cnt = 0;
	//int ps_reg[0x22];
#endif
	
	
	if(ps_data->psi_set || !(ps_data->ps_enabled))
		return 0;
	
	ret = stk_ps_val(client);	
	if(ret == 0)
	{				
		if(word_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
#ifdef STK_DEBUG_PRINTF				
			printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
#endif
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;	
#ifdef STK_DEBUG_PRINTF				
			printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
#endif
		}	
	}	
	diff = ps_data->psa - ps_data->psi;
#ifdef STK_DEBUG_PRINTF				
	printk(KERN_INFO "%s: diff:%d  psa:%d, psi:%d max diff:%d\n", __func__, diff, ps_data->psa, ps_data->psi, ps_data->stk_max_min_diff);
#endif	
	if(diff > ps_data->stk_max_min_diff)
	{
		ps_data->psi_set = ps_data->psi;
		ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
		ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: tune0 thd_h:%d	thd_l:%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
#endif
 
#if 0 //def STK_DEBUG_PRINTF	
		cnt = 0x6;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x7;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x8;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x9;
		ps_reg[cnt] = sensor_read_reg(client, cnt); 
		
		printk(KERN_INFO "%s: befor [0x06/0x07]%d, %d  [0x08/0x09]%d, %d	\n", __func__, ps_reg[0x6], ps_reg[0x7], ps_reg[0x8], ps_reg[0x9]);
#endif
 
 
		stk3x1x_set_ps_thd_h(client, ps_data->ps_thd_h);
		stk3x1x_set_ps_thd_l(client, ps_data->ps_thd_l);
 
#if 0 //def STK_DEBUG_PRINTF	
		cnt = 0x6;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x7;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x8;
		ps_reg[cnt] = sensor_read_reg(client, cnt);
		cnt = 0x9;
		ps_reg[cnt] = sensor_read_reg(client, cnt);	
 
		printk(KERN_INFO "%s: after [0x06/0x07]%d, %d  [0x08/0x09]%d, %d  \n", __func__, ps_reg[0x6], ps_reg[0x7], ps_reg[0x8], ps_reg[0x9]);
#endif
 
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: FAE tune0 found thd_h:%d  thd_l:%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
#endif
	}
	
	return 0;
}	
#endif
 
static int stk_ps_report(struct i2c_client *client, int ps)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	char buffer[2] = {0};	
	int reg_flag = 0;
	
#if 0 //def STK_DEBUG_PRINTF	
	int cnt = 0;
	int ps_reg[0x22];	
#endif
 
 
	buffer[0] = STK_FLAG_REG;
	result = sensor_rx_data(client, buffer, 1);	
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	reg_flag = buffer[0];
	reg_flag = (reg_flag & 0x1);
 
	ps_data->ps_distance_last = reg_flag ? 1:0;
 
#if 0 //def STK_DEBUG_PRINTF	
	cnt = 0x6;
	ps_reg[cnt] = sensor_read_reg(client, cnt);
	cnt = 0x7;
	ps_reg[cnt] = sensor_read_reg(client, cnt);
	cnt = 0x8;
	ps_reg[cnt] = sensor_read_reg(client, cnt);
	cnt = 0x9;
	ps_reg[cnt] = sensor_read_reg(client, cnt);	
 
	printk(KERN_INFO "%s: [0x06/0x07]%d, %d  [0x08/0x09]%d, %d  \n", __func__, ps_reg[0x6], ps_reg[0x7], ps_reg[0x8], ps_reg[0x9]);
#endif
 
	if(ps > ps_data->ps_thd_h){
		ps_data->ps_distance_last = 0;
	}else if(ps < ps_data->ps_thd_l){
		ps_data->ps_distance_last = 1;
	}
 
	input_report_abs(sensor->input_dev, ABS_DISTANCE, ps_data->ps_distance_last);
	input_sync(sensor->input_dev);
#ifdef STK_DEBUG_PRINTF		
	printk("%s:ps=0x%x,flag=%d, dis=%d\n",__func__, ps,reg_flag, ps_data->ps_distance_last);
#endif
 
	
	return 0;
}
 
static int proximity_sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
 
	int res = 0;
	char value;
	int val = 0;
	
	printk("stk %s init ...\n", __func__);
	
	ps_data = kzalloc(sizeof(struct stk3x1x_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_ERR "%s: failed to allocate stk3x1x_data\n", __func__);
		return -ENOMEM;
	}
	res = sensor_write_reg(client, STK_WAIT_REG, 0x7F);
	if(res < 0){
		printk("stk %s int error 1\n", __func__);	
		goto EXIT_ERR;
	}
	value = sensor_read_reg(client, STK_WAIT_REG);
	if(value != 0x7F)
	{
		printk("stk %s i2c test error\n", __func__);		
		goto EXIT_ERR;
	}
 
	res = sensor_write_reg(client, STK_SW_RESET_REG, 0x0);
	if(res < 0){
		printk("stk %s int error 2\n", __func__);	
		goto EXIT_ERR;
	}
	
	//usleep_range(13000, 15000);	
	res = stk3x1x_check_pid(client);
	
	if(res < 0){
		printk("stk %s int error 3\n", __func__);	
		goto EXIT_ERR;
	}
	
	res = sensor_write_reg(client, STK_STATE_REG, stk3x1x_pfdata.state_reg);
	if(res < 0){
		printk("stk %s int error 4\n", __func__);	
		goto EXIT_ERR;
	}
	res = sensor_write_reg(client, STK_PSCTRL_REG, stk3x1x_pfdata.psctrl_reg);
	if(res < 0){
		printk("stk %s int error 5\n", __func__);	
		goto EXIT_ERR;
	}
 
	
	res = sensor_write_reg(client, STK_ALSCTRL_REG, stk3x1x_pfdata.alsctrl_reg);
	if(res < 0){
		printk("stk %s int error 6\n", __func__);	
		goto EXIT_ERR;
	}
 
	if(ps_data->pid == STK3310SA_PID || ps_data->pid == STK3311SA_PID)
		stk3x1x_pfdata.ledctrl_reg &= 0x3F;	
	res = sensor_write_reg(client, STK_LEDCTRL_REG, stk3x1x_pfdata.ledctrl_reg);
	if(res < 0){
		printk("stk %s int error 7\n", __func__);	
		goto EXIT_ERR;
	}
	
	res = sensor_write_reg(client, STK_WAIT_REG, stk3x1x_pfdata.wait_reg);
	if(res < 0){
		printk("stk %s int error 8\n", __func__);	
		goto EXIT_ERR;
	}	
	
	value = 0x0;
	res = sensor_write_reg(client, STK_INT_REG, value);
	if(res < 0){
		printk("stk %s int error 10\n", __func__);	
		goto EXIT_ERR;
	}
	
	stk3x1x_set_ps_thd_h(client, stk3x1x_pfdata.ps_thd_h);	
	stk3x1x_set_ps_thd_l(client, stk3x1x_pfdata.ps_thd_l);	
 
	printk("stk %s initing \n", __func__);		
#ifdef STK_TUNE0
	stk_ps_tune_zero_init(client);
#endif	
 
#ifdef STK_ALS_FIR
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
	atomic_set(&ps_data->firlength, STK_FIR_LEN);	
#endif
	atomic_set(&ps_data->recv_reg, 0);  	
	ps_data->ps_enabled = false;
	ps_data->ps_distance_last = 1;	
	ps_data->stk_max_min_diff = stk3x1x_pfdata.stk_max_min_diff;
	ps_data->stk_lt_n_ct = stk3x1x_pfdata.stk_lt_n_ct;
	ps_data->stk_ht_n_ct = stk3x1x_pfdata.stk_ht_n_ct;
	ps_data->ps_thd_h = stk3x1x_pfdata.ps_thd_h;
	ps_data->ps_thd_l = stk3x1x_pfdata.ps_thd_l;
	printk("stk %s init successful \n", __func__);
 
	val = sensor_read_reg(client, STK_INT_REG);
	val &= ~INT_CTRL_PS_AND_LS;
	if (sensor->pdata->irq_enable)
		val |= PS_INT_ENABLE;
	else
		val &= ~PS_INT_ENABLE;
	res = sensor_write_reg(client, STK_INT_REG, val);
	if (res) {
		dev_err(&client->dev, "%s:write INT_CTRL fail\n", __func__);
		return res;
	}
 
	return 0;
	
	
EXIT_ERR:
	printk(KERN_ERR "stk init fail dev: %d\n", res);
	return res;
 
 
}
 
static int proximity_sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	
	int result = 0;
	int value = 0;
	char buffer[2] = {0};	
	//printk("stk %s\n", __func__);
 
	if(sensor->ops->read_len < 2)	//sensor->ops->read_len = 1
	{
		printk("%s:lenth is error,len=%d\n",__func__,sensor->ops->read_len);
		return -1;
	}
	
	value = sensor_read_reg(client, STK_FLAG_REG);
	if(value < 0)
	{
		printk("stk %s read STK_FLAG_REG, ret=%d\n", __func__, value);
		return value;
	}
	if(!(value & STK_FLG_PSDR_MASK))
		return 0;
	
	memset(buffer, 0, 2);
	buffer[0] = sensor->ops->read_reg;
	result = sensor_rx_data(client, buffer, sensor->ops->read_len);	
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	value = (buffer[0] << 8) | buffer[1];
	if(value < 0) {
#ifdef STK_DEBUG_PRINTF
		printk("stk %s: value == %d return \n",__func__,value);
#endif				
		return result;
	}
	
#ifdef STK_DEBUG_PRINTF
	printk("stk %s: value == %d \n",__func__,value);
#endif		
#ifdef STK_TUNE0	
	if(ps_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(client, value);
	else
		stk_ps_tune_zero_func_fae(client, value);	
#endif	
	stk_ps_report(client, value);
 
	if (sensor->pdata->irq_enable && sensor->ops->int_status_reg) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		if (value & STK_FLAG_PSINT) {
			value &= ~STK_FLAG_PSINT;
			result = sensor_write_reg
				(client, sensor->ops->int_status_reg,
				value);
			if (result) {
				dev_err(&client->dev, "%s:write status reg error\n",
					__func__);
				return result;
			}
		}
	}
 
	return result;
}
 
struct sensor_operate proximity_stk3x1x_ops = {
	.name				= "ps_stk3x1x",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_STK3X1X,		//i2c id number
	.read_reg			= STK_DATA1_PS_REG,			//read data
	.read_len			= 2,				//data length
	.id_reg				= SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,		//device id
	.precision			= 16,				//16 bits
	.ctrl_reg 			= STK_STATE_REG,			//enable or disable 
	.int_status_reg 	= STK_FLAG_REG,			//intterupt status register
	.range				= {0,1},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,		
	.active				= proximity_sensor_active,	
	.init				= proximity_sensor_init,
	.report				= proximity_sensor_report_value,
	// int 	brightness[2];//backlight min_brightness max_brightness 
	// int int_ctrl_reg;
	// int (*suspend)(struct i2c_client *client);
	// int (*resume)(struct i2c_client *client);
	// struct miscdevice *misc_dev;	
};
 
/****************operate according to sensor chip:end************/
static int psensor_stk3x1x_probe(struct i2c_client *client,
				const struct i2c_device_id *devid)
{
	return sensor_register_device(client, NULL, devid, &proximity_stk3x1x_ops);
}
 
static int psensor_stk3x1x_remove(struct i2c_client *client)
{
	return sensor_unregister_device(client, NULL, &proximity_stk3x1x_ops);
}
 
static const struct i2c_device_id psensor_stk3x1x_id[] = {
	{"ps_stk3x1x", PROXIMITY_ID_STK3X1X},
	{}
};
 
MODULE_DEVICE_TABLE(i2c, psensor_stk3x1x_id);
 
static struct i2c_driver psensor_stk3x1x_driver = {
	.probe = psensor_stk3x1x_probe,
	.remove = psensor_stk3x1x_remove,
	.shutdown = sensor_shutdown,
	.id_table = psensor_stk3x1x_id,
	.driver = {
		.name = "psensor_stk3x1x",
	#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
	#endif
	},
};
 
module_i2c_driver(psensor_stk3x1x_driver);
 
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
