/*
 * 
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#define IC_VALID_CHIPID  0x11

#define IC_CMD_CHIPID     0
#define IC_CMD_CODEVER    1
#define IC_CMD_GSTAT      2
#define IC_CMD_KEYS3      3
#define IC_CMD_KEYS4      4
#define IC_CMD_SLIDE      5
#define IC_CMD_GPIOS      6
#define IC_CMD_SUBVER     7
#define IC_CMD_CALIBRATE  10
#define IC_CMD_DRIVE_X    70
#define IC_CMD_PWMEN_X    74
#define IC_CMD_PWM_DUTY   76

#define IC_NUM_LEDS_X	8

//#define IC_CYCLE_INTERVAL	(1*HZ)
#define IC_CYCLE_INTERVAL	(30)

#define IC_CYCLE_INTERVAL_HAVE_KEY	(15)
#define IC_CYCLE_INTERVAL_NO_KEY		(2*HZ)


//#define KEY_LONG_KEY			80		//20*75 MS
//取消长按，把时间改成无限长
#define KEY_LONG_KEY			65000		



#define KEY_JIFFIES	(1 * HZ)			/* 1s */
#define KEY_DEBOUNCE_JIFFIES		(10 / (MSEC_PER_SEC / HZ))	/* 10ms */

#define D1_KEY_ADDRESS    0x18

static int ProjectNum;
static int gIsD1KeyBoard = 0;
/*
static unsigned char i2c_key_key2code2[] = {
	       KEY_BACKSPACE,KEY_F8,KEY_SPACE,KEY_DOWN,KEY_ENTER,
		KEY_F9,KEY_F3,KEY_LEFT,KEY_RIGHT,KEY_F7,
		KEY_UP, KEY_ESC 	
};*/
static unsigned char i2c_key_key2code_d0[] = {
	       KEY_3,KEY_2,KEY_6,KEY_1,KEY_5,
		KEY_9,KEY_4,KEY_8,KEY_7,KEY_F5,
		KEY_0, KEY_F6 	
};

static unsigned char i2c_key_key2code_d1[] = {
	       KEY_1,KEY_2,KEY_3,KEY_4,KEY_5,
		KEY_6,KEY_7,KEY_8,KEY_9,KEY_F5,
		KEY_0, KEY_F6 	
};


static unsigned char i2c_key_key2code_r7[] = {
	    KEY_F5,KEY_3,KEY_2,KEY_1,KEY_0,
		KEY_6,KEY_5,KEY_4,KEY_7,KEY_8,
		KEY_9, KEY_F6 	
};

//static unsigned char ismapflag=0;

static unsigned char i2c_key_capacity[] = {
	       KEY_8,KEY_3,KEY_2,KEY_1,KEY_0,
		KEY_6,KEY_5,KEY_4,KEY_7,KEY_9,
		KEY_RIGHTSHIFT,	KEY_LEFTSHIFT,
		KEY_F5,KEY_F6,KEY_F1,KEY_F2,
		KEY_SPACE,KEY_UP,KEY_BACKSPACE,
		KEY_RIGHT,KEY_F3,KEY_LEFT,KEY_DOWN,
		KEY_ENTER,KEY_F7,KEY_ESC,KEY_F8,KEY_F9
};

struct i2c_key_data {
	int irq;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work dwork;
	spinlock_t lock;        /* Protects canceling/rescheduling of dwork */
	unsigned short keycodes[ARRAY_SIZE(i2c_key_capacity)];
	u16 key_matrix;
	int delay_time;
	bool pre_enter_int;
	int irq_gpio;
	int long_key_time;
	int have_key_scan_time;
	int no_key_scan_time;
	int key1_timer;
	int key2_timer;
	struct timer_list timer;
};


static int i2c_key_read_block(struct i2c_client *client,
			     u8 inireg, u8 *buffer, unsigned int count)
{
	int error, idx = 0;

	/*
	 * Can't use SMBus block data read. Check for I2C functionality to speed
	 * things up whenever possible. Otherwise we will be forced to read
	 * sequentially.
	 */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C))	{

		error = i2c_smbus_write_byte(client, inireg + idx);
		if (error) {
			dev_err(&client->dev,
				"couldn't send request. Returned %d\n", error);
			return error;
		}

		error = i2c_master_recv(client, buffer, count);
		if (error != count) {
			dev_err(&client->dev,
				"couldn't read registers. Returned %d bytes\n", error);
			return error;
		}
	} else {

		while (count--) {
			int data;

			error = i2c_smbus_write_byte(client, inireg + idx);
			if (error) {
				dev_err(&client->dev,
					"couldn't send request. Returned %d\n", error);
				return error;
			}

			data = i2c_smbus_read_byte(client);
			if (data < 0) {
				dev_err(&client->dev,
					"couldn't read register. Returned %d\n", data);
				return data;
			}

			buffer[idx++] = data;
		}
	}

	return 0;
}

static int i2c_key_get_key(struct i2c_key_data *i2c_key)
{
	struct i2c_client *client = i2c_key->client;
	struct input_dev *input = i2c_key->input;
	char regs[2]={0,0};
	int ret;
	
	if(gIsD1KeyBoard){
		ret = i2c_key_read_block(client, 3, regs, 1);
	}else{
		ret = i2c_key_read_block(client, 0, regs, 2);
	}
	if (ret) {
		dev_err(&client->dev,
			"could not perform chip read.\n");
		return ret;
	}
		
	if(ProjectNum==1){ //R2M 7寸
		if(regs[0]){
			if(!i2c_key->key1_timer){
				input_report_key(input,i2c_key_key2code_r7[regs[0]-1],1);
				input_sync(input);
				input_report_key(input,i2c_key_key2code_r7[regs[0]-1],0);
				input_sync(input);
				i2c_key->key1_timer = i2c_key->long_key_time;
			}
		}
		else{
			i2c_key->key1_timer = 0;
		}
		
		if(regs[1]){
			if(!i2c_key->key2_timer) {
				input_report_key(input,i2c_key_key2code_r7[regs[1]-1],1);
				input_sync(input);
				input_report_key(input,i2c_key_key2code_r7[regs[1]-1],0);
				input_sync(input);	
				i2c_key->key2_timer = i2c_key->long_key_time;
			}
		}
		else{
			i2c_key->key2_timer = 0;
		}
	}else if(gIsD1KeyBoard){
		if(regs[0]){
			char ledsAllOffBuf[3] = {0X01,0XFF,0X00};
			char ledsOnBuf[3] = {0,0,0};

			if(!i2c_key->key1_timer){
				input_report_key(input,i2c_key_key2code_d1[regs[0]-1],1);
				input_sync(input);
				input_report_key(input,i2c_key_key2code_d1[regs[0]-1],0);
				input_sync(input);
				i2c_key->key1_timer = i2c_key->long_key_time;
			}
			
			i2c_master_send(client, ledsAllOffBuf, 3);

			
			ledsOnBuf[0] = 0x01;
			ledsOnBuf[1] = regs[0];
			ledsOnBuf[2] = 0x01;

			i2c_master_send(client, ledsOnBuf, 3);
			msleep(100);
			i2c_master_send(client, ledsAllOffBuf, 3);
			msleep(100);
			i2c_master_send(client, ledsOnBuf, 3);
			msleep(100);
			i2c_master_send(client, ledsAllOffBuf, 3);
		

			
		}
		else{
			i2c_key->key1_timer = 0;
		}
		
	}else{  //D0M 4.3寸
		if(regs[0]){
			if(!i2c_key->key1_timer){
				input_report_key(input,i2c_key_key2code_d0[regs[0]-1],1);
				input_sync(input);
				input_report_key(input,i2c_key_key2code_d0[regs[0]-1],0);
				input_sync(input);
				i2c_key->key1_timer = i2c_key->long_key_time;
			}
		}
		else{
			i2c_key->key1_timer = 0;
		}
		if(regs[1]){
			if(!i2c_key->key2_timer) {
				input_report_key(input,i2c_key_key2code_d0[regs[1]-1],1);
				input_sync(input);
				input_report_key(input,i2c_key_key2code_d0[regs[1]-1],0);
				input_sync(input);	
				i2c_key->key2_timer = i2c_key->long_key_time;
			}
		}
		else{
			i2c_key->key2_timer = 0;
		}	
	}

	return 0;
}

static irqreturn_t i2c_key_irq(int irq, void *_i2c_key)
{
	struct i2c_key_data *i2c_key = _i2c_key;
	unsigned long flags;
	//printk(KERN_INFO "my drv irq happend!\n");
	spin_lock_irqsave(&i2c_key->lock, flags);

	mod_delayed_work(system_wq, &i2c_key->dwork, 0);

	spin_unlock_irqrestore(&i2c_key->lock, flags);

	return IRQ_HANDLED;
}

static void i2c_key_schedule_read(struct i2c_key_data *i2c_key)
{
	spin_lock_irq(&i2c_key->lock);
	schedule_delayed_work(&i2c_key->dwork, i2c_key->delay_time);
	spin_unlock_irq(&i2c_key->lock);
}

static void i2c_key_worker(struct work_struct *work)
{
	int val;
	struct i2c_key_data *i2c_key =
		container_of(work, struct i2c_key_data, dwork.work);

	//dev_dbg(&i2c_key->client->dev, "worker\n");

	i2c_key_get_key(i2c_key);
	i2c_key->pre_enter_int = true;
	
	/* Avoid device lock up by checking every so often */
	//i2c_key_schedule_read(i2c_key);
	//modify worker timer
	val = gpio_get_value(i2c_key->irq_gpio); 
	if(val){
		i2c_key->delay_time =  i2c_key->no_key_scan_time;
		//printk("########## have key rescan key###########");
	}
	else{
		i2c_key->delay_time = i2c_key->have_key_scan_time;
	}
	i2c_key_schedule_read(i2c_key);	
	
}


static void touch_keys_timer(unsigned long data)
{
	struct i2c_key_data *i2c_key = (struct i2c_key_data*)data;

	if(i2c_key->key1_timer >0)
		i2c_key->key1_timer --;
	
	if(i2c_key->key2_timer >0)
		i2c_key->key2_timer --;
	
	mod_timer(&i2c_key->timer, jiffies + KEY_DEBOUNCE_JIFFIES);
		
}

static int i2c_key_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_key_data *i2c_key;
	struct input_dev *input;
	unsigned long irq_flags;
	struct device_node *np = client->dev.of_node;
	int i;
	int error;
	int rc;
	//printk(KERN_INFO "my drv start probe!!\n");
	/* Check functionality */
	error = i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE);
	//printk(KERN_INFO "my drv i2c check succ!\n");
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	//printk(KERN_INFO "my drv start identify!\n");
	//if (!i2c_key_identify(client))
	//	return -ENODEV;

	//printk(KERN_INFO "my drv start input device!\n");
	/* Chip is valid and active. Allocate structure */
	i2c_key = kzalloc(sizeof(struct i2c_key_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!i2c_key || !input) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	//printk(KERN_INFO "my drv start init delaywork!\n");
	i2c_key->client = client;
	i2c_key->input = input;
	i2c_key->delay_time = IC_CYCLE_INTERVAL;
	INIT_DELAYED_WORK(&i2c_key->dwork, i2c_key_worker);
	spin_lock_init(&i2c_key->lock);

	input->name = "WRT D0 TOUCH_KEYBOARD";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x1001;
	input->id.product = 0x0011;
	input->id.version = 0x0003;

	input->keycode = i2c_key->keycodes;
	input->keycodesize = sizeof(i2c_key->keycodes[0]);
	input->keycodemax = ARRAY_SIZE(i2c_key_capacity);

	__set_bit(EV_KEY, input->evbit);
	__clear_bit(EV_REP, input->evbit);
	for (i = 0; i < ARRAY_SIZE(i2c_key_capacity); i++) {
		i2c_key->keycodes[i] = i2c_key_capacity[i];
		__set_bit(i2c_key_capacity[i], input->keybit);
	}
	__clear_bit(KEY_RESERVED, input->keybit);


	
	i2c_key->irq_gpio = of_get_named_gpio_flags(np,"irq_gpio",0,(enum of_gpio_flags *)&irq_flags);
	if(!gpio_is_valid(i2c_key->irq_gpio)){
		dev_err(&client->dev, "failed to get i2c irq gpio!\n");
		goto err_free_mem;
	}
	client->irq = i2c_key->irq_gpio;
	gpio_request(client->irq,"i2c-key");
	gpio_direction_input(client->irq);
	//printk(KERN_INFO "my drv start reg irq,gpio = 0x%X!\n",client->irq);
	if (client->irq>=0) {
		i2c_key->irq = gpio_to_irq(client->irq);
		//printk(KERN_INFO "my drv start reg irq,irq = %d!\n",i2c_key->irq);
		error = request_irq(i2c_key->irq, i2c_key_irq,
				    IRQF_TRIGGER_FALLING, "i2c_key", i2c_key);
		if (error) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", i2c_key->irq);
			goto err_free_mem;
		}
	}
	rc = of_property_read_u32(np, "ikey,long-key-time", &i2c_key->long_key_time);
        if (rc) {
		i2c_key->long_key_time = KEY_LONG_KEY;
		//dev_warn(&client->dev,"ikey,long-key-time use defaule %d\n",i2c_key->long_key_time);
        }
	rc = of_property_read_u32(np, "ikey,have-key-scan-time", &i2c_key->have_key_scan_time);
        if (rc) {
		i2c_key->have_key_scan_time = IC_CYCLE_INTERVAL_HAVE_KEY;
	//	dev_warn(&client->dev,"ikey,have-key-scan-time use defaule  %d\n",i2c_key->have_key_scan_time);
        }	
		
	rc = of_property_read_u32(np, "ikey,no-key-scan-time", &i2c_key->no_key_scan_time);
		if (rc) {
		i2c_key->no_key_scan_time = IC_CYCLE_INTERVAL_NO_KEY;
		//dev_warn(&client->dev,	"ikey,no-key-scan-time use defaule  %d\n",i2c_key->no_key_scan_time);
	}
	
	/*根据不同项目 定义不同的按键值*/
	if(!of_property_read_u32(np, "wrt,project-num", &ProjectNum))
		printk("ProjectNum:%d\n",ProjectNum);
	
	
	i2c_key->key1_timer = 0;
	i2c_key->key2_timer = 0;

	//printk(KERN_INFO "my drv irq register success!\n");

	error = input_register_device(i2c_key->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device\n");
		goto err_free_irq;
	}

	setup_timer(&i2c_key->timer, touch_keys_timer, (unsigned long)i2c_key);
	mod_timer(&i2c_key->timer, jiffies + KEY_JIFFIES);

	i2c_set_clientdata(client, i2c_key);
	i2c_key_schedule_read(i2c_key);

	return 0;


err_free_irq:
	if (client->irq)
		free_irq(i2c_key->irq, i2c_key);
err_free_mem:
	input_free_device(input);
	kfree(i2c_key);
	return error;
}

static int i2c_key_remove(struct i2c_client *client)
{
	struct i2c_key_data *i2c_key = i2c_get_clientdata(client);

	/* Release IRQ so no queue will be scheduled */
	if (i2c_key->irq)
		free_irq(i2c_key->irq, i2c_key);
	
	del_timer_sync(&i2c_key->timer);
	
	cancel_delayed_work_sync(&i2c_key->dwork);

	input_unregister_device(i2c_key->input);
	kfree(i2c_key);

	return 0;
}


//MODULE_DEVICE_TABLE(of, i2c_key_idtable_match);
static const struct i2c_device_id i2c_key_idtable[] = {
	{"ikey_i2c_key", 0,},
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_key_idtable);

static struct of_device_id i2c_key_idtable_match[] = {
	{.compatible = "ikey,i2c_key" },
	{ },
};
static struct i2c_driver i2c_key_driver = {
	.driver = {
		.name	= "ikey_i2c_key",
		.owner  = THIS_MODULE,
		.of_match_table	= of_match_ptr(i2c_key_idtable_match),
	},

	.id_table	= i2c_key_idtable,
	.probe		= i2c_key_probe,
	.remove		= i2c_key_remove,
};



static const struct i2c_device_id i2c_key_idtable_d1[] = {
	{"ikey_i2c_key_d1", 0,},
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_key_idtable_d1);

static struct of_device_id i2c_key_idtable_match_d1[] = {
	{.compatible = "ikey,i2c_key_d1" },
	{ },
};
static struct i2c_driver i2c_key_driver_d1 = {
	.driver = {
		.name	= "ikey_i2c_key_d1",
		.owner  = THIS_MODULE,
		.of_match_table	= of_match_ptr(i2c_key_idtable_match_d1),
	},

	.id_table	= i2c_key_idtable,
	.probe		= i2c_key_probe,
	.remove		= i2c_key_remove,
};

static int i2c_client_read(struct i2c_client *client, char *buf, int len)
{
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = client->flags | I2C_M_RD;
	msg.buf = buf;
	msg.len = len;
#ifdef CONFIG_I2C_ROCKCHIP_COMPAT
	msg.scl_rate = 100 * 1000;
#endif

	return i2c_transfer(client->adapter, &msg, 1);
}


static int i2c_client_detect(unsigned int nr,unsigned short addr)
{
	char val[8];
	struct i2c_client client;

	client.flags = 0;
	client.addr = addr;
	client.adapter = i2c_get_adapter(nr);
    return i2c_client_read(&client, val, 1);
}

static void is_d1_keyboard(void)
{
	if(i2c_client_detect(1,D1_KEY_ADDRESS) > 0){
		printk("is D1 keyboard !!!");
		gIsD1KeyBoard = 1;
		return;
	}
	printk("is D0 keyboard !!!");
	gIsD1KeyBoard = 0;
	
}

//module_i2c_driver(i2c_key_driver);
static int __init my_init(void)
{
	is_d1_keyboard();
	if(gIsD1KeyBoard > 0){
		printk("D1 keyboard init !!!");
		return i2c_add_driver(&i2c_key_driver_d1);
	}
	printk("D0 keyboard init !!!");
	return i2c_add_driver(&i2c_key_driver);
}
static void __exit my_exit(void)
{
	if(gIsD1KeyBoard > 0){
		printk("D1 keyboard exit !!!");
		i2c_del_driver(&i2c_key_driver_d1);
	}else{
		printk("D1 keyboard exit !!!");
		i2c_del_driver(&i2c_key_driver);
	}
}

MODULE_AUTHOR("SeanChan");
MODULE_DESCRIPTION("touch_key_Driver for WRT D0/D1/R7 main machine with 4.inch");
MODULE_LICENSE("GPL");

module_init(my_init);
module_exit(my_exit);

