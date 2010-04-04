/*
 * Bravo Modem Manager library
 *
 * Copyright (C) 2009 Intrinsyc Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/bravo_gpio.h>


/*****************************************************************************
 * Configuration
 *****************************************************************************/

#define BMM_GPIO_MODEM_POWER   S3C64XX_GPE(4)
#define BMM_GPIO_MODEM_ENABLE  S3C64XX_GPL(12)


/*****************************************************************************
 * Logging/Debugging
 *****************************************************************************/

#define BRAVO_MODEM_MGR_DEBUG          1
#define BRAVO_MODEM_MGR_DEBUG_VERBOSE  0

#if BRAVO_MODEM_MGR_DEBUG
#define DEBUGPRINT(x...) printk(x)
#else  /* BRAVO_MODEM_MGR_DEBUG */
#define DEBUGPRINT(x...)
#endif /* BRAVO_MODEM_MGR_DEBUG */

void bravo_modem_mgr_gpio_reg_dump(void)
{
    /* Note that we cheat here by hard-coding the registers that get dumped. */

    DEBUGPRINT(KERN_INFO "\n");
    DEBUGPRINT(KERN_INFO "MODEM GPIO REGISTERS\n");
    DEBUGPRINT(KERN_INFO "ModemMGR: GPECON  = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPECON));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPEPUD  = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPEPUD));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPEDAT  = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPEDAT));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPLCON0 = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPLCON));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPLCON1 = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPLCON1));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPLPUD  = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPLPUD));
    DEBUGPRINT(KERN_INFO "ModemMGR: GPLDAT  = 0x%08X\n", get_s3c_reg_val(S3C64XX_GPLDAT));
    DEBUGPRINT(KERN_INFO "\n");
}


/*****************************************************************************
 * Helpers - GPIO
 *****************************************************************************/

//
// Returns 0 if the given GPIO line is Low.
// Returns 1 if the given GPIO line is High.
//
static int GetGpioValue(unsigned int GpioID)
{
    int value = 0;

    s3c_gpdat_getval(GpioID, &value);

#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
    if (BMM_GPIO_MODEM_ENABLE == GpioID)
    {
        DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Modem Enable state: %s\n", ((0 == value) ? "DISABLED" : "ENABLED") );
    }

    if (BMM_GPIO_MODEM_POWER == GpioID)
    {
        DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Modem Power state: %s\n", ((0 == value) ? "DISABLED" : "ENABLED") );
    }
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

    return ((0 == value) ? 0: 1);
}

//
// Sets a GPIO Line to the given value only
// if it is not already set to that value.
//
static void SetGpioValue(unsigned int GpioID, int newValue)
{
    int oldValue = GetGpioValue(GpioID);

    // Only drive the line if the value is changing

    if (newValue != oldValue)
    {
#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
        if (BMM_GPIO_MODEM_ENABLE == GpioID)
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Setting Modem Enable state to %s\n", ((0 == newValue) ? "DISABLED" : "ENABLED") );
        }

        if (BMM_GPIO_MODEM_POWER == GpioID)
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Setting Modem Power state to %s\n", ((0 == newValue) ? "DISABLED" : "ENABLED") );
        }
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

        s3c_gpdat_setval(GpioID, newValue);
    }
    else
    {
#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
        if (BMM_GPIO_MODEM_ENABLE == GpioID)
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Ignoring requested Modem Enable state change since Modem Enable is already %s\n", ((0 == newValue) ? "DISABLED" : "ENABLED") );
        }

        if (BMM_GPIO_MODEM_POWER == GpioID)
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Ignoring requested Modem Power state change since Modem Power is already %s\n", ((0 == newValue) ? "DISABLED" : "ENABLED") );
        }
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE
    }
}


/*****************************************************************************
 * Helpers - Property Checks
 *****************************************************************************/

static bool bravo_modem_mgr_are_we_in_flight_mode(void)
{
    bool fInFlightMode = false;

    // For now, always return FALSE.

#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Flight Mode is %s.\n", (fInFlightMode ? "ENABLED" : "DISABLED") );
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

    return fInFlightMode;
}

static bool bravo_modem_mgr_are_we_in_grace_period(void)
{
    bool fInGracePeriod = false;

    // For now, always return FALSE.

#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Grace Period is %s.\n", (fInGracePeriod ? "ACTIVE" : "INACTIVE") );
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

    return fInGracePeriod;
}


/*****************************************************************************
 * Helpers - Modem On/Off
 *****************************************************************************/
static void bravo_modem_mgr_turn_modem_on(void)
{
    if (!bravo_modem_mgr_are_we_in_flight_mode())
    {
#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
        DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Enabling the modem.\n");
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

        s3c_gpio_cfgpin(S3C64XX_GPN(11), S3C_GPIO_INPUT);
        s3c_gpio_setpull(S3C64XX_GPN(11), S3C_GPIO_PULL_NONE);

        SetGpioValue(BMM_GPIO_MODEM_ENABLE, 1);
        msleep(200);
        SetGpioValue(BMM_GPIO_MODEM_POWER,  1);
    }
    else
    {
#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
        DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Keeping the modem OFF because we are in Flight Mode.\n");
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE
    }
}

static void bravo_modem_mgr_turn_modem_off(void)
{
#if BRAVO_MODEM_MGR_DEBUG_VERBOSE
        DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Disabling the modem.\n");
#endif // BRAVO_MODEM_MGR_DEBUG_VERBOSE

    SetGpioValue(BMM_GPIO_MODEM_ENABLE, 0);
    msleep(50);
    SetGpioValue(BMM_GPIO_MODEM_POWER,  0);
    
    s3c_gpio_cfgpin(S3C64XX_GPN(11), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S3C64XX_GPN(11), S3C_GPIO_PULL_NONE);
    s3c_gpdat_setval(S3C64XX_GPN(11), 0);

}


/*****************************************************************************
 * SysFS Handlers
 *****************************************************************************/

static ssize_t bravo_modem_mgr_get_modem_wake_state(struct device *dev, struct device_attribute *attr, char *buf)
{
    int modem_enable_state = GetGpioValue(BMM_GPIO_MODEM_ENABLE);
    int modem_power_state  = GetGpioValue(BMM_GPIO_MODEM_POWER);
    int modem_state        = (modem_enable_state & modem_power_state);

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Modem wake state: %s\n", ((0 == modem_state) ? "DISABLED" : "ENABLED") );

	return sprintf(buf, "%d\n", ((0 == modem_state) ? 0 : 1));
}

static ssize_t bravo_modem_mgr_set_modem_wake_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t retval = count;

    if (NULL == buf)
    {
        printk(KERN_ERR "bravo_modem_mgr: No modem wake state provided.\n");
        retval = -EINVAL;
    }
    else
    {
        /* We only care about the first char in the buffer */
        if ('0' == buf[0])
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Setting modem wake state to DISABLED...\n");
            bravo_modem_mgr_turn_modem_off();
        }
        else if ('1' == buf[0])
        {
            DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Setting modem wake state to ENABLED...\n");
            bravo_modem_mgr_turn_modem_on();
        }
        else
        {
            printk(KERN_ERR "bravo_modem_mgr: Invalid modem wake state specified: %s\n", buf);
            retval = -EINVAL;
        }
    }

	return retval;
}

static DEVICE_ATTR(modem_wake_state, S_IWUGO | S_IRUGO, bravo_modem_mgr_get_modem_wake_state, bravo_modem_mgr_set_modem_wake_state);

/* SysFS table entries */
static struct device_attribute* const g_bravo_modem_mgr_attributes[] =
{
    &dev_attr_modem_wake_state,
}; 


/*****************************************************************************
 * Power Management
 *****************************************************************************/

#ifdef CONFIG_PM
static int bravo_modem_mgr_suspend(struct platform_device *pdev, pm_message_t state)
{
    // Turn the modem 100% off.
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Suspending - Modem Power -> OFF, Modem Enable -> OFF\n");
    bravo_modem_mgr_turn_modem_off();
    return 0;
}

static int bravo_modem_mgr_resume(struct platform_device *pdef)
{
    // Even though we are resuming, we want to keep the modem disabled until RIL re-enables it.
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Resuming - Modem Power -> OFF, Modem Enable -> OFF\n");
    bravo_modem_mgr_turn_modem_off();
    return 0;
}
#else  /* CONFIG_PM */
#define bravo_modem_mgr_suspend NULL
#define bravo_modem_mgr_resume  NULL
#endif /* CONFIG_PM */


/*****************************************************************************
 * Module initialization/deinitialization
 *****************************************************************************/

static int __init bravo_modem_mgr_probe(struct platform_device *pdev)
{
    int err = 0;
    int i;

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Configuring Bravo Modem Manager\n");

    // Default Power-On State: POWER = OFF, ENABLE = OFF

    /*** MODEM POWER CONFIGURATION ***/

    s3c_gpio_setpull(BMM_GPIO_MODEM_POWER, S3C_GPIO_PULL_NONE);
    s3c_gpdat_setval(BMM_GPIO_MODEM_POWER, 0);
    s3c_gpio_cfgpin(BMM_GPIO_MODEM_POWER, S3C_GPIO_OUTPUT);

    /*** MODEM ACTIVE CONFIGURATION ***/

    s3c_gpio_setpull(BMM_GPIO_MODEM_ENABLE, S3C_GPIO_PULL_NONE);
    s3c_gpdat_setval(BMM_GPIO_MODEM_ENABLE, 0);
    s3c_gpio_cfgpin(BMM_GPIO_MODEM_ENABLE, S3C_GPIO_OUTPUT);

    if (bravo_modem_mgr_are_we_in_grace_period())
    {
        bravo_modem_mgr_turn_modem_on();
    }

    /* Configure SysFS entries */
    for (i = 0; i < ARRAY_SIZE(g_bravo_modem_mgr_attributes); i++)
    {
        err = device_create_file(&(pdev->dev), g_bravo_modem_mgr_attributes[i]);
        if (0 > err)
        {
            while (i--)
            {
                device_remove_file(&(pdev->dev), g_bravo_modem_mgr_attributes[i]);
            }
            printk(KERN_ERR "bravo_modem_mgr: ERROR - failed to register SYSFS\n");
            goto ERROR;
        }
    }

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Bravo Modem Manager configuration complete\n");

    return 0;

ERROR:
    return err;
}

static void bravo_modem_mgr_shutdown(struct platform_device *pdef)
{
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Shutting down Bravo Modem Manager\n");

    // Default Power-Off State: POWER = OFF, ENABLE = OFF

    s3c_gpdat_setval(BMM_GPIO_MODEM_ENABLE, 0);
    s3c_gpdat_setval(BMM_GPIO_MODEM_POWER,  0);

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Bravo Modem Manager shut down complete\n");
}

static int __devexit bravo_modem_mgr_remove(struct platform_device *pdev)
{
    int i;

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Removing Bravo Modem Manager\n");

    bravo_modem_mgr_shutdown(pdev);

    /* Remove SysFS entries */
    for (i = 0; i < ARRAY_SIZE(g_bravo_modem_mgr_attributes); i++)
    {
        device_remove_file(&(pdev->dev), g_bravo_modem_mgr_attributes[i]);
	}

    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Bravo Modem Manager removal complete\n");
    return 0;
}

static struct platform_driver g_bravo_modem_mgr_driver =
{
    .probe    = bravo_modem_mgr_probe,
    .remove   = __devexit_p(bravo_modem_mgr_remove),
    .shutdown = bravo_modem_mgr_shutdown,
    .suspend  = bravo_modem_mgr_suspend,
    .resume   = bravo_modem_mgr_resume,
    .driver   =
                {
                    .name   = "bravo_modem_mgr",
                    .bus    = &platform_bus_type,
                    .owner  = THIS_MODULE,
                },
};


/*****************************************************************************
 * Module init/cleanup
 *****************************************************************************/

static int __init bravo_modem_mgr_init(void)
{
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Initializing Bravo Modem Manager\n");

    return platform_driver_register(&g_bravo_modem_mgr_driver);
}

static void __exit bravo_modem_mgr_cleanup(void)
{
    DEBUGPRINT(KERN_INFO "bravo_modem_mgr: Cleaning Up Bravo Modem Manager\n");

    platform_driver_unregister(&g_bravo_modem_mgr_driver);
}

module_init(bravo_modem_mgr_init);
module_exit(bravo_modem_mgr_cleanup);


/*****************************************************************************
 * Final Administrivia
 *****************************************************************************/

MODULE_AUTHOR("Intrinsyc Software Inc., <support@intrinsyc.com>");
MODULE_DESCRIPTION("Bravo Modem Manager");
MODULE_LICENSE("GPL");

