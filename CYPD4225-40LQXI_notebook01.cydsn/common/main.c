/**
 * @file main.c
 *
 * @brief @{Main source file for CCG firmware implementation.@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <project.h>
#include <flash_config.h>
#include <system.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <swap.h>
#include <vdm.h>
#include <pdo.h>
#include <app.h>
#include <hal_ccgx.h>
#include <timer.h>
#include <hpi.h>
#include <hpd.h>
#include <boot.h>
#include <flash.h>
#include <status.h>
#include <ccgx_version.h>
#include <app_version.h>
#include <utils.h>
#include <gpio.h>

#if AR_SLAVE_IF_ENABLE
#include <ar_slave.h>
#endif /* AR_SLAVE_IF_ENABLE */

#define PORT_START_IDX   (TYPEC_PORT_0_IDX)

/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
__attribute__ ((section(".customer_region"), used))
const uint32_t customer_info[8] = {0x00};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[5] = {0};

#if CCG_FIRMWARE_APP_ONLY
/*
   Provide a variable required by the HPI library. The real variable only
   exists if the boot-loadable component is included.
 */
volatile uint32_t cyBtldrRunType = 0;
#endif

/* Solution PD event handler */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
#if (CCG_HPI_PD_ENABLE == 1)
    /* Pass the callback to HPI */
    hpi_pd_event_handler(port, evt, data);
#else /* (CCG_HPI_PD_ENABLE == 0) */
    (void)port;
    (void)evt;
    (void)data;
    /* Do nothing. */
#endif /* (CCG_HPI_PD_ENABLE) */
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const app_cbk_t app_callback =
{
    app_event_handler,
    psrc_set_voltage,
    psrc_set_current,
    psrc_enable,
    psrc_disable,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_rdo,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm
};

app_cbk_t* app_get_callback_ptr(uint8_t port)
{
    (void)port;
    /* Solution callback pointer is same for all ports */
    return ((app_cbk_t *)(&app_callback));
}

#if CCG_HPI_ENABLE
static void
update_hpi_regs (
        void)
{
#if CCG_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint8_t ver_invalid[8] = {0};

    /* Flash access is not allowed. */
    flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08;              /* FW2 is not valid. */
    hpi_set_mode_regs (mode, reason);
    hpi_update_versions (ver_invalid, (uint8_t *)&base_version, ver_invalid);
    hpi_update_fw_locations (0, CCG_LAST_FLASH_ROW_NUM);

#else /* !CCG_FIRMWARE_APP_ONLY */

    uint8_t mode, reason = 0x00;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0};

    /* Set mode variables and flash access limits based on the active firmware. */
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        mode = 0x81 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW2 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x08;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_IMG1_LAST_FLASH_ROW_NUM + 1, CCG_IMG2_LAST_FLASH_ROW_NUM,
                CCG_IMG2_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }
    else
    {
        mode = 0x82 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW1 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x04;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, gl_img2_fw_metadata->boot_last_row,
                CCG_IMG1_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }

    hpi_set_mode_regs (mode, reason);

    /* Calculate the version address from the firmware metadata. */
    if ((reason & 0x04) == 0)
    {
        fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
        fw1_ver = (((uint32_t)fw1_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->boot_last_row + 1;
    }
    else
    {
        fw1_ver = (uint32_t)ver_invalid;
        fw1_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    if ((reason & 0x08) == 0)
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = (((uint32_t)fw2_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->boot_last_row + 1;
    }
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    /* Update version information in the HPI registers. */
    hpi_update_versions (
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

    /* Update firmware location registers. */
    hpi_update_fw_locations (fw1_loc, fw2_loc);

#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* CCG_HPI_ENABLE */

#if APP_FW_LED_ENABLE

#if DP_BRIDGE_ENABLE
#include <dp_sid.h>

/*
 * DP bridge diagnostic blink code on the FW LED, repeated continuously:
 *   1 long flash (1 s)           -> start marker
 *   N short blinks               -> LCD side (port 1) stage, always >= 1
 *   2 s pause
 *   M short blinks               -> host side (port 0) stage, always >= 1
 *   3 s pause
 *
 * LCD side (port 1):                     Host side (port 0):
 *   1: nothing attached                    1: nothing attached
 *   2: Type-C attached, no PD contract     2: Type-C attached, no PD contract
 *   3: PD contract, CCG still UFP          3: PD contract, but CCG is DFP (wrong)
 *   4: CCG is DFP (DR_SWAP done)           4: PD contract, CCG is UFP
 *   5: DP mode entered on the LCD          5: host entered DP mode
 *   6: LCD configured for DP (C/D)         6: host configured DP (C/D)
 *   7: LCD reports HPD high                7: HPD high reported to the host
 */
#define LED_SHORT_ON_MS         (250u)
#define LED_SHORT_OFF_MS        (350u)
#define LED_MARKER_ON_MS        (1000u)
#define LED_GROUP_GAP_MS        (2000u)
#define LED_END_GAP_MS          (3000u)

void led_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    static uint8_t phase = 0;       /* 0: marker, 1: LCD blinks, 2: host blinks. */
    static uint8_t count = 0;       /* Blinks left in the current group. */
    static bool    led_on = false;
    static uint8_t host_stage = 0;
    uint8_t  lcd_stage;
    uint16_t period;

    (void)port;
    (void)id;

    if (led_on)
    {
        /* Turn the LED off and decide how long to stay off. */
        led_on = false;
        gpio_set_value (FW_LED_GPIO_PORT_PIN, false);

        if (phase == 0)
        {
            /* Marker done: latch the stages and start the LCD group. */
            dp_bridge_get_stage (&lcd_stage, &host_stage);
            phase  = 1;
            count  = lcd_stage;
            period = LED_GROUP_GAP_MS;
        }
        else
        {
            count--;
            period = LED_SHORT_OFF_MS;
        }

        /* Group finished: go to the next group (or back to the marker). */
        if (count == 0)
        {
            if (phase == 1)
            {
                phase  = 2;
                count  = host_stage;
                period = LED_GROUP_GAP_MS;
            }
            if ((phase == 2) && (count == 0))
            {
                phase  = 0;
                period = LED_END_GAP_MS;
            }
        }
    }
    else
    {
        /* Turn the LED on for the marker or a short blink. */
        led_on = true;
        gpio_set_value (FW_LED_GPIO_PORT_PIN, true);
        period = (phase == 0) ? LED_MARKER_ON_MS : LED_SHORT_ON_MS;
    }

    timer_start (0, LED_TIMER_ID, period, led_timer_cb);
}

#else

/* Blink the LED every LED_TIMER_PERIOD ms. This serves as an indication of the firmware running. */
void led_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    (void)port;
    (void)id;

    gpio_set_value (FW_LED_GPIO_PORT_PIN, !(gpio_read_value (FW_LED_GPIO_PORT_PIN)));
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);
}

#endif /* DP_BRIDGE_ENABLE */

#endif /* APP_FW_LED_ENABLE */

int main()
{
    uint32_t conf_addr;
    uint8_t  port;

#if CCG_HPI_ENABLE
    uint8_t i;
#endif /* CCG_HPI_ENABLE */

    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x= 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

#if CCG_FIRMWARE_APP_ONLY
    sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
#else /* !CCG_FIRMWARE_APP_ONLY */
    if ((uint32_t)&base_version < CCG_FW1_CONFTABLE_MAX_ADDR)
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
    }
    else
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_2);

#ifndef CCG_PSEUDO_METADATA_DISABLE
        /* Determine pseudo metadata table address for Image 1 from the BOOT LAST row
         * parameter of Image 2. */
        gl_img1_fw_pseudo_metadata = (sys_fw_metadata_t *)((gl_img2_fw_metadata->boot_last_row
                << CCG_FLASH_ROW_SHIFT_NUM) + (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE));
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    }
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Validate the signature and checksum of the configuration table. */
    conf_addr = (uint32_t)get_pd_config ();
    if (boot_validate_configtable ((uint8_t *)conf_addr) != CCG_STAT_SUCCESS)
    {
#if CCG_FIRMWARE_APP_ONLY
        /* Can't do anything if config table is not valid. */
        while (1);
#else /* !CCG_FIRMWARE_APP_ONLY */
        /* Erase the firmware metadata so that this binary stops loading. */
        if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
        {
            flash_row_clear (CCG_IMG1_METADATA_ROW_NUM);
        }
        else
        {
            flash_row_clear (CCG_IMG2_METADATA_ROW_NUM);
        }

        /* Now reset the device. */
        CySoftwareReset ();
#endif /* CCG_FIRMWARE_APP_ONLY */
    }

#ifndef CCG_PSEUDO_METADATA_DISABLE
    /* Check if Other FW image is pending to be validated. If other image is valid,
     * device will undergo reset and then jump to the other image. */
    boot_check_for_valid_fw ();
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    /*
     * Configure the FET type usage model. This should be modified only if
     * the board supports a different configuration. Wrong configuration
     * shall result in damage of the boards and attached devices. Leave it
     * to default configuration for working with standard cypress design.
     *
     * For CCG4, the default for PCTRL is active high and CCTRL is active low.
     */
    pd_hal_set_fet_drive(PD_FET_DR_ACTIVE_HIGH, PD_FET_DR_ACTIVE_LOW);

    system_init();

    /* Timer INIT has to be done first. */
    timer_init();

    /* Enable global interrupts */
    CyGlobalIntEnable;

#if AR_SLAVE_IF_ENABLE
    /* Initialize the Alpine-Ridge slave interface. */
    ar_slave_init (AR_SLAVE_SCB_INDEX, 0);
#endif /* AR_SLAVE_IF_ENABLE */

#if CCG_HPI_ENABLE

#if CCG_FIRMWARE_APP_ONLY
    /* Configure HPI for no-boot operation. */
    hpi_set_no_boot_mode (true);
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Initialize the HPI interface. */
    hpi_init (HPI_SCB_INDEX);

    /* Update HPI registers with mode and version information. */
    update_hpi_regs ();

#endif /* CCG_HPI_ENABLE */

    /* Perform application level initialization. */
    app_init();

    /* Initialize the Device Policy Manager for each PD port on the device. */
    for(port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
        dpm_init(port, app_get_callback_ptr(port));
    }

#if CCG_HPI_ENABLE

    /* Send a reset complete event to the EC. */
    hpi_send_fw_ready_event ();

    /* Wait until EC ready event has been received or 100 ms has elapsed. */
    for (i = 0; i < 100; i ++)
    {
        hpi_task ();
        if (hpi_is_ec_ready ())
            break;

        CyDelay (1);
    }

#endif /* CCG_HPI_ENABLE */

    for (port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_HPI_ENABLE
        /* Start the DPM for the port only if it is enabled at the HPI level. It is possible that the port
           got disabled before we got here.
         */
        if ((hpi_get_port_enable () & (1 << port)) != 0)
#endif /* CCG_HPI_ENABLE */
        {
            dpm_start(port);
        }
    }

#if APP_FW_LED_ENABLE

    /* Configure the LED control GPIO as an output. */
    gpio_hsiom_set_config (FW_LED_GPIO_PORT_PIN, HSIOM_MODE_GPIO, GPIO_DM_STRONG, true);
    /* Start a timer that will blink the FW ACTIVE LED, if required. */
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);

#endif /* APP_FW_LED_ENABLE */

    while(1)
    {
        /* Handle the device policy tasks for each PD port. */
        for(port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            dpm_task(port);
            app_task(port);

#if CCG_HPI_ENABLE
            /* Handle any pending HPI commands. */
            hpi_task ();
#endif /* CCG_HPI_ENABLE */
        }

#if SYS_DEEPSLEEP_ENABLE

        /* If possible, enter sleep mode for power saving. */
        system_sleep();

#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* End of file */
