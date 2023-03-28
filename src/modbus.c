/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modbus/modbus.h>

#include <zephyr/devicetree.h> // PWM
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

/*//#include "uart_async_adapter.h"// bluetooth

#include <zephyr/types.h>
//#include <zephyr/kernel.h>
//#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

//#include <zephyr/device.h>
//#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

//#include <dk_buttons_and_leds.h>

//#include <zephyr/settings/settings.h>

//#include <stdio.h>

//#include <zephyr/logging/log.h>*/
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>
#include "uart_async_adapter.h"
#include <soc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <zephyr/settings/settings.h>

#include <stdio.h> // BLuetooth

LOG_MODULE_REGISTER(modbus_app, LOG_LEVEL_INF);

static int client_iface;

const static struct modbus_iface_param client_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = 50000,
	.serial = {
		.baud = 115200,							  // 19200,
		.parity = UART_CFG_PARITY_EVEN,			  // UART_CFG_PARITY_NONE,
		.stop_bits_client = UART_CFG_STOP_BITS_1, // UART_CFG_STOP_BITS_2,
	},
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

int init_modbus_client(void)
{
	const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};

	client_iface = modbus_iface_get_by_name(iface_name);

	return modbus_init_client(client_iface, client_param);
}

typedef enum
{
/*	
	eAvance    = 0x8000,
	eArriere   = 0x4000,

	eAlarmRst  = 0x0080,
	eStop      = 0x0020,
	ePosition0 = 0x0010,
	eStart     = 0x0008,
	eArret     = 0x0000,
	*/

	eAvance    = 0x4000,
	eArriere   = 0x8000,

//	eAlarmRst  = 0x0080,
	eStop      = 0x0000,
	ePosition0 = 0x0800,
	eStart     = 0x0010,
	eArret     = 0x0020,
} eCmdMoteurModbus;

typedef union {
	int32_t u32;
	struct {
		uint16_t u16L;
		uint16_t u16H;
	};
} ui32_t;

int sendMoteur_avance(int32_t steps, uint32_t speed_mhz)
{
	ui32_t temp;
	union {
		struct
		{
			uint16_t steps_H;
			uint16_t steps_L;
			uint16_t speed_H;
			uint16_t speed_L;
		};
		uint16_t buff[4];
	}
	holding_reg;

	temp.u32 = steps;
	holding_reg.steps_H = temp.u16H;
	holding_reg.steps_L = temp.u16L;

	temp.u32 = speed_mhz;
	holding_reg.speed_H = temp.u16H;
	holding_reg.speed_L = temp.u16L;
	return modbus_write_holding_regs(client_iface, 1, 0x1802, (uint16_t *)&holding_reg, sizeof(holding_reg) / sizeof(uint16_t));
}

int sendMoteur_start(void)
{
	return  modbus_write_holding_reg(client_iface, 1, 0x007D, 0x0008);
}

int sendMoteur_stop(void)
{
	return  modbus_write_holding_reg(client_iface, 1, 0x007D, eStop);
}

int sendMoteur_position0(void)
{
	return  modbus_write_holding_reg(client_iface, 1, 0x007D, ePosition0);
}

int sendMoteur_arret(void) {
	return  modbus_write_holding_reg(client_iface, 1, 0x007D, eArret);
}

void main_modbus(void)
{
	uint16_t holding_reg[] = {0x0000, 0x03e8, 0x0000, 0x1388}; //{0x0000,0x0000,0x0000,0x0002,0x0006,0x7C28,0x0000,0x07D0,0x0000,0x05DC,0x0000,0x05DC,0x0000,0x038E,0x0000,0x0001};
	const uint8_t coil_qty = 3;
	uint8_t coil[1] = {0};
	const int32_t sleep = 2500;		 // 250
	int16_t start_address = 0x1802;	 // F10
	int16_t start_address1 = 0x007D; //{0x0000,0x1388,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}; commandez le moteur F06
	int16_t start_address2 = 0x1840; // Lirez les registers, addres du register et # de registers. F03
									 //	int16_t reg_val = 0x0008;
									 //	int16_t reg_val = 0x8000;
	int16_t num_regs = 0x0006;
	static uint8_t node = 1;
	int err;
	bool comande;
	eCmdMoteurModbus reg_val;
	// modbus
	init_modbus_client();
	k_msleep(sleep);

	if (0 != sendMoteur_stop()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(100);

	if (0 != sendMoteur_position0()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_arret() ){
			LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_avance(-1000, 1000)) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(100);
	if (0 != sendMoteur_start()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_stop()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_position0()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_avance(20000, 1000)) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(100);
	if (0 != sendMoteur_start()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(10000);

	if (0 != sendMoteur_stop()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	if (0 != sendMoteur_avance(1000, 1000)) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(100);
	if (0 != sendMoteur_start()) {
		LOG_ERR("FC16 failed");
	}
	k_msleep(2000);

	while(1) {
		k_msleep(1000);
	}

















	//	print_uart("Hello! I'm your echo bot.\r\n");

// 	err = modbus_write_holding_regs(client_iface, node, start_address, holding_reg, ARRAY_SIZE(holding_reg));
// 	if (err != 0) {
// 		LOG_ERR("FC16 failed");
// //		return;
// 	}

// 	k_msleep(sleep);

	/*err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
	if (err != 0) {
		LOG_ERR("FC06 failed");
//		return;
	}

	k_msleep(sleep);*/

	/*err = modbus_read_holding_regs(client_iface, node, start_address2, holding_reg, num_regs);
	if (err != 0) {
		LOG_ERR("FC03 failed with %d", err);
//		return;
	}*/

	LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg), "WR|RD 1holding register:");
	comande = 1;
	k_msleep(sleep);

	while (true)
	{
		uint16_t addr = 0;
#if 0
		err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC01 failed with %d", err);
//			return;
		}

		//LOG_INF("Coils state 0x%02x", coil[0]);

		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_write_coil(client_iface, node, addr++, true);
		if (err != 0) {
			LOG_ERR("FC05 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
		err = modbus_read_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC01 failed with %d", err);
//			return;
		}

		//LOG_INF("Coils state 0x%02x", coil[0]);

		coil[0] = 0;
		err = modbus_write_coils(client_iface, node, 0, coil, coil_qty);
		if (err != 0) {
			LOG_ERR("FC15 failed with %d", err);
//			return;
		}

		k_msleep(sleep);
#endif

		if (comande)
		{

			/*err = modbus_write_holding_regs(client_iface, node, start_address, holding_reg, ARRAY_SIZE(holding_reg));
			if (err != 0) {
				LOG_ERR("FC16 failed");
//				return;
			}*/

			// reg_val = eAlarmRst;
			// err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			// if (err != 0)
			// {
			// 	LOG_ERR("FC06 failed");
			// 	//				return;
			// }

			k_msleep(sleep);
			reg_val = ePosition0;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}
			k_msleep(sleep);
			
			k_msleep(999999);
		

			reg_val = eAvance;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}

			k_msleep(sleep);
			reg_val = eStop;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}

			k_msleep(sleep);
			reg_val = eArriere;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}

			k_msleep(sleep);
			reg_val = eStop;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}

			k_msleep(sleep);

			if (0 != sendMoteur_avance(500, 2000))
			{
				LOG_ERR("FC06 failed");
			}
			if (0 != sendMoteur_start())
			{
				LOG_ERR("FC06 failed");
			}

			k_msleep(10000);

			if (0 != sendMoteur_avance(600, 2000))
			{
				LOG_ERR("FC06 failed");
			}
			if (0 != sendMoteur_start())
			{
				LOG_ERR("FC06 failed");
			}

			k_msleep(sleep);


			k_msleep(20000);
			reg_val = eStop;
			err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			if (err != 0)
			{
				LOG_ERR("FC06 failed");
				//				return;
			}

			// reg_val = eStop;
			// err = modbus_write_holding_reg(client_iface, node, start_address1, reg_val);
			// if (err != 0)
			// {
			// 	LOG_ERR("FC06 failed");
			// }
			/*err = modbus_read_holding_regs(client_iface, node, start_address2, holding_reg, ARRAY_SIZE(holding_reg));
			if (err != 0) {
				LOG_ERR("FC03 failed with %d", err);
//				return;
			}*/

			// LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg),"WR|RD 4holding register:");

			// k_msleep(sleep);
			comande = 0;
			// pour PWM
			//			for (int num_led = 0; num_led < num_leds; num_led++) {
			//			run_led_test(led_pwm, num_led);
			//			}
			// the end PWM.
		}

#if (USB_VCP > 0)
		while (k_msgq_get(&uart_msgq, &tx_buf, K_SECONDS(1)) == 0)
		{
			print_uart("Echo: ");
			print_uart(tx_buf);
			print_uart("\r\n");
			// holding_reg[0] = strtoul(rx_buf[0],NULL,16);// + strtoul(rx_buf[1],NULL,16);

			/*start_address2 = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);
			num_regs = ((int)rx_buf[4]-48)*4096 + ((int)rx_buf[5]-48)*256 + ((int)rx_buf[6]-48)*16 + ((int)rx_buf[7]-48);//code F03*/

			reg_val = ((int)rx_buf[0] - 48) * 4096 + ((int)rx_buf[1] - 48) * 256 + ((int)rx_buf[2] - 48) * 16 + ((int)rx_buf[3] - 48); // F06

			/*start_address = ((int)rx_buf[0]-48)*4096 + ((int)rx_buf[1]-48)*256 + ((int)rx_buf[2]-48)*16 + ((int)rx_buf[3]-48);//Adrress.

			holding_reg[15]= ((int)rx_buf[64]-48)*4096 + ((int)rx_buf[65]-48)*256 + ((int)rx_buf[66]-48)*16 + ((int)rx_buf[67]-48);
			holding_reg[14] = ((int)rx_buf[60]-48)*4096 + ((int)rx_buf[61]-48)*256 + ((int)rx_buf[62]-48)*16 + ((int)rx_buf[63]-48);
			holding_reg[13]= ((int)rx_buf[56]-48)*4096 + ((int)rx_buf[57]-48)*256 + ((int)rx_buf[58]-48)*16 + ((int)rx_buf[59]-48);
			holding_reg[12] = ((int)rx_buf[52]-48)*4096 + ((int)rx_buf[53]-48)*256 + ((int)rx_buf[54]-48)*16 + ((int)rx_buf[55]-48);
			holding_reg[11]= ((int)rx_buf[48]-48)*4096 + ((int)rx_buf[49]-48)*256 + ((int)rx_buf[50]-48)*16 + ((int)rx_buf[51]-48);
			holding_reg[10] = ((int)rx_buf[44]-48)*4096 + ((int)rx_buf[45]-48)*256 + ((int)rx_buf[46]-48)*16 + ((int)rx_buf[47]-48);
			holding_reg[9]= ((int)rx_buf[40]-48)*4096 + ((int)rx_buf[41]-48)*256 + ((int)rx_buf[42]-48)*16 + ((int)rx_buf[43]-48);
			holding_reg[8] = ((int)rx_buf[36]-48)*4096 + ((int)rx_buf[37]-48)*256 + ((int)rx_buf[38]-48)*16 + ((int)rx_buf[39]-48);
			holding_reg[7]= ((int)rx_buf[32]-48)*4096 + ((int)rx_buf[33]-48)*256 + ((int)rx_buf[34]-48)*16 + ((int)rx_buf[35]-48);
			holding_reg[6] = ((int)rx_buf[28]-48)*4096 + ((int)rx_buf[29]-48)*256 + ((int)rx_buf[30]-48)*16 + ((int)rx_buf[31]-48);
			holding_reg[5]= ((int)rx_buf[24]-48)*4096 + ((int)rx_buf[25]-48)*256 + ((int)rx_buf[26]-48)*16 + ((int)rx_buf[27]-48);
			holding_reg[4] = ((int)rx_buf[20]-48)*4096 + ((int)rx_buf[21]-48)*256 + ((int)rx_buf[22]-48)*16 + ((int)rx_buf[23]-48);
			holding_reg[3]= ((int)rx_buf[16]-48)*4096 + ((int)rx_buf[17]-48)*256 + ((int)rx_buf[18]-48)*16 + ((int)rx_buf[19]-48);
			holding_reg[2] = ((int)rx_buf[12]-48)*4096 + ((int)rx_buf[13]-48)*256 + ((int)rx_buf[14]-48)*16 + ((int)rx_buf[15]-48);
			holding_reg[1]= ((int)rx_buf[8]-48)*4096 + ((int)rx_buf[9]-48)*256 + ((int)rx_buf[10]-48)*16 + ((int)rx_buf[11]-48);
			holding_reg[0] = ((int)rx_buf[4]-48)*4096 + ((int)rx_buf[5]-48)*256 + ((int)rx_buf[6]-48)*16 + ((int)rx_buf[7]-48);//F10*/

			// holding_reg[0] = ((int)rx_buf[0]-48)*1000 + ((int)rx_buf[1]-48)*100 + ((int)rx_buf[2]-48)*10 + ((int)rx_buf[3]-48);
			// holding_reg[0] = strtoul(rx_buf,NULL,16);
			// holding_reg[0] = ((uint16_t)tx_buf[0] << 8) | ((uint8_t)tx_buf[1]); //
			// holding_reg[1] = ((uint16_t)tx_buf[2] << 8) | ((uint8_t)tx_buf[3]); //
			// strtoul(rx_buf,2,10);// >> strtoul(rx_buf,3,10);
			comande = 1;
			LOG_HEXDUMP_INF(holding_reg, sizeof(holding_reg), "WR|RD 3holding register:");
			k_msleep(sleep);
		}
#endif //(USB_VCP > 0)
	}
}

/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

K_THREAD_DEFINE(main_modbus_id, STACKSIZE, main_modbus, NULL, NULL, NULL, PRIORITY, 0, 0);
