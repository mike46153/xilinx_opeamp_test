/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This is a sample demonstration application that showcases usage of rpmsg
This application is meant to run on the remote CPU running baremetal code.
This application echoes back data that was sent to it by the master core. */

#include "xil_printf.h"
#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include "platform_info.h"
#include "rpmsg-echo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "xgpiops.h"

#define SHUTDOWN_MSG	0xEF56A55A

#define LPRINTF(fmt, ...) xil_printf("%s():%u " fmt, __func__, __LINE__, ##__VA_ARGS__)
#define LPERROR(fmt, ...) LPRINTF("ERROR: " fmt, ##__VA_ARGS__)

static struct rpmsg_endpoint lept;
static int shutdown_req = 0;
static TaskHandle_t comm_task;

#define RPMSG_HEADER_LEN 16
//#define MAX_RPMSG_BUFF_SIZE (512 - RPMSG_HEADER_LEN)
#define MAX_RPMSG_BUFF_SIZE (8192 - RPMSG_HEADER_LEN)
#define PAYLOAD_MIN_SIZE	1
#define PAYLOAD_MAX_SIZE	(MAX_RPMSG_BUFF_SIZE - 24)
#define NUM_PAYLOADS		(PAYLOAD_MAX_SIZE/PAYLOAD_MIN_SIZE)

XGpioPs Gpio;
#define TOP_COE_AXI_SLAVE_BASEADDR			XPAR_TOP_CORE_SLAVE_V4_0_S00_AXI_BASEADDR
#define RXFnt_test_GPIO	     	0x0003F318
#define BIT_0  (0x1<<0 )
#define BIT_1  (0x1<<1 )
#define BIT_2  (0x1<<2 )
#define BIT_3  (0x1<<3 )
uint32_t openamp_data_check;

struct _payload {
	unsigned long num;
	unsigned long size;
	char data[];
};

int GpioInput(int pin, u32 *DataRead)
{
	/* Set the direction for the specified pin to be input. */
	XGpioPs_SetDirectionPin(&Gpio, pin, 0x0);
	/* Read the state of the data so that it can be  verified. */
	*DataRead = XGpioPs_ReadPin(&Gpio, pin);

	return XST_SUCCESS;
}
/*-----------------------------------------------------------------------------*
 *  Xil_Out32_AND_Not_IO_LOW
 *-----------------------------------------------------------------------------*/
void Xil_Out32_AND_Not(UINTPTR address, u32 value){

	u32 temp;

	temp = Xil_In32(address);

	temp &= ~value;

	Xil_Out32(address,temp);

}
/*-----------------------------------------------------------------------------*
 *  Xil_Out32_OR_IO_high
 *-----------------------------------------------------------------------------*/
void Xil_Out32_OR(UINTPTR address, u32 value){

	u32 temp;

	temp = Xil_In32(address);

	temp |= value;

	Xil_Out32(address,temp);

}
/*-----------------------------------------------------------------------------*
 *  RPMSG endpoint callbacks
 *-----------------------------------------------------------------------------*/
static int rpmsg_endpoint_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
				 uint32_t src, void *priv)
{
	(void)priv;
	(void)src;
	openamp_data_check = 0xA5;

	/* On reception of a shutdown we signal the application to terminate */
	if ((*(unsigned int *)data) == SHUTDOWN_MSG) {
		ML_INFO("shutdown message is received.\r\n");
		shutdown_req = 1;
		return RPMSG_SUCCESS;
	}
	//xil_printf("data addr: 0x%p\n\r", data);
	Xil_Out32_OR(TOP_COE_AXI_SLAVE_BASEADDR+RXFnt_test_GPIO, BIT_0);
#if 1
	for(int i = 2 * sizeof(unsigned long) + RPMSG_HEADER_LEN; i<PAYLOAD_MAX_SIZE; i++){
		if(openamp_data_check != *((char *)data + i)){
			ML_ERR("Data Received Incorrect\r\n");
		}
	}
#endif
	Xil_Out32_AND_Not(TOP_COE_AXI_SLAVE_BASEADDR+RXFnt_test_GPIO, BIT_0);
	/* Send data back to master */
	//if (rpmsg_send(ept, data, len) < 0)
	if (rpmsg_trysend(ept, data, len) < 0)
	{
		ML_ERR("rpmsg_send failed\r\n");
	}

	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint *ept)
{
	(void)ept;
	ML_INFO("unexpected Remote endpoint destroy\r\n");
	shutdown_req = 1;
}

/*-----------------------------------------------------------------------------*
 *  Application
 *-----------------------------------------------------------------------------*/
int app(struct rpmsg_device *rdev, void *priv)
{
	int ret;

	/* Initialize RPMSG framework */
	ML_INFO("Try to create rpmsg endpoint.\r\n");

	ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME,
				RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
				rpmsg_endpoint_cb,
				rpmsg_service_unbind);
	if (ret) {
		ML_ERR("Failed to create endpoint.\r\n");
		return -1;
	}

	ML_INFO("Successfully created rpmsg endpoint.\r\n");
	while(1) {
		platform_poll(priv);
		/* we got a shutdown request, exit */
		if (shutdown_req) {
			break;
		}
	}
	ML_DBG("out of platform_poll loop\r\n");
	rpmsg_destroy_ept(&lept);

	return 0;
}

/*-----------------------------------------------------------------------------*
 *  Processing Task
 *-----------------------------------------------------------------------------*/
static void processing(void *unused_arg)
{
	void *platform;
	struct rpmsg_device *rpdev;

	/* can't use ML_INFO, metal_log setup is in init_system */
	LPRINTF("Starting application...\r\n");
	/* Initialize platform */
	if (platform_init(0, NULL, &platform)) {
		LPERROR("Failed to initialize platform.\r\n");
	} else {
		rpdev = platform_create_rpmsg_vdev(platform, 0,
						   VIRTIO_DEV_SLAVE,
						   NULL, NULL);
		if (!rpdev){
			ML_ERR("Failed to create rpmsg virtio device.\r\n");
		} else {
			app(rpdev, platform);
			platform_release_rpmsg_vdev(rpdev, platform);
		}
	}

	ML_INFO("Stopping application...\r\n");
	platform_cleanup(platform);

	/* Terminate this task */
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------------------------*
 *  Application entry point
 *-----------------------------------------------------------------------------*/
int main(int ac, char **av)
{
	BaseType_t stat;

	init_9528();
	init_9375();
	Xil_Out32(0x9c600000, 0x9);
	printf("9375 init complete\n\r");

	/* Create the tasks */
	stat = xTaskCreate(processing, ( const char * ) "HW2",
				1024, NULL, 2, &comm_task);
	if (stat != pdPASS) {
		LPERROR("cannot create task\r\n");
	} else {
		/* Start running FreeRTOS tasks */
		vTaskStartScheduler();
	}

	/* Will not get here, unless a call is made to vTaskEndScheduler() */
	while (1) ;

	/* suppress compilation warnings*/
	return 0;
}
