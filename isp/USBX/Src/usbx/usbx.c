/**
 ******************************************************************************
 * @file    usbx.c
 * @author  AIS Application Team
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#include <assert.h>
#include "usbx.h"
#include "stm32n6xx_hal.h"
#include "usb_desc.h"
#include "usb_desc_internal.h"
#include "ux_api.h"
#include "ux_dcd_stm32.h"
#include "ux_device_class_video.h"
#include "ux_device_class_cdc_acm.h"

static uint8_t usbx_mem_pool[USBX_MEM_SIZE] USBX_ALIGN_32;
#ifdef UVC_LIB_USE_DMA
static uint8_t usbx_mem_pool_uncached[USBX_MEM_SIZE] USBX_UNCACHED USBX_ALIGN_32;
#endif
static uint8_t usb_desc_fs[USBX_MAX_CONF_LEN];
static uint8_t usb_desc_hs[USBX_MAX_CONF_LEN];
static uint8_t usb_dev_strings[USBX_MAX_STRING_LEN];
static uint8_t usb_dev_langid[USBX_MAX_LANGID_LEN];
extern PCD_HandleTypeDef usbx_pcd_handle;

#define RX_PACKET_SIZE 512
/* Use read global buffer instead of local (memory issue) */
unsigned char rx_buffer[RX_PACKET_SIZE];

static UINT usbx_device_cb(ULONG cb_evt)
{
  //printf("%s evt %d\n", __func__, (int) cb_evt);

#if defined(UX_DEVICE_STANDALONE)
  if (cb_evt == UX_DCD_STM32_SOF_RECEIVED) {
    ux_system_tasks_run();
    ux_system_tasks_run();
  }
#endif

  return 0;
}

static int usbx_extract_string(uint8_t langid[2], int index, uint8_t *string_desc, uint8_t *p_dst, int dst_len)
{
  int str_len = (string_desc[0] - 2) / 2;
  int i;

  if (dst_len < str_len + 4)
    return -1;

  p_dst[0] = langid[0];
  p_dst[1] = langid[1];
  p_dst[2] = index;
  p_dst[3] = str_len;
  for (i = 0; i < str_len; i++)
    p_dst[4 + i] = string_desc[2 + 2 * i];

  return str_len + 4;
}

static int usbx_build_dev_strings(uint8_t langid[2], uint8_t *p_dst, int dst_len)
{
  uint8_t string_desc[128];
  int res = 0;
  int len;

  len = usb_get_manufacturer_string_desc(string_desc, sizeof(string_desc));
  if (len < 0)
    return len;
  res += usbx_extract_string(langid, 1, string_desc, &p_dst[res], dst_len - res);
  if (res < 0)
    return 0;

  len = usb_get_product_string_desc(string_desc, sizeof(string_desc));
  if (len < 0)
    return len;
  res += usbx_extract_string(langid, 2, string_desc, &p_dst[res], dst_len - res);
  if (res < 0)
    return 0;

  len = usb_get_serial_string_desc(string_desc, sizeof(string_desc));
  if (len < 0)
    return len;
  res += usbx_extract_string(langid, 3, string_desc, &p_dst[res], dst_len - res);
  if (res < 0)
    return 0;

  return res;
}

static UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  printf("%s %p\n", __func__, cdc_acm_instance);
  cdc_acm = cdc_acm_instance;
}

VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  cdc_acm = NULL;
  printf("%s %p\n", __func__, cdc_acm_instance);
}


VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER line_coding;
  UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER line_state;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;
  ULONG request;
  int ret;

  device = &_ux_system_slave -> ux_system_slave_device;
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;
  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request) {
  case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING:
    ret = ux_device_class_cdc_acm_ioctl(cdc_acm_instance, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING, &line_coding);
    assert(ret == 0);
    printf("Set %d bps\n", (int) line_coding.ux_slave_class_cdc_acm_parameter_baudrate);
    break;
  case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE:
    ret = ux_device_class_cdc_acm_ioctl(cdc_acm_instance, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE, &line_state);
    assert(ret == 0);
    printf("Set line state rts = %d / dtr = %d\n", (int) line_state.ux_slave_class_cdc_acm_parameter_rts,
                                                   (int) line_state.ux_slave_class_cdc_acm_parameter_dtr);
    break;
  default:
    printf("Unsupported request 0x%08x (%d)\n", (int)request, (int)request);
  }
}

uint32_t usbx_read(uint8_t* payload)
{
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;
  ULONG rx_len = 0;
  int ret;


  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
    return 0;
  if (!cdc_acm)
    return 0;

  ret = ux_device_class_cdc_acm_read_run(cdc_acm, rx_buffer, RX_PACKET_SIZE, &rx_len);
  if(ret==UX_STATE_NEXT && rx_len > 0){
    memcpy(payload, rx_buffer, rx_len);
    return rx_len;
  } 
  else 
  {
    return 0;
  }
}

void usbx_write(unsigned char *msg, uint32_t len)
{
  UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;
  UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER line_state;
  ULONG len_send;
  int ret;

  if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
    return ;

  if (!cdc_acm)
    return ;

  ret = ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE, &line_state);
  assert(ret == 0);
  if (!line_state.ux_slave_class_cdc_acm_parameter_dtr)
    return ;

  ret = ux_device_class_cdc_acm_write_run(cdc_acm, msg, len, &len_send);
  while(ret == UX_STATE_WAIT){
    /* If the CDC is busy we cannot send data.
    Wait for the CDC state to change and retry */
    ret = ux_device_class_cdc_acm_write_run(cdc_acm, msg, len, &len_send);
  }
  assert(ret == UX_STATE_NEXT); // UX_STATE_NEXT state is the success state
}

int usbx_init(PCD_HandleTypeDef *pcd_handle, PCD_TypeDef *pcd_instance)
{
  usb_desc_conf desc_conf = { 0 };
  uint8_t lang_string_desc[4];
  int usb_dev_strings_len;
  int usb_dev_langid_len;
  int usb_desc_hs_len;
  int usb_desc_fs_len;
  int len;
  int ret;

#ifdef UVC_LIB_USE_DMA
  ret = ux_system_initialize(usbx_mem_pool, USBX_MEM_SIZE, usbx_mem_pool_uncached, USBX_MEM_SIZE);
#else
  ret = ux_system_initialize(usbx_mem_pool, USBX_MEM_SIZE, UX_NULL, 0);
#endif
  if (ret)
    return ret;

  /* Build High Speed configuration descriptor */
  desc_conf.is_hs = 1;
  usb_desc_hs_len = usb_get_device_desc(usb_desc_hs, sizeof(usb_desc_hs), 1, 2, 3);
  assert(usb_desc_hs_len > 0);

  len = usb_get_configuration_desc(&usb_desc_hs[usb_desc_hs_len], sizeof(usb_desc_hs) - usb_desc_hs_len, &desc_conf);
  assert(len > 0);
  usb_desc_hs_len += len;

  /* Build Full Speed configuration descriptor */
  desc_conf.is_hs = 0;
  usb_desc_fs_len = usb_get_device_desc(usb_desc_fs, sizeof(usb_desc_fs), 1, 2, 3);
  assert(usb_desc_fs_len > 0);

  len = usb_get_configuration_desc(&usb_desc_fs[usb_desc_fs_len], sizeof(usb_desc_fs) - usb_desc_fs_len, &desc_conf);
  assert(len > 0);
  usb_desc_fs_len += len;

  len = usb_get_lang_string_desc(lang_string_desc, sizeof(lang_string_desc));
  assert(len == sizeof(lang_string_desc));
  usb_dev_langid[0] = lang_string_desc[2];
  usb_dev_langid[1] = lang_string_desc[3];
  usb_dev_langid_len = 2;

  usb_dev_strings_len = usbx_build_dev_strings(usb_dev_langid, usb_dev_strings, sizeof(usb_dev_strings));
  assert(usb_dev_strings_len > 0);

  ret = ux_device_stack_initialize(usb_desc_hs, usb_desc_hs_len,
		  	  	  	  	  	  	   usb_desc_fs, usb_desc_fs_len,
                                   usb_dev_strings, usb_dev_strings_len,
                                   usb_dev_langid, usb_dev_langid_len, usbx_device_cb);
  if (ret)
    return ret;

  UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm_parameter;
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = USBD_CDC_ACM_Activate;
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = USBD_CDC_ACM_Deactivate;
  cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change    = USBD_CDC_ACM_ParameterChange;

  /* Register first CDC instance corresponding to Interface 0 */
  ret = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry, 1, 0, &cdc_acm_parameter);
  assert(ret == 0);

  return ux_dcd_stm32_initialize((ULONG)pcd_instance, (ULONG)pcd_handle);
}

#if defined(UX_DEVICE_STANDALONE)
ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
  ALIGN_TYPE ret = __get_PRIMASK();

  __disable_irq();

  return ret;
}

VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
  if (!flags)
    __enable_irq();
}
#endif
