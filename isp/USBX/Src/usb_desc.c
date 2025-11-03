/**
 ******************************************************************************
 * @file    usb_desc.c
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

#include "usb_desc.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "stm32n6xx_hal.h"

#include "usbx_conf.h"
#include "usbx_codes.h"
#include "usb_desc_internal.h"

#define container_of(ptr, type, member) (type *) ((unsigned char *)ptr - offsetof(type,member))
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

#define DEV_MANUFACTURER_STRING      "STMicroelectronics"
#define DEV_PRODUCT_STRING           "STM32 uvc"

static void gen_serial_int_to_ascii(uint32_t value, char *p_buf, uint8_t len)
{
  uint8_t idx = 0U;

  for (idx = 0U ; idx < len ; idx ++)
  {
    if (((value >> 28)) < 0xAU)
      p_buf[idx] = (value >> 28) + '0';
    else
      p_buf[idx] = (value >> 28) + 'A' - 10U;
    value = value << 4;
  }
}

static void gen_serial(char serial[13])
{
  uint32_t deviceserial0;
  uint32_t deviceserial1;
  uint32_t deviceserial2;

  deviceserial0 = HAL_GetUIDw0();
  deviceserial1 = HAL_GetUIDw1();
  deviceserial2 = HAL_GetUIDw2();
  deviceserial0 += deviceserial2;

  gen_serial_int_to_ascii(deviceserial0, &serial[0], 8);
  gen_serial_int_to_ascii(deviceserial1, &serial[8], 4);
  serial[12] = '\0';
}

static int usb_get_string_desc(void *p_dst, int dst_len, char *name)
{
  int desc_len = 2 + strlen(name) * 2;
  uint8_t *dst = p_dst;

  if (dst_len < desc_len)
    return -1;

  *dst++ = desc_len; /* bLength = desc_len */
  *dst++ = 0x03; /* string descriptor */

  while (*name) {
    *dst++ = *name++;
    *dst++ = 0;
  }

  return dst_len;
}

static int get_children_len(struct usb_desc_head *child)
{
  int total_len = 0;

  while (child) {
    total_len += child->bLength + get_children_len(child->children);
    child = child->next_child;
  }

  return total_len;
}

static void append_as_child(struct usb_desc_head *parent, struct usb_desc_head *child)
{
  struct usb_desc_head *prev_child_head = parent->children;

  parent->children = child;
  child->next_child = prev_child_head;
}

static int gen_default_desc(struct usb_desc_head *head, uint8_t *p_dst, int dst_len)
{
  if (head->bLength > dst_len)
    return -1;

  memcpy(p_dst, head->raw, head->bLength);
  if (head->next) {
    assert(head->next->gen);
    return head->next->gen(head->next, p_dst + head->bLength, dst_len - head->bLength);
  } else {
    return dst_len - head->bLength;
  }
}

static void update(struct usb_desc_head *head)
{
  while (head) {
    if (head->update)
      head->update(head);
    head = head->next;
  }
}

static int generate(struct usb_desc_head *head, uint8_t *p_dst, int dst_len)
{
  int ret;

  ret = head->gen(head, p_dst, dst_len);

  return ret < 0 ? ret : dst_len - ret;
}


static void build_dev_desc(struct usb_dev_desc *desc, struct usb_desc_head *next, usb_desc_conf *p_conf,
                           int idx_manufacturer, int idx_product, int idx_serial)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x01;
  desc->raw.bcdUSB = 0x0200;
  desc->raw.bDeviceClass = 0x02;
  desc->raw.bDeviceSubClass = 0x00;
  desc->raw.bDeviceProtocol = 0x00;
  desc->raw.bMaxPacketSize = 64;
  desc->raw.idVendor = 0x0483;
  desc->raw.idProduct = 0x5740;
  desc->raw.bcdDevice = 0x0100;
  desc->raw.iManufacturer = idx_manufacturer;
  desc->raw.iProduct = idx_product;
  desc->raw.iSerialNumber = idx_serial;
  desc->raw.bNumConfigurations = 1;
}

static void update_usb_conf_desc(struct usb_desc_head *head)
{
  struct usb_conf_desc *desc = container_of(head, struct usb_conf_desc, head);

  desc->raw.wTotalLength += get_children_len(desc->head.children);
}

static void build_usb_conf_desc(struct usb_conf_desc *desc, struct usb_desc_head *next)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.update = update_usb_conf_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x02;
  desc->raw.wTotalLength = desc->raw.bLength;
  desc->raw.bNumInterfaces = 2;
  desc->raw.bConfigurationValue = 1;
  desc->raw.iConfiguration = 0;
  desc->raw.bmAttributes = 0xc0; /* self powered */
  desc->raw.bMaxPower = 50; /* 100 ma */
}

static void build_std_itf_desc(struct std_itf_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent,
                               int ep_nb, int itf_nf, int class, int subclass, int protocol)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x04;
  desc->raw.bInterfaceNumber = itf_nf;
  desc->raw.bAlternateSetting = 0;
  desc->raw.bNumEndpoints = ep_nb;
  desc->raw.bInterfaceClass = class; /* Communications Interface Class */
  desc->raw.bInterfaceSubClass = subclass; /* Abstract Control Model */
  desc->raw.bInterfaceProtocol = protocol; /* AT Commands: V.250 etc */
  desc->raw.iInterface = 0;

  append_as_child(parent, &desc->head);
}

static void build_cdc_hdr_func_desc(struct cdc_hdr_func_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = 0x00; /* Header Functional Descriptor */
  desc->raw.bcdCDC = 0x0110; /* CDC 1.1 */

  append_as_child(parent, &desc->head);
}

static void build_cdc_acm_func_desc(struct cdc_acm_func_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = 0x02; /* Abstract Control Management Functional Descriptor */
  desc->raw.bmCapabilities = 0x0e; //orig: 0x0e

  append_as_child(parent, &desc->head);
}

static void build_cdc_union_func_desc(struct cdc_union_func_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent,
                                      int ctrl_itf, int data_itf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = 0x06; /* Union Functional Descriptor */
  desc->raw.bControlInterface = ctrl_itf;
  desc->raw.bSubordinateInterface0 = data_itf;

  append_as_child(parent, &desc->head);
}

static void build_cdc_call_mgt_func_desc(struct cdc_call_mgt_func_desc *desc, struct usb_desc_head *next,
                                         struct usb_desc_head *parent, int data_itf)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = CS_INTERFACE;
  desc->raw.bDescriptorSubType = 0x01; /* Call Management Functional Descriptor */
  desc->raw.bmCapabilities = 0x00;
  desc->raw.bDataInterface = data_itf;

  append_as_child(parent, &desc->head);
}

static void build_std_ep_desc(struct std_ep_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent,
                              int addr, int attribute, int packet_size, int interval)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 5;
  desc->raw.bEndpointAddress = addr;
  desc->raw.bmAttributes = attribute;
  desc->raw.wMaxPacketSize = packet_size;
  desc->raw.bInterval = interval;

  append_as_child(parent, &desc->head);
}

static void build_std_iad_desc(struct std_iad_desc *desc, struct usb_desc_head *next, struct usb_desc_head *parent)
{
  desc->head.bLength = sizeof(desc->raw);
  desc->head.raw = (uint8_t *) &desc->raw;
  desc->head.gen = gen_default_desc;
  desc->head.next = next;
  desc->raw.bLength = sizeof(desc->raw);
  desc->raw.bDescriptorType = 0x0b;
  desc->raw.bFirstInterface = 0; // orig: 2
  desc->raw.bInterfaceCount = 2;
  desc->raw.bFunctionClass = 0x02; /* Communications Interface Class */
  desc->raw.bFunctionSubClass = 0x02; /* Abstract Control Mode */
  desc->raw.bFunctionProtocol = 0x00; // orig: 0x00
  desc->raw.iFunction = 0;

  append_as_child(parent, &desc->head);
}

int usb_get_device_desc(void *p_dst, int dst_len, int idx_manufacturer, int idx_product, int idx_serial)
{
  struct usb_dev_desc dev_desc = { 0 };

  build_dev_desc(&dev_desc, NULL, NULL, idx_manufacturer, idx_product, idx_serial);
  update(&dev_desc.head);

  return generate(&dev_desc.head, p_dst, dst_len);
}

int usb_get_device_qualifier_desc(void *p_dst, int dst_len)
{
  uint8_t *dst = p_dst;

  if (dst_len < 10)
    return -1;

  dst[0] = 0x0a; /* bLength = 10 */
  dst[1] = 0x06; /* dev qualifier desc type */
  dst[2] = 0x00; /* bcdUSB low */
  dst[3] = 0x02; /* bcdUSB high */
  dst[4] = 0xEF; /* bDeviceClass */
  dst[5] = 0x02; /* bDeviceSubClass */
  dst[6] = 0x01; /* bDeviceProtocol */
  dst[7] = 0x40; /* bMaxPacketSize0 64 */
  dst[8] = 0x01; /* bNumConfigurations */
  dst[9] = 0x00; /* bReserved */

  return dst[0];
}

int usb_get_lang_string_desc(void *p_dst, int dst_len)
{
  uint8_t *dst = p_dst;

  if (dst_len < 4)
    return -1;

  dst[0] = 0x04; /* bLength = 4 */
  dst[1] = 0x03; /* string descriptor */
  dst[2] = 0x09; /* United States */
  dst[3] = 0x04; /* United States */

  return dst[0];
}

int usb_get_manufacturer_string_desc(void *p_dst, int dst_len)
{
  return usb_get_string_desc(p_dst, dst_len, DEV_MANUFACTURER_STRING);
}

int usb_get_product_string_desc(void *p_dst, int dst_len)
{
  return usb_get_string_desc(p_dst, dst_len, DEV_PRODUCT_STRING);
}

int usb_get_serial_string_desc(void *p_dst, int dst_len)
{
  char serial[13];

  gen_serial(serial);
  return usb_get_string_desc(p_dst, dst_len, serial);
}

static void build_usb_cdc_conf_desc(struct usb_cdc_conf_desc *desc, int max_packet_size)
{
	  	build_usb_conf_desc(&desc->conf_desc, &desc->cdc_iad_desc.head);
    	build_std_iad_desc(&desc->cdc_iad_desc, &desc->cdc_ctrl_itf.head, &desc->conf_desc.head);
    	build_std_itf_desc(&desc->cdc_ctrl_itf, &desc->hdr_func_desc.head, &desc->cdc_iad_desc.head, 1, 0,
                            0x02,/* Communications Interface Class */
                            0x02,/* Abstract Control Model */
                            0x01);/* AT Commands: V.250 etc */
        build_cdc_hdr_func_desc(&desc->hdr_func_desc, &desc->acm_func_desc.head, &desc->cdc_ctrl_itf.head);
        build_cdc_acm_func_desc(&desc->acm_func_desc, &desc->union_func_desc.head, &desc->hdr_func_desc.head);
        build_cdc_union_func_desc(&desc->union_func_desc, &desc->call_mgt_func_desc.head, &desc->acm_func_desc.head, 0, 1);
        build_cdc_call_mgt_func_desc(&desc->call_mgt_func_desc, &desc->cdc_acm_ctrl_ep_desc.head, &desc->union_func_desc.head, 1);
        build_std_ep_desc(&desc->cdc_acm_ctrl_ep_desc, &desc->cdc_data_itf.head, &desc->call_mgt_func_desc.head,
                           0x81, /* IN ep */
                           0x03, /* irq */
                           8, /* max packet size */
                           8); /* interval */
        build_std_itf_desc(&desc->cdc_data_itf, &desc->cdc_acm_data_in_ep_desc.head, &desc->cdc_acm_ctrl_ep_desc.head, 2, 1,
							0x0a,/* cdc data itf */
							0x00,/* undefined */
							0x00);/* undefined */
        build_std_ep_desc(&desc->cdc_acm_data_in_ep_desc, &desc->cdc_acm_data_out_ep_desc.head, &desc->cdc_data_itf.head,
                          	0x04, /* OUT ep */ // orig: 0x04
                            0x02, /* bulk */
							max_packet_size, /* max packet size */
                            0);
        build_std_ep_desc(&desc->cdc_acm_data_out_ep_desc, NULL, &desc->cdc_acm_data_in_ep_desc.head,
                           0x82, /* IN ep */ // orig: 0x83
                           0x02, /* bulk */
						   max_packet_size, /* max packet size */
                           0);
}

static int build_hs_usb_configuration_desc(uint8_t *p_dst, int dst_len)
{
  struct usb_cdc_conf_desc desc = { 0 };
  int max_packet_size = 512;

  build_usb_cdc_conf_desc(&desc, max_packet_size);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}

static int build_fs_usb_configuration_desc(uint8_t *p_dst, int dst_len)
{
  struct usb_cdc_conf_desc desc = { 0 };
  int max_packet_size = 64;

  build_usb_cdc_conf_desc(&desc, max_packet_size);
  update(&desc.conf_desc.head);
  return generate(&desc.conf_desc.head, p_dst, dst_len);
}



int usb_get_configuration_desc(void *p_dst, int dst_len, usb_desc_conf *p_conf)
{
  if (p_conf->is_hs)
    return build_hs_usb_configuration_desc(p_dst, dst_len);
  else
    return build_fs_usb_configuration_desc(p_dst, dst_len);
}

