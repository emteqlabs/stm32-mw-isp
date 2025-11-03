 /**
 ******************************************************************************
 * @file    usb_desc_internal.h
 * @author  AIS Application Team
 *
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

#include <stdint.h>
#include "cmsis_compiler.h"

struct usb_desc_head {
  uint8_t bLength;
  uint8_t *raw;
  struct usb_desc_head *next;
  struct usb_desc_head *next_child;
  struct usb_desc_head *children;
  int (*gen)(struct usb_desc_head *head, uint8_t *p_dst, int dst_len);
  void (*update)(struct usb_desc_head *head);
};

struct usb_dev_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} __PACKED;

struct usb_dev_desc {
  struct usb_desc_head head;
  struct usb_dev_desc_raw raw;
};

struct usb_conf_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} __PACKED;

struct usb_conf_desc {
  struct usb_desc_head head;
  struct usb_conf_desc_raw raw;
};

struct std_itf_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} __PACKED;

struct std_itf_desc {
  struct usb_desc_head head;
  struct std_itf_desc_raw raw;
};

struct cdc_hdr_func_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint16_t bcdCDC;
} __PACKED;

struct cdc_hdr_func_desc {
  struct usb_desc_head head;
  struct cdc_hdr_func_desc_raw raw;
};

struct cdc_acm_func_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint8_t bmCapabilities;
} __PACKED;

struct cdc_acm_func_desc {
  struct usb_desc_head head;
  struct cdc_acm_func_desc_raw raw;
};

struct cdc_union_func_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint8_t bControlInterface;
  uint8_t bSubordinateInterface0;
} __PACKED;

struct cdc_union_func_desc {
  struct usb_desc_head head;
  struct cdc_union_func_desc_raw raw;
};

struct cdc_call_mgt_func_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint8_t bmCapabilities;
  uint8_t bDataInterface;
} __PACKED;

struct cdc_call_mgt_func_desc {
  struct usb_desc_head head;
  struct cdc_call_mgt_func_desc_raw raw;
};

struct std_ep_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
} __PACKED;

struct std_ep_desc {
  struct usb_desc_head head;
  struct std_ep_desc_raw raw;
};

struct std_iad_desc_raw {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bFirstInterface;
  uint8_t bInterfaceCount;
  uint8_t bFunctionClass;
  uint8_t bFunctionSubClass;
  uint8_t bFunctionProtocol;
  uint8_t iFunction;
} __PACKED;

struct std_iad_desc {
  struct usb_desc_head head;
  struct std_iad_desc_raw raw;
};


struct usb_cdc_conf_desc {
	  struct usb_conf_desc conf_desc;
   struct std_iad_desc cdc_iad_desc;
    struct std_itf_desc cdc_ctrl_itf;
      struct cdc_hdr_func_desc hdr_func_desc;
        struct cdc_acm_func_desc acm_func_desc;
        struct cdc_union_func_desc union_func_desc;
        struct cdc_call_mgt_func_desc call_mgt_func_desc;
      struct std_ep_desc cdc_acm_ctrl_ep_desc;
    struct std_itf_desc cdc_data_itf;
      struct std_ep_desc cdc_acm_data_in_ep_desc;
      struct std_ep_desc cdc_acm_data_out_ep_desc;
};
