/* Copyright (C) 2009 - 2019 National Aeronautics and Space Administration. All Foreign Rights are Reserved to the U.S. Government.

This software is provided "as is" without any warranty of any, kind either express, implied, or statutory, including, but not
limited to, any warranty that the software will conform to, specifications any implied warranties of merchantability, fitness
for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the program, or
any warranty that the software will be error free.

In no event shall NASA be liable for any damages, including, but not limited to direct, indirect, special or consequential damages,
arising out of, resulting from, or in any way connected with the software or its documentation.  Whether or not based upon warranty,
contract, tort or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software,
documentation or services provided hereunder

ITC Team
NASA IV&V
ivv-itc@lists.nasa.gov
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "simulith.h"
#include "libgpio.h"

// Storage for transport_port_t devices mapped to gpio_info_t devices
#define HWLIB_GPIO_MAX_DEVICES 256
static transport_port_t* simulith_gpio_devices[HWLIB_GPIO_MAX_DEVICES] = {0};

// Helper function to get or create simulith transport_port_t for gpio_info_t
static transport_port_t* get_simulith_gpio_device(gpio_info_t* device)
{
    if (!device) return NULL;
    int idx = (int)device->pin;
    if (idx < 0 || idx >= HWLIB_GPIO_MAX_DEVICES) return NULL;
    if (!simulith_gpio_devices[idx]) {
        transport_port_t* port = (transport_port_t*)calloc(1, sizeof(transport_port_t));
        if (!port) return NULL;
        snprintf(port->name, sizeof(port->name), "GPIO%d", device->pin);
        snprintf(port->address, sizeof(port->address), "ipc:///tmp/simulith_pub:%d", SIMULITH_GPIO_BASE_PORT + device->pin);
        port->is_server = 0;
        simulith_gpio_devices[idx] = port;
    }
    return simulith_gpio_devices[idx];
}

int32_t gpio_init(gpio_info_t* device) 
{    
    if (!device) return GPIO_ERROR;
    if (device->pin > 255) return GPIO_ERROR; // Pin limit check
    
    transport_port_t* port = get_simulith_gpio_device(device);
    if (!port) return GPIO_ERROR;
    int result = simulith_transport_init(port);
    if (result == SIMULITH_TRANSPORT_SUCCESS) {
        device->isOpen = GPIO_OPEN;
        return GPIO_SUCCESS;
    }
    return GPIO_ERROR;
}

int32_t gpio_read(gpio_info_t* device, uint8_t* value)
{
    if (!device || !value || device->isOpen != GPIO_OPEN) return GPIO_ERROR;
    
    transport_port_t* port = get_simulith_gpio_device(device);
    if (!port) return GPIO_ERROR;
    // Send read request: [cmd=0, pin]
    uint8_t req[2] = {0, (uint8_t)device->pin};
    int rc = simulith_transport_send(port, req, sizeof(req));
    if (rc != 2) return GPIO_ERROR;
    // Poll for response: [cmd=0, pin, value]
    uint8_t resp[3];
    int poll_attempts = 20;
    int got_resp = 0;
    for (int i = 0; i < poll_attempts; ++i) {
        int available = simulith_transport_available(port);
        if (available > 0) {
            rc = simulith_transport_receive(port, resp, sizeof(resp));
            if (rc == 3 && resp[0] == 0 && resp[1] == (uint8_t)device->pin) {
                *value = resp[2];
                got_resp = 1;
                break;
            }
        }
        OS_TaskDelay(2);
    }
    if (got_resp) return GPIO_SUCCESS;
    return GPIO_ERROR;
}

int32_t gpio_write(gpio_info_t* device, uint8_t value)
{
    if (!device || device->isOpen != GPIO_OPEN) return GPIO_ERROR;
    if (value > 1) return GPIO_ERROR; // Value validation
    
    transport_port_t* port = get_simulith_gpio_device(device);
    if (!port) return GPIO_ERROR;
    // Send write request: [cmd=1, pin, value]
    uint8_t req[3] = {1, (uint8_t)device->pin, (uint8_t)(value & 0x1)};
    int rc = simulith_transport_send(port, req, sizeof(req));
    if (rc == 3) return GPIO_SUCCESS;
    return GPIO_ERROR;
}

int32_t gpio_close(gpio_info_t* device)
{
    if (!device) return GPIO_ERROR;
    
    transport_port_t* port = get_simulith_gpio_device(device);
    if (!port) return GPIO_ERROR;
    int rc = simulith_transport_close(port);
    if (rc == SIMULITH_TRANSPORT_SUCCESS) {
        device->isOpen = GPIO_CLOSED;
        return GPIO_SUCCESS;
    }
    return GPIO_ERROR;
}
