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

#include "simulith.h"
#include "libspi.h"

// Storage for transport_port_t devices mapped to spi_info_t devices
#define HWLIB_SPI_MAX_DEVICES (MAX_SPI_BUSES * 8)
static transport_port_t* simulith_spi_devices[HWLIB_SPI_MAX_DEVICES] = {0};

// Helper function to get or create simulith transport_port_t for spi_info_t
static transport_port_t* get_simulith_device(spi_info_t* device)
{
    if (!device) return NULL;
    int idx = device->bus * 8 + device->cs;
    if (idx < 0 || idx >= HWLIB_SPI_MAX_DEVICES) return NULL;
    if (!simulith_spi_devices[idx]) {
        transport_port_t* port = (transport_port_t*)calloc(1, sizeof(transport_port_t));
        if (!port) return NULL;
        snprintf(port->name, sizeof(port->name), "SPI%d_CS%d", device->bus, device->cs);
        snprintf(port->address, sizeof(port->address), "ipc:///tmp/simulith_pub:%d", SIMULITH_SPI_BASE_PORT + (device->bus * 8) + device->cs);
        port->is_server = 0;
        simulith_spi_devices[idx] = port;
    }
    return simulith_spi_devices[idx];
}

int32_t spi_init_dev(spi_info_t* device)
{
    if (!device) return SPI_ERROR;
    
    transport_port_t* port = get_simulith_device(device);
    if (!port) return SPI_ERROR;
    int result = simulith_transport_init(port);
    if (result == SIMULITH_TRANSPORT_SUCCESS) {
        device->isOpen = SPI_DEVICE_OPEN;
        return SPI_SUCCESS;
    }
    return SPI_ERROR;
}

int32_t spi_select_chip(spi_info_t* device)
{
    // In simulith, chip select is implicit in addressing, so this is a no-op
    if (!device) return SPI_ERROR;
    return SPI_SUCCESS;
}

int32_t spi_unselect_chip(spi_info_t* device)
{
    // In simulith, chip select is implicit in addressing, so this is a no-op
    if (!device) return SPI_ERROR;
    return SPI_SUCCESS;
}

int32_t spi_write(spi_info_t* device, uint8_t data[], const uint32_t numBytes)
{
    if (!device || device->isOpen != SPI_DEVICE_OPEN) return SPI_ERROR;
    
    transport_port_t* port = get_simulith_device(device);
    if (!port) return SPI_ERROR;
    int result = simulith_transport_send(port, data, numBytes);
    if (result < 0) return SPI_ERROR;
    return result;
}

int32_t spi_read(spi_info_t* device, uint8_t data[], const uint32_t numBytes)
{
    if (!device || device->isOpen != SPI_DEVICE_OPEN) return SPI_ERROR;
    transport_port_t* port = get_simulith_device(device);
    if (!port) return SPI_ERROR;
    int poll_attempts = 100; /* previously 20 */
    int got_resp = 0;
    int result = -1;
    for (int i = 0; i < poll_attempts; ++i) {
        int available = simulith_transport_available(port);
        if (available > 0) {
            result = simulith_transport_receive(port, data, numBytes);
            if (result == (int)numBytes) {
                got_resp = 1;
                break;
            }
        }
        OS_TaskDelay(2);
    }
    if (!got_resp) return SPI_ERROR;
    return result;
}

int32_t spi_transaction(spi_info_t* device, uint8_t *txBuff, uint8_t * rxBuffer, uint32_t length, uint16_t delay, uint8_t bits, uint8_t deselect)
{
    if (!device || device->isOpen != SPI_DEVICE_OPEN) return SPI_ERROR;
    
    transport_port_t* port = get_simulith_device(device);
    if (!port) return SPI_ERROR;

    int32_t status = -1;
    int sent = simulith_transport_send(port, txBuff, length);
    if (sent == (int)length) 
    {
        int poll_attempts = 100;
        int got_resp = 0;
        int r = -1;
        for (int i = 0; i < poll_attempts; ++i) 
        {
            int available = simulith_transport_available(port);
            if (available > 0) 
            {
                r = simulith_transport_receive(port, rxBuffer, length);
                if (r == (int)length) 
                {
                    got_resp = 1;
                    break;
                }
            }
            OS_TaskDelay(2);
        }
        if (got_resp) status = SIMULITH_TRANSPORT_SUCCESS;
    }
    if (status != SIMULITH_TRANSPORT_SUCCESS) return SPI_ERROR;
    return SPI_SUCCESS;
}

int32_t spi_close_device(spi_info_t* device)
{
    if (!device) return SPI_ERROR;
    
    transport_port_t* port = get_simulith_device(device);
    if (!port) return SPI_ERROR;
    int result = simulith_transport_close(port);
    if (result == SIMULITH_TRANSPORT_SUCCESS) {
        device->isOpen = SPI_DEVICE_CLOSED;
        return SPI_SUCCESS;
    }
    return SPI_ERROR;
}
