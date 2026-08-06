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
#include "libi2c.h"

/*
 * Helper: Map an i2c_bus_info_t to a unique TCP port.
 * Uses SIMULITH_I2C_BASE_PORT + (bus_id * 100) + device_addr
 * Example: base_port = 52000, Bus 0 Device 10 -> 52010, Bus 1 Device 23 -> 52123
 */
#define HWLIB_I2C_MAX_DEVICES 256

static void make_simulith_i2c_address(char* out, size_t outlen, int bus_id, int device_addr) 
{
    int port = SIMULITH_I2C_BASE_PORT + (bus_id * 100) + device_addr;
    snprintf(out, outlen, "ipc:///tmp/simulith_pub:%d", port);
    OS_printf("HWLIB: make_simulith_i2c_address: Bus %d Device 0x%02X -> %s\n", bus_id, device_addr, out);
}

/*
 * Simulith I2C device storage: indexed by a hash of bus_id and device address
 */
static transport_port_t *simulith_i2c_devices[HWLIB_I2C_MAX_DEVICES] = {0};

int32_t i2c_master_init(i2c_bus_info_t* device)
{
    int32_t status = I2C_SUCCESS;

    if (!device) 
    {
        OS_printf("HWLIB: i2c_master_init: device is NULL\n");
        return I2C_ERROR;
    }

    int idx = (int)(device->handle);
    int bus_id = (int)(device->handle); /* keep original semantics: handle encodes bus */
    int device_addr = (int)(device->addr);

    if (idx < 0 || idx >= HWLIB_I2C_MAX_DEVICES) {
        OS_printf("HWLIB: i2c_master_init: invalid handle/index %d\n", idx);
        return I2C_ERROR;
    }

    if (!simulith_i2c_devices[idx]) {
        transport_port_t *i2c_dev = (transport_port_t *)calloc(1, sizeof(transport_port_t));
        if (!i2c_dev) {
            OS_printf("HWLIB: i2c_master_init: failed to allocate transport_port_t\n");
            return I2C_ERROR;
        }
        /* Set logical name for logging */
        snprintf(i2c_dev->name, sizeof(i2c_dev->name), "I2C%d_0x%02X", bus_id, device_addr);
        make_simulith_i2c_address(i2c_dev->address, sizeof(i2c_dev->address), bus_id, device_addr);
        i2c_dev->is_server = 0; // Always connect, never bind
        OS_printf("HWLIB: i2c_master_init: created transport port '%s' -> %s (handle %d, bus %d, addr 0x%02X)\n",
                  i2c_dev->name, i2c_dev->address, idx, bus_id, device_addr);
        simulith_i2c_devices[idx] = i2c_dev;
    }

    transport_port_t *i2c_dev = simulith_i2c_devices[idx];
    status = simulith_transport_init((transport_port_t*)i2c_dev);
    if(status == SIMULITH_TRANSPORT_SUCCESS)
    {
        device->isOpen = I2C_OPEN;
    }
    else
    {
        OS_printf("HWLIB: simulith_i2c_init failed with status %d\n", status);
        device->isOpen = I2C_CLOSED;
        status = I2C_ERROR;
    }
    return status;
}

/* nos i2c transaction */
int32_t i2c_master_transaction(i2c_bus_info_t* device, uint8_t addr, void * txbuf, uint8_t txlen,
                               void * rxbuf, uint8_t rxlen, uint16_t timeout)
{
    if (!device) return I2C_ERROR;
    
    int bus_id = (int)(device->handle);
    int device_addr = (int)(device->addr);
    int idx = (int)(device->handle);

    if (idx < 0 || idx >= HWLIB_I2C_MAX_DEVICES) {
        OS_printf("HWLIB: i2c_master_transaction: invalid handle/index %d\n", idx);
        return I2C_ERROR;
    }
    if (!simulith_i2c_devices[idx]) {
        OS_printf("HWLIB: i2c_master_transaction: transport port not initialized for handle %d (bus %d addr 0x%02X)\n", idx, bus_id, device_addr);
        return I2C_ERROR;
    }
    transport_port_t *i2c_dev = simulith_i2c_devices[idx];
    int32_t status = -1;
    int sent = simulith_transport_send((transport_port_t*)i2c_dev, (const uint8_t*)txbuf, txlen);
    OS_printf("HWLIB: i2c_master_transaction: send returned %d (expected %u)\n", sent, txlen);
    if (sent > 0) {
        /* hex dump first up to 64 bytes of tx */
        int dump = (sent < 64) ? sent : 64;
        OS_printf("HWLIB: i2c_master_transaction: TX[%d] first %d bytes:", sent, dump);
        for (int i = 0; i < dump; ++i) OS_printf(" %02X", ((uint8_t*)txbuf)[i]);
        OS_printf("\n");
    }
    if (sent == (int)txlen) {
        int poll_attempts = (timeout > 0) ? (int)(timeout / 2) : 20;
        int r = 0;
        for (int i = 0; i < poll_attempts; ++i) {
            int available = simulith_transport_available((transport_port_t*)i2c_dev);
            if (available > 0) {
                r = simulith_transport_receive((transport_port_t*)i2c_dev, (uint8_t*)rxbuf, rxlen);
                break;
            }
            OS_TaskDelay(2);
        }
        if (r == (int)rxlen) status = SIMULITH_TRANSPORT_SUCCESS;
        else {
            OS_printf("HWLIB: i2c_master_transaction: no response after %d polls\n", poll_attempts);
        }
    }
    if(status != SIMULITH_TRANSPORT_SUCCESS)
    {
        OS_printf("HWLIB: simulith_i2c_transaction failed with status %d\n", status);
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

int32_t i2c_read_transaction(i2c_bus_info_t* device, uint8_t addr, void * rxbuf, uint8_t rxlen, uint8_t timeout)
{
    if (!device) return I2C_ERROR;
    int bus_id = (int)(device->handle);
    int device_addr = (int)(device->addr);
    int idx = (int)(device->handle);

    if (idx < 0 || idx >= HWLIB_I2C_MAX_DEVICES) {
        OS_printf("HWLIB: i2c_read_transaction: invalid handle/index %d\n", idx);
        return I2C_ERROR;
    }
    if (!simulith_i2c_devices[idx]) {
        OS_printf("HWLIB: i2c_read_transaction: transport port not initialized for handle %d (bus %d addr 0x%02X)\n", idx, bus_id, device_addr);
        return I2C_ERROR;
    }

    transport_port_t *i2c_dev = simulith_i2c_devices[idx];
    int32_t status = 0;
    int poll_attempts = (timeout > 0) ? (int)(timeout / 2) : 20;
    for (int i = 0; i < poll_attempts; ++i) {
        int available = simulith_transport_available((transport_port_t*)i2c_dev);
        if (available > 0) {
            status = simulith_transport_receive((transport_port_t*)i2c_dev, (uint8_t*)rxbuf, rxlen);
            break;
        }
        OS_TaskDelay(2);
    }
    if (status <= 0) {
        OS_printf("HWLIB: i2c_read_transaction: no response after %d polls\n", poll_attempts);
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

int32_t i2c_write_transaction(i2c_bus_info_t* device, uint8_t addr, void * txbuf, uint8_t txlen, uint8_t timeout)
{
    if (!device) return I2C_ERROR;
    int bus_id = (int)(device->handle);
    int device_addr = (int)(device->addr);
    int idx = (int)(device->handle);

    if (idx < 0 || idx >= HWLIB_I2C_MAX_DEVICES) {
        OS_printf("HWLIB: i2c_write_transaction: invalid handle/index %d\n", idx);
        return I2C_ERROR;
    }
    if (!simulith_i2c_devices[idx]) {
        OS_printf("HWLIB: i2c_write_transaction: transport port not initialized for handle %d (bus %d addr 0x%02X)\n", idx, bus_id, device_addr);
        return I2C_ERROR;
    }

    transport_port_t *i2c_dev = simulith_i2c_devices[idx];
    int32_t status = simulith_transport_send((transport_port_t*)i2c_dev, (const uint8_t*)txbuf, txlen);
    if(status < 0)
    {
        OS_printf("HWLIB: simulith_i2c_write failed with status %d\n", status);
        return I2C_ERROR;
    }
    return I2C_SUCCESS;
}

int32_t i2c_multiple_transaction(i2c_bus_info_t* device, uint8_t addr, struct i2c_rdwr_ioctl_data* rdwr_data, uint16_t timeout)
{
    // For now, return error as this is more complex to implement with the current architecture
    OS_printf("HWLIB: i2c_multiple_transaction: not implemented in simulith mode\n");
    return I2C_ERROR;
}

int32_t i2c_master_close(i2c_bus_info_t* device) 
{
    if (!device) return I2C_ERROR;
    
    int bus_id = (int)(device->handle);
    int device_addr = (int)(device->addr);
    int idx = (int)(device->handle);

    if (idx < 0 || idx >= HWLIB_I2C_MAX_DEVICES) {
        OS_printf("HWLIB: i2c_master_close: invalid handle/index %d\n", idx);
        return I2C_ERROR;
    }
    if (!simulith_i2c_devices[idx]) {
        OS_printf("HWLIB: i2c_master_close: transport port not initialized for handle %d (bus %d addr 0x%02X)\n", idx, bus_id, device_addr);
        return I2C_ERROR;
    }

    transport_port_t *i2c_dev = simulith_i2c_devices[idx];
    int32_t status = simulith_transport_close((transport_port_t*)i2c_dev);
    
    if(status < 0)
    {
        OS_printf("HWLIB: simulith_i2c_close failed with status %d\n", status);
    }
    
    free(simulith_i2c_devices[idx]);
    simulith_i2c_devices[idx] = NULL;
    device->isOpen = I2C_CLOSED;
    
    return (status == SIMULITH_TRANSPORT_SUCCESS) ? I2C_SUCCESS : I2C_ERROR;
}
