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
#include "libuart.h"

/*
 * Helper: Map a uart_info_t (by device string or index) to a unique TCP port.
 * Uses a SIMULITH_UART_BASE_PORT and increments for each UART.
 * Example: base_port = 60000, UART0 -> 60000, UART1 -> 60001, etc.
 */
#define HWLIB_UART_MAX_PORTS 32

static void make_simulith_uart_address(char* out, size_t outlen, int idx) 
{
    snprintf(out, outlen, "ipc:///tmp/simulith_pub:%d", SIMULITH_UART_BASE_PORT + idx);
    OS_printf("HWLIB: make_simulith_uart_address: %s\n", out);
}

/*
 * Simulith UART port storage: indexed by UART number (handle)
 */
static transport_port_t *simulith_uart_ports[HWLIB_UART_MAX_PORTS] = {0};

int32_t uart_init_port(uart_info_t* device)
{
    int32_t status = UART_SUCCESS;

    if (!device) 
    {
        OS_printf("HWLIB: uart_init_port: device is NULL\n");
        return UART_ERROR;
    }

    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS) 
    {
        OS_printf("HWLIB: uart_init_port: invalid UART index %d\n", idx);
        return UART_ERROR;
    }

    if (!simulith_uart_ports[idx]) 
    {
        transport_port_t *port = (transport_port_t *)calloc(1, sizeof(transport_port_t));
        if (!port) 
        {
            OS_printf("HWLIB: uart_init_port: failed to allocate transport_port_t\n");
            return UART_ERROR;
        }
        // Set logical name for logging
        if (device->deviceString) 
        {
            strncpy(port->name, device->deviceString, sizeof(port->name)-1);
        } else 
        {
            snprintf(port->name, sizeof(port->name), "UART%d", idx);
        }
        make_simulith_uart_address(port->address, sizeof(port->address), idx);
        port->is_server = 0; // Always connect, never bind
        simulith_uart_ports[idx] = port;
    }

    transport_port_t *port = simulith_uart_ports[idx];
    status = simulith_transport_init((transport_port_t*)port);
    if(status == SIMULITH_TRANSPORT_SUCCESS)
    {
        device->isOpen = PORT_OPEN;
    }
    else
    {
        OS_printf("HWLIB: simulith_uart_init failed with status %d\n", status);
        device->isOpen = PORT_CLOSED;
        status = UART_ERROR;
    }
    return status;
}

/* uart flush */
int32_t uart_flush(uart_info_t* device)
{
    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS || !simulith_uart_ports[idx]) return UART_ERROR;
    simulith_transport_flush((transport_port_t*)simulith_uart_ports[idx]);
    return UART_SUCCESS;
}

/* uart write */
int32_t uart_write_port(uart_info_t* device, uint8_t data[], const uint32_t numBytes)
{
    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS || !simulith_uart_ports[idx]) return UART_ERROR;
    int32_t status = simulith_transport_send((transport_port_t*)simulith_uart_ports[idx], data, numBytes);
    if((uint32_t) status != numBytes)
    {
        OS_printf("HWLIB: simulith_uart_send failed with status %d\n", status);
        status = UART_ERROR;
    }
    return status;
}

/* uart read */
int32_t uart_read_port(uart_info_t* device, uint8_t data[], const uint32_t numBytes)
{
    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS || !simulith_uart_ports[idx]) return UART_ERROR;
    int32_t status = simulith_transport_receive((transport_port_t*)simulith_uart_ports[idx], data, numBytes);
    if(status < 0)
    {
        OS_printf("HWLIB: simulith_uart_receive failed with status %d\n", status);
    }
    return status;
}

/* uart number bytes available */
int32_t uart_bytes_available(uart_info_t* device)
{
    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS || !simulith_uart_ports[idx]) return UART_ERROR;
    int32_t status = simulith_transport_available((transport_port_t*)simulith_uart_ports[idx]);
    if(status < 0)
    {
        OS_printf("HWLIB: simulith_uart_available failed with status %d\n", status);
    }
    return (int32_t)simulith_uart_ports[idx]->rx_buf_len; // Return the length of the RX buffer
}

int32_t uart_close_port(uart_info_t* device) 
{
    int idx = (int)(device->handle);
    if (idx < 0 || idx >= HWLIB_UART_MAX_PORTS || !simulith_uart_ports[idx]) return UART_ERROR;
    int32_t status = simulith_transport_close((transport_port_t*)simulith_uart_ports[idx]);
    if(status < 0)
    {
        OS_printf("HWLIB: simulith_uart_close failed with status %d\n", status);
    }
    free(simulith_uart_ports[idx]);
    simulith_uart_ports[idx] = NULL;
    return status;
}
