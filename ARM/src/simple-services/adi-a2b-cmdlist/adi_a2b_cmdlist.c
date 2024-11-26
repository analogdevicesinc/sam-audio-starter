/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>

#include "adi_a2b_commandlist.h"
#include "adi_a2b_cmdlist.h"

#ifdef FREE_RTOS
    #include "FreeRTOS.h"
    #include "task.h"
    #define ADI_A2B_CMDLIST_ENTER_CRITICAL()  vTaskSuspendAll()
    #define ADI_A2B_CMDLIST_EXIT_CRITICAL()   xTaskResumeAll()
#else
    #define ADI_A2B_CMDLIST_ENTER_CRITICAL()
    #define ADI_A2B_CMDLIST_EXIT_CRITICAL()
#endif

#ifndef ADI_A2B_CMDLIST_MAX_LISTS
#define ADI_A2B_CMDLIST_MAX_LISTS 1
#endif

#ifndef ADI_A2B_CMDLIST_MEMSET
#define ADI_A2B_CMDLIST_MEMSET memset
#endif

#ifndef ADI_A2B_CMDLIST_MEMCPY
#define ADI_A2B_CMDLIST_MEMCPY memcpy
#endif

#ifndef ADI_A2B_CMDLIST_READBUF_LEN
#define ADI_A2B_CMDLIST_READBUF_LEN 16
#endif

#define     AD24xx_REG_CHIP                    0x00
#define     AD24xx_REG_NODEADR                 0x01
#define     AD24xx_REG_VENDOR                  0x02
#define     AD24xx_REG_PRODUCT                 0x03
#define     AD24xx_REG_VERSION                 0x04
#define     AD24xx_REG_SWCTL                   0x09
#define     AD24xx_REG_BCDNSLOTS               0x0A
#define     AD24xx_REG_LDNSLOTS                0x0B
#define     AD24xx_REG_LUPSLOTS                0x0C
#define     AD24xx_REG_DNSLOTS                 0x0D
#define     AD24xx_REG_UPSLOTS                 0x0E
#define     AD24xx_REG_RESPCYCS                0x0F
#define     AD24xx_REG_SLOTFMT                 0x10
#define     AD24xx_REG_DATCTL                  0x11
#define     AD24xx_REG_CONTROL                 0x12
#define     AD24xx_REG_DISCVRY                 0x13
#define     AD24xx_REG_INTSRC                  0x16
#define     AD24xx_REG_INTTYPE                 0x17
#define     AD24xx_REG_INTMSK0                 0x1B
#define     AD24xx_REG_NODE                    0x29
#define     AD24xx_REG_I2SGCFG                 0x41
#define     AD24xx_REG_I2SCFG                  0x42
#define     AD24xx_REG_I2STXOFFSET             0x44
#define     AD24xx_REG_PDMCTL                  0x47
#define     AD24xx_REG_CLK1CFG                 0x59
#define     AD24xx_REG_CLK2CFG                 0x5A

#define     AD24xx_BITM_NODEADR_BRCST          0x80
#define     AD24xx_BITM_NODEADR_PERI           0x20
#define     AD24xx_BITM_NODEADR_NODE           0x0F
#define     AD24xx_BITM_CONTROL_NEWSTRCT       0x01
#define     AD24xx_BITM_CONTROL_ENDDSC         0x02
#define     AD24xx_BITM_CONTROL_MSTR           0x80
#define     AD24xx_BITM_CONTROL_SOFTRST        0x04
#define     AD24xx_BITM_INTSRC_MSTINT          0x80
#define     AD24xx_BITM_INTSRC_SLVINT          0x40
#define     AD24xx_BITM_INTMSK0_PWREIEN        0x10
#define     AD24xx_BITM_NODE_LAST              0x80
#define     AD24xx_BITM_I2SCFG_TX              0x0F
#define     AD24xx_BITM_DATCTL_ENDSNIFF        0x20
#define     AD24xx_BITM_SWCTL_ENSW             0x01
#define     AD24xx_ENUM_SWCTL_MODE2            0x20

enum {
    ADI_A2B_CMDLIST_IRQ_NONE = 0,
    ADI_A2B_CMDLIST_IRQ_PLL_LOCK_PENDING,
    ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING,
};

typedef enum _ADI_A2B_CMDLIST_ACCESS_TYPE {
    ADI_A2B_CMDLIST_UNKNOWN_ACCESS = 0,
    ADI_A2B_CMDLIST_NO_ACCESS,
    ADI_A2B_CMDLIST_BLOCK_ACCESS,
    ADI_A2B_CMDLIST_MASTER_ACCESS,
    ADI_A2B_CMDLIST_SLAVE_ACCESS
} ADI_A2B_CMDLIST_ACCESS_TYPE;

typedef struct _ADI_A2B_CMDLIST_IRQ_REGS {
    uint8_t intSrc;
    uint8_t intType;
} ADI_A2B_CMDLIST_IRQ_REGS;

typedef struct _ADI_A2B_CMDLIST_RESULT_STRING {
    ADI_A2B_CMDLIST_RESULT result;
    const char *string;
} ADI_A2B_CMDLIST_RESULT_STRING;

typedef struct _ADI_A2B_CMDLIST_PRIV_NODE_INFO {
    uint8_t respCycs;
    uint32_t timeoutMs;
    uint8_t UPSLOTS;
    uint8_t LUPSLOTS;
    uint8_t BCDNSLOTS;
    uint8_t DNSLOTS;
    uint8_t LDNSLOTS;
    bool disableClocks;
} ADI_A2B_CMDLIST_PRIV_NODE_INFO;

typedef struct _ADI_A2B_CMDLIST {
    bool busy;
    ADI_A2B_CMDLIST_CFG cfg;
    void *cmdList;
    uint32_t cmdListLen;
    A2B_CMD_TYPE cmdListType;
    void *cmdPtr;
    uint32_t cmdIdx;
    uint32_t cmdLine;
    uint8_t cmdListMasterAddr;
    uint8_t nodesDiscovered;
    uint8_t nodesScanned;
    uint8_t nodeIdx;
    bool faultDetected;
    uint8_t faultNode;
    ADI_A2B_CMDLIST_NODE_INFO nodeInfo[ADI_A2B_CMDLIST_MAX_NODES];
    ADI_A2B_CMDLIST_PRIV_NODE_INFO privNodeInfo[ADI_A2B_CMDLIST_MAX_NODES];
    ADI_A2B_CMDLIST_OVERRIDE_INFO oi;
    uint8_t SLOTFMT;
    uint8_t DATCTL;
    uint8_t UPSLOTS;
    uint8_t LUPSLOTS;
    uint8_t DNSLOTS;
    uint8_t LDNSLOTS;
    uint8_t I2SCFG;
    uint8_t I2SGCFG;
    uint8_t I2STXOFFSET;
    bool scanned;
    bool enableSniffing;
    bool abortOnError;
    ADI_A2B_RESPCYCS_FORMULA respcycsFormula;
} ADI_A2B_CMDLIST;

/* ADI Command-list independent command data structure */
typedef struct {
    unsigned char deviceAddr;
    unsigned char opCode;
    unsigned char spiCmdWidth;
    unsigned int spiCmd;
    unsigned char addrWidth;
    unsigned int addr;
    unsigned char dataWidth;
    unsigned short dataCount;
    unsigned char *configData;
    unsigned char protocol;
} ADI_A2B_CMDLIST_CMD;

static ADI_A2B_CMDLIST cmdList[ADI_A2B_CMDLIST_MAX_LISTS];

const ADI_A2B_CMDLIST_RESULT_STRING ADI_A2B_CMDLIST_RESULT_STRINGS[] =  {
    { ADI_A2B_CMDLIST_SUCCESS, "SUCCESS" },
    { ADI_A2B_CMDLIST_ERROR, "ERROR" },
    { ADI_A2B_CMDLIST_CFG_ERROR, "CFG ERROR" },
    { ADI_A2B_CMDLIST_BUS_ERROR, "BUS ERROR" },
    { ADI_A2B_CMDLIST_BUS_TIMEOUT, "BUS TIMEOUT" },
    { ADI_A2B_CMDLIST_ODD_I2C_ADDRESS_ERROR, "ODD I2C ADDRESS ERROR" },
    { ADI_A2B_CMDLIST_CORRUPT_INIT_FILE, "CORRUPT INIT FILE" },
    { ADI_A2B_CMDLIST_UNSUPPORTED_INIT_FILE, "UNSUPPORTED INIT FILE" },
    { ADI_A2B_CMDLIST_UNSUPPORTED_READ_LENGTH, "UNSUPPORTED READ LENGTH" },
    { ADI_A2B_CMDLIST_UNSUPPORTED_DATA_WIDTH, "UNSUPPORTED DATA WIDTH" },
    { ADI_A2B_CMDLIST_UNSUPPORTED_PROTOCOL, "UNSUPPORTED PROTOCOL" },
    { ADI_A2B_CMDLIST_A2B_I2C_WRITE_ERROR, "I2C WRITE ERROR" },
    { ADI_A2B_CMDLIST_A2B_I2C_READ_ERROR, "I2C READ ERROR" },
    { ADI_A2B_CMDLIST_A2B_MEMORY_ERROR, "BUFFER MEMORY ERROR" },
    { ADI_A2B_CMDLIST_A2B_BUS_POS_SHORT_TO_GROUND, "POSITIVE WIRE SHORTED TO GND" },
    { ADI_A2B_CMDLIST_A2B_BUS_NEG_SHORT_TO_VBAT, "NEGATIVE WIRE SHORTED TO VBAT" },
    { ADI_A2B_CMDLIST_A2B_BUS_SHORT_TOGETHER, "WIRES SHORTED TOGETHER" },
    { ADI_A2B_CMDLIST_A2B_BUS_OPEN_OR_WRONG_PORT, "WIRE OPEN OR WRONG PORT" },
    { ADI_A2B_CMDLIST_A2B_BUS_REVERSED_OR_OPEN, "REVERSED WIRES OR WIRE OPEN" },
    { ADI_A2B_CMDLIST_A2B_BUS_REVERSED_OR_WRONG_PORT, "REVERSED WIRES OR WRONG PORT" },
    { ADI_A2B_CMDLIST_A2B_BUS_INDETERMINATE_FAULT, "INDETERMINATE FAULT" },
    { ADI_A2B_CMDLIST_A2B_BUS_UNKNOWN_FAULT, "UNKNOWN FAULT DETECTED" },
    { ADI_A2B_CMDLIST_A2B_BUS_NO_FAULT, "NO FAULT DETECTED" },
    { ADI_A2B_CMDLIST_A2B_BUS_SHORT_TO_GROUND, "CABLE SHORT TO GROUND" },
    { ADI_A2B_CMDLIST_A2B_BUS_SHORT_TO_VBAT, "CABLE SHORT TO VBAT" },
    { ADI_A2B_CMDLIST_A2B_BUS_DISCONNECT_OR_OPEN_CIRCUIT, "CABLE DISCONNECTED OR OPEN" },
    { ADI_A2B_CMDLIST_A2B_BUS_REVERSE_CONNECTED, "CABLE REVERSE CONNECTED" },
    { ADI_A2B_CMDLIST_A2B_BAD_NODE, "BAD NODE" },
    { ADI_A2B_CMDLIST_A2B_NOT_LAST_NODE, "NOT LAST NODE" },
    { ADI_A2B_CMDLIST_RESPCYCS_ERROR, "RESPCYCS CALCULATION ERROR" },
    { ADI_A2B_CMDLIST_END, "END OF COMMAND LIST" },
    { (ADI_A2B_CMDLIST_RESULT)0, NULL }
};

static void adi_a2b_cmdlist_reset(ADI_A2B_CMDLIST *list)
{
    list->cmdPtr = list->cmdList;
    list->cmdIdx = 0;
    list->cmdLine = 0;
    list->nodeIdx = 0;
}

static void adi_a2b_cmdlist_discover_restart(ADI_A2B_CMDLIST *list)
{
    list->nodesDiscovered = 0;
    list->faultDetected = false;
    list->faultNode = 0;
}

const char *adi_a2b_cmdlist_result_str(ADI_A2B_CMDLIST_RESULT result)
{
    const char *result_str = "Unknown";
    const ADI_A2B_CMDLIST_RESULT_STRING *rs;

    rs = ADI_A2B_CMDLIST_RESULT_STRINGS;
    while (rs->string != NULL) {
        if (rs->result == result) {
            result_str = rs->string;
            break;
        }
        rs++;
    }

    return(result_str);
}

static void adi_a2b_cmdlist_log(
    ADI_A2B_CMDLIST *list, bool newLine, const char *fmt, ...)
{
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    va_list va;
    if (cfg->log) {
        va_start(va, fmt);
        cfg->log(newLine, cfg->usr, fmt, va);
        va_end(va);
    }
}

static void adi_a2b_cmdlist_log_error(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_RESULT result)
{
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    if (cfg->errLog && (result != ADI_A2B_CMDLIST_SUCCESS)) {
        cfg->errLog(true, cfg->usr, adi_a2b_cmdlist_result_str(result));
    }
}

static void adi_a2b_cmdlist_log_write(ADI_A2B_CMDLIST *list,
    uint8_t address, uint8_t *out, uint16_t outLen,
    uint8_t *out2, uint16_t out2Len)
{
    adi_a2b_cmdlist_log(list, true, "(w %02X) ", address);
    if (out && outLen) {
        for (uint16_t i = 0; i < outLen; i++) {
            adi_a2b_cmdlist_log(list, false, "%02X ", ((char *)out)[i]);
        }
    }
    if (out2 && out2Len) {
        for (uint16_t i = 0; i < out2Len; i++) {
            adi_a2b_cmdlist_log(list, false, "%02X ", ((char *)out2)[i]);
        }
    }
    adi_a2b_cmdlist_log(list, false, "\n");
}

static void adi_a2b_cmdlist_log_read(ADI_A2B_CMDLIST *list,
    uint8_t address, uint8_t *in, uint16_t inLen)
{
    adi_a2b_cmdlist_log(list, true, "(r %02X) ", address);
    if (in && inLen) {
        for (uint16_t i = 0; i < inLen; i++) {
            adi_a2b_cmdlist_log(list, false, "%02X ", ((char *)in)[i]);
        }
    }
    adi_a2b_cmdlist_log(list, false, "\n");
}

static void adi_a2b_cmdlist_log_write_read(ADI_A2B_CMDLIST *list,
    uint8_t address, uint8_t *out, uint16_t outLen,
    uint8_t *in, uint16_t inLen)
{
    adi_a2b_cmdlist_log(list, true, "(w %02X) ", address);
    if (out && outLen) {
        for (uint16_t i = 0; i < outLen; i++) {
            adi_a2b_cmdlist_log(list, false, "%02X ", ((char *)out)[i]);
        }
    }
    adi_a2b_cmdlist_log(list, false, "(r) ", address);
    if (in && inLen) {
        for (uint16_t i = 0; i < inLen; i++) {
            adi_a2b_cmdlist_log(list, false, "%02X ", ((char *)in)[i]);
        }
    }
    adi_a2b_cmdlist_log(list, false, "\n");
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_write_ctrl_reg(
    ADI_A2B_CMDLIST *list, uint8_t reg, uint8_t value, uint8_t i2cAddr)
{
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    uint8_t tx[2] = { reg, value };
    ADI_A2B_CMDLIST_RESULT result;

    adi_a2b_cmdlist_log(
        list, true, "(w %02X) %02X %02x\n", i2cAddr, reg, value
    );

    result = cfg->twiWrite(
        cfg->handle, i2cAddr, tx, 2, cfg->usr
    );

    adi_a2b_cmdlist_log_error(list, result);

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_write_write(
    ADI_A2B_CMDLIST *list, uint8_t address,
    void *out, uint16_t outLen,  void *out2, uint16_t out2Len)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    uint8_t *wBuf;

    if (cfg->twiWriteWrite) {
        result = cfg->twiWriteWrite(
            cfg->handle, address,
            out, outLen, out2, out2Len,
            cfg->usr
        );
    } else {
        wBuf = cfg->getBuffer(outLen + out2Len, cfg->usr);
        if (wBuf == NULL) {
            return(ADI_A2B_CMDLIST_A2B_MEMORY_ERROR);
        }
        ADI_A2B_CMDLIST_MEMCPY(wBuf, out, outLen);
        ADI_A2B_CMDLIST_MEMCPY(wBuf + outLen, out2, out2Len);
        result = cfg->twiWrite(
            cfg->handle, address,
            wBuf, outLen + out2Len, cfg->usr
        );
        cfg->freeBuffer(wBuf, cfg->usr);
    }

    adi_a2b_cmdlist_log_write(list, address, out, outLen, out2, out2Len);

    adi_a2b_cmdlist_log_error(list, result);

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_write_ctrl_reg_block(
    ADI_A2B_CMDLIST *list,
    uint32_t addr, uint8_t addrBytes,
    uint16_t len, uint8_t *values, uint8_t i2cAddr)
{
    ADI_A2B_CMDLIST_RESULT result;
    uint8_t adr[4];

    if (addrBytes == 1) {
        adr[0] = addr & 0xFF;
    } else if (addrBytes == 2) {
        adr[0] = ((addr >> 8) & 0xFF);
        adr[1] = addr & 0xFF;
    } else if (addrBytes == 4) {
        adr[0] = ((addr >> 24) & 0xFF);
        adr[1] = ((addr >> 16) & 0xFF);
        adr[2] = ((addr >> 8) & 0xFF);
        adr[3] = addr & 0xFF;
    } else {
        return(ADI_A2B_CMDLIST_UNSUPPORTED_ADDR_BYTES);
    }

    result = adi_a2b_cmdlist_write_write(
        list, i2cAddr, adr, addrBytes, values, len
    );

    return (result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_read_ctrl_reg(
    ADI_A2B_CMDLIST *list,
    uint8_t reg, uint8_t *value, uint8_t i2cAddr)
{
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    ADI_A2B_CMDLIST_RESULT result;

    adi_a2b_cmdlist_log(list, true, "(w %02X) %02X ", i2cAddr, reg);

    result = cfg->twiWriteRead(
        cfg->handle, i2cAddr, &reg, 1, value, 1, cfg->usr
    );

    adi_a2b_cmdlist_log(list, false, "(r) ");
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        adi_a2b_cmdlist_log(list, false, "%02X ", *value);
    }
    adi_a2b_cmdlist_log(list, false, "\n");

    adi_a2b_cmdlist_log_error(list, result);

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_read_ctrl_reg_block(
    ADI_A2B_CMDLIST *list,
    uint32_t addr, uint8_t addrBytes,
    uint16_t len, uint8_t *values, uint8_t i2cAddr)
{
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    ADI_A2B_CMDLIST_RESULT result;
    uint8_t adr[4];

    if (addrBytes == 1) {
        adr[0] = addr & 0xFF;
    } else if (addrBytes == 2) {
        adr[0] = ((addr >> 8) & 0xFF);
        adr[1] = addr & 0xFF;
    } else if (addrBytes == 4) {
        adr[0] = ((addr >> 24) & 0xFF);
        adr[1] = ((addr >> 16) & 0xFF);
        adr[2] = ((addr >> 8) & 0xFF);
        adr[3] = addr & 0xFF;
    } else {
        return(ADI_A2B_CMDLIST_UNSUPPORTED_ADDR_BYTES);
    }

    result = cfg->twiWriteRead(
        cfg->handle, i2cAddr, adr, addrBytes, values, len, cfg->usr
    );

    adi_a2b_cmdlist_log_write_read(list, i2cAddr, adr, addrBytes, values, len);

    adi_a2b_cmdlist_log_error(list, result);

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_next_cmd(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_CMD *cmd)
{
    A2B_CMD *i2cCmd = NULL;
    A2B_CMD_SPI *spiCmd = NULL;
    size_t cmdLen = 0;

    if (list->cmdIdx >= list->cmdListLen) {
        return(ADI_A2B_CMDLIST_END);
    }

    if (list->cmdListType == A2B_CMD_TYPE_I2C) {
        cmdLen = sizeof(*i2cCmd);
        i2cCmd = list->cmdPtr;
    } else {
        cmdLen = sizeof(*spiCmd);
        spiCmd = list->cmdPtr;
    }

    if (i2cCmd) {
        cmd->deviceAddr = i2cCmd->nDeviceAddr;
        cmd->opCode = i2cCmd->eOpCode;
        cmd->addrWidth = i2cCmd->nAddrWidth;
        cmd->addr = i2cCmd->nAddr;
        cmd->dataWidth = i2cCmd->nDataWidth;
        cmd->dataCount = i2cCmd->nDataCount;
        cmd->configData = i2cCmd->paConfigData;
        cmd->spiCmdWidth = 0;
        cmd->spiCmd = 0;
        cmd->protocol = A2B_CMD_PROTO_I2C;
    } else {
        cmd->deviceAddr = spiCmd->nDeviceAddr;
        cmd->opCode = spiCmd->eOpCode;
        cmd->addrWidth = spiCmd->nAddrWidth;
        cmd->addr = spiCmd->nAddr;
        cmd->dataWidth = spiCmd->nDataWidth;
        cmd->dataCount = spiCmd->nDataCount;
        cmd->configData = spiCmd->paConfigData;
        cmd->spiCmdWidth = spiCmd->nSpiCmdWidth;
        cmd->spiCmd = spiCmd->nSpiCmd;
        cmd->protocol = spiCmd->eProtocol;
    }

    list->cmdPtr = (void *)((uintptr_t)list->cmdPtr + cmdLen);
    list->cmdIdx += cmdLen;
    list->cmdLine++;

    /* SPI Not currently supported */
    if (cmd->protocol == A2B_CMD_PROTO_SPI) {
        return(ADI_A2B_CMDLIST_UNSUPPORTED_PROTOCOL);
    }

    return(ADI_A2B_CMDLIST_SUCCESS);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_next(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_CMD *cmd,
    ADI_A2B_CMDLIST_ACCESS_TYPE *mode,
    uint8_t *reg, uint8_t *val, bool *singleReg, uint8_t node)
{
    ADI_A2B_CMDLIST_RESULT result;
    ADI_A2B_CMDLIST_ACCESS_TYPE _mode;
    ADI_A2B_CMDLIST_PRIV_NODE_INFO *privNodeInfo;
    uint8_t _reg, _val;
    bool _singleReg;

    /* Get the next command */
    result = adi_a2b_cmdlist_next_cmd(list, cmd);
    if (result != ADI_A2B_CMDLIST_SUCCESS) {
        return(result);
    }

    /* Check to see if this access is to the master or slave I2C address */
    if (cmd->deviceAddr == list->cmdListMasterAddr) {
        _mode = ADI_A2B_CMDLIST_MASTER_ACCESS;
    } else if (cmd->deviceAddr == list->cmdListMasterAddr + 1) {
        _mode = ADI_A2B_CMDLIST_SLAVE_ACCESS;
    } else if ((cmd->deviceAddr == 0x00) && (cmd->opCode == A2B_CMD_OP_DELAY)) {
        _mode = ADI_A2B_CMDLIST_NO_ACCESS;
    } else {
        _mode = ADI_A2B_CMDLIST_UNKNOWN_ACCESS;
    }

    /*
     * Pre-decode single register accesses
     */
    if ((cmd->dataCount == 1) && (cmd->addrWidth == 1)) {
        _reg = (uint8_t)(cmd->addr & 0xFF);
        _val = cmd->configData[0];
        _singleReg = true;
    } else {
        _reg = 0xFF;
        _val = 0;
        _singleReg = false;
    }

    /* Override master address if requested */
    if (list->oi.masterAddr_override) {
        if (_mode == ADI_A2B_CMDLIST_MASTER_ACCESS) {
            cmd->deviceAddr = list->oi.masterAddr;
        } else if (_mode == ADI_A2B_CMDLIST_SLAVE_ACCESS) {
            cmd->deviceAddr = list->oi.masterAddr + 1;
        }
    }

    /* Override master registers if requested */
    if ( (_singleReg) &&
         (_mode == ADI_A2B_CMDLIST_MASTER_ACCESS) &&
         (cmd->opCode == A2B_CMD_OP_WRITE) ) {
        switch (_reg) {
            case AD24xx_REG_I2SGCFG:
                _val = list->oi.I2SGCFG_override ? list->oi.I2SGCFG : _val;
                break;
            case AD24xx_REG_I2SCFG:
                _val = list->oi.I2SCFG_override ? list->oi.I2SCFG : _val;
                break;
            case AD24xx_REG_DNSLOTS:
                _val = list->oi.DNSLOTS_override ? list->oi.DNSLOTS : _val;
                break;
            case AD24xx_REG_UPSLOTS:
                _val = list->oi.UPSLOTS_override ? list->oi.UPSLOTS : _val;
                break;
            case AD24xx_REG_SLOTFMT:
                _val = list->oi.SLOTFMT_override ? list->oi.SLOTFMT : _val;
                break;
            case AD24xx_REG_RESPCYCS:
                _val = list->oi.RESPCYCS_override ? list->oi.RESPCYCS : _val;
                break;
            case AD24xx_REG_DISCVRY:
                _val = list->oi.sRESPCYCS_override[list->nodeIdx] ?
                           list->oi.sRESPCYCS[list->nodeIdx] : _val;
                list->nodeIdx++;
                break;
            case AD24xx_REG_DATCTL:
                _val = list->oi.DATCTL_override ? list->oi.DATCTL : _val;
                _val |= list->enableSniffing ? AD24xx_BITM_DATCTL_ENDSNIFF : 0;
                break;
            default:
                break;
        }
    }

    /*
     * Enforce ADI_A2B_CMDLIST_IOCTL_DISABLE_CLOCKS and
     * override slave registers if requested
     */
    if ( (_singleReg) &&
         (_mode == ADI_A2B_CMDLIST_SLAVE_ACCESS) &&
         (cmd->opCode == A2B_CMD_OP_WRITE) ) {
        privNodeInfo = &list->privNodeInfo[node];
        switch (_reg) {
            case AD24xx_REG_CLK1CFG:
            case AD24xx_REG_CLK2CFG:
            case AD24xx_REG_I2SCFG:
            case AD24xx_REG_PDMCTL:
                if (privNodeInfo->disableClocks) {
                    _mode = ADI_A2B_CMDLIST_BLOCK_ACCESS;
                }
                break;
            case AD24xx_REG_BCDNSLOTS:
                _val = list->oi.sBCDNSLOTS_override[node] ? list->oi.sBCDNSLOTS[node] : _val;
                break;
            case AD24xx_REG_DNSLOTS:
                _val = list->oi.sDNSLOTS_override[node] ? list->oi.sDNSLOTS[node] : _val;
                break;
            case AD24xx_REG_LDNSLOTS:
                _val = list->oi.sLDNSLOTS_override[node] ? list->oi.sLDNSLOTS[node] : _val;
                break;
            case AD24xx_REG_UPSLOTS:
                _val = list->oi.sUPSLOTS_override[node] ? list->oi.sUPSLOTS[node] : _val;
                break;
            case AD24xx_REG_LUPSLOTS:
                _val = list->oi.sLUPSLOTS_override[node] ? list->oi.sLUPSLOTS[node] : _val;
                break;
            default:
                break;
        }
    }

    *mode = _mode; *reg = _reg; *val = _val; *singleReg = _singleReg;

    return(ADI_A2B_CMDLIST_SUCCESS);
}

static uint8_t adi_a2b_cmdlist_master_address(ADI_A2B_CMDLIST *list)
{
    uint8_t masterAddr;

    masterAddr = list->oi.masterAddr_override ?
        list->oi.masterAddr : list->cmdListMasterAddr;

    return(masterAddr);
}


static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_full_bus_off(ADI_A2B_CMDLIST *list)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t masterAddr;

    masterAddr = adi_a2b_cmdlist_master_address(list);
    result = adi_a2b_cmdlist_write_ctrl_reg(
        list, AD24xx_REG_SWCTL, 0x00,
        masterAddr
    );

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_node_bus_off(ADI_A2B_CMDLIST *list)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[2] = { AD24xx_REG_SWCTL, 0x00 } ;

    if (list->nodesDiscovered == 0) {
        result = adi_a2b_cmdlist_full_bus_off(list);
    } else {
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, list->nodesDiscovered - 1, false, false, 0,
            wBuf, sizeof(wBuf), NULL, 0, 0);
    }

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_pwreien(ADI_A2B_CMDLIST *list)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t masterAddr;

    uint8_t wBuf[2] = { AD24xx_REG_INTMSK0, AD24xx_BITM_INTMSK0_PWREIEN } ;
    uint8_t rBuf[1];

    if (list->nodesDiscovered == 0) {
        masterAddr = adi_a2b_cmdlist_master_address(list);
        result = adi_a2b_cmdlist_read_ctrl_reg(
            list, wBuf[0], &rBuf[0],
            masterAddr
        );
        if (result != ADI_A2B_CMDLIST_SUCCESS) { goto abort; }
        wBuf[1] |= rBuf[0];
        result = adi_a2b_cmdlist_write_ctrl_reg(
            list, wBuf[0], wBuf[1],
            masterAddr
        );
        if (result != ADI_A2B_CMDLIST_SUCCESS) { goto abort; }
    } else {
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, list->nodesDiscovered - 1, false, false, 0,
            wBuf, 1, rBuf, 1, true
        );
        if (result != ADI_A2B_CMDLIST_SUCCESS) { goto abort; }
        wBuf[1] |= rBuf[0];
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, list->nodesDiscovered - 1, false, false, 0,
            wBuf, sizeof(wBuf), NULL, 0, 0
        );
        if (result != ADI_A2B_CMDLIST_SUCCESS) { goto abort; }
    }

abort:
    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_irq_poll(
    ADI_A2B_CMDLIST *list, uint8_t irqPending, uint32_t timeoutMs)
{
    ADI_A2B_CMDLIST_RESULT result;
    uint32_t start, elapsedMs;
    ADI_A2B_CMDLIST_IRQ_REGS irqRegs;
    bool polling;
    uint8_t masterAddr;
    bool discoveryDone;

    polling = true;
    discoveryDone = false;
    start = list->cfg.getTime(list->cfg.usr);
    masterAddr = adi_a2b_cmdlist_master_address(list);
    result = ADI_A2B_CMDLIST_SUCCESS;

    do {

        /* Perform atomic read of interrupt status registers */
        result = adi_a2b_cmdlist_read_ctrl_reg_block(list, AD24xx_REG_INTSRC, 1,
            sizeof(irqRegs), (uint8_t *)&irqRegs, masterAddr);
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            return(result);
        }

        /* Service a select group of possible interrupts */
        if (irqRegs.intSrc & (AD24xx_BITM_INTSRC_MSTINT | AD24xx_BITM_INTSRC_SLVINT)) {
            switch (irqRegs.intType) {
                case 0xFF:
                    if (irqPending == ADI_A2B_CMDLIST_IRQ_PLL_LOCK_PENDING) {
                        adi_a2b_cmdlist_delay(list, 10);
                        result = ADI_A2B_CMDLIST_SUCCESS;
                        polling = false;
                    }
                    break;
                case 0x18:
                    if (irqPending == ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING) {
                        result = ADI_A2B_CMDLIST_SUCCESS;
                        discoveryDone = true;
                        /* Do not terminate polling.  Line fault interrupt
                         * may come after discovery complete.
                         */
                    }
                    break;
                case 0x09:
                    adi_a2b_cmdlist_full_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_POS_SHORT_TO_GROUND;
                    list->faultDetected = true;
                    break;
                case 0x0A:
                    adi_a2b_cmdlist_full_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_NEG_SHORT_TO_VBAT;
                    list->faultDetected = true;
                    break;
                case 0x0B:
                    adi_a2b_cmdlist_node_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_SHORT_TOGETHER;
                    list->faultDetected = true;
                    break;
                case 0x0C:
                    adi_a2b_cmdlist_node_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_OPEN_OR_WRONG_PORT;
                    list->faultDetected = true;
                    break;
                case 0x0D:
                    adi_a2b_cmdlist_node_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_REVERSED_OR_WRONG_PORT;
                    list->faultDetected = true;
                    break;
                case 0x0E:
                    adi_a2b_cmdlist_node_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_REVERSED_OR_OPEN;
                    list->faultDetected = true;
                    break;
                case 0x29:
                    adi_a2b_cmdlist_full_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_SHORT_TO_GROUND;
                    list->faultDetected = true;
                    break;
                case 0x2A:
                    adi_a2b_cmdlist_full_bus_off(list);
                    result = ADI_A2B_CMDLIST_A2B_BUS_SHORT_TO_VBAT;
                    list->faultDetected = true;
                    break;
                default:
                    break;
            }
            if (list->faultDetected) {
                list->faultNode = list->nodesDiscovered;
                polling = false;
            }
        } else {
            /* Terminate discovery if no further interrupt pending discovery done */
            if ((irqPending == ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING) && discoveryDone) {
                list->nodesDiscovered++;
                if (list->cfg.event) {
                    list->cfg.event(
                        list, ADI_A2B_CMDLIST_EVENT_NODE_PRE_INIT,
                        list->nodesDiscovered - 1, NULL, list->cfg.usr
                    );
                }
                polling = false;
            }
        }

        /* Check polling inverval */
        if (polling) {
            elapsedMs = list->cfg.getTime(list->cfg.usr) - start;
            if (elapsedMs > timeoutMs) {
                result = ADI_A2B_CMDLIST_BUS_TIMEOUT;
                list->faultNode = list->nodesDiscovered;
                list->faultDetected = true;
                polling = false;
            } else {
                adi_a2b_cmdlist_delay(list, 5);
            }
        }

    } while (polling);

    return(result);
}

#ifdef ADI_A2B_CMDLIST_RESPCYCS_OVERRIDE
static uint8_t find_min(uint8_t *arr, size_t size)
{
    uint8_t min = arr[--size];
    while (size) {
        min = (arr[--size] < min) ? arr[size] : min;
    }
    return(min);
}

static uint8_t find_max(uint8_t *arr, size_t size)
{
    uint8_t max = arr[--size];
    while (size) {
        max = (arr[--size] > max) ? arr[size] : max;
    }
    return(max);
}

static uint8_t SLOTFMT_to_size(uint8_t fmt)
{
    uint8_t size;
    size = 8 + ((fmt & 0x07) * 4);
    if ((size == 24) && (fmt & 0x8)) {
        size += 6;
    } else if ((size == 32) && (fmt & 0x8)) {
        size += 7;
    } else {
        size += 1;
    }
    return(size);
}

/* Master override register select */
#define OVERRIDE(reg) \
    (oi->reg##_override ? oi->reg : list->reg)

/* Slave override register select*/
#define sOVERRIDE(reg,node) \
    (oi->s##reg##_override[node] ? oi->s##reg[node] : ni->reg)

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_respcycs_override(ADI_A2B_CMDLIST *list,
    ADI_A2B_CMDLIST_OVERRIDE_INFO *oi)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    ADI_A2B_CMDLIST_PRIV_NODE_INFO *ni;
    unsigned dnSlotActivity;
    unsigned upSlotActivity;
    uint8_t RESPCYCS_DN[ADI_A2B_CMDLIST_MAX_NODES];
    uint8_t RESPCYCS_UP[ADI_A2B_CMDLIST_MAX_NODES];
    uint8_t RESPOFFS, maxRespCycsDn, minRespCycsUp;
    uint8_t dnSize, upSize, dnSlots, upSlots;
    uint8_t reg, i;

    /* Bypass if not requested */
    if (list->respcycsFormula == RESPCYCS_FORMULA_UNKNOWN) {
        return(ADI_A2B_CMDLIST_SUCCESS);
    }

    /* Do not support RESPCYCS_FORMULA_B */
    if (list->respcycsFormula == RESPCYCS_FORMULA_B) {
        return(ADI_A2B_CMDLIST_RESPCYCS_ERROR);
    }

    /* Make sure the list has been scanned */
    if (!list->scanned) {
        result = adi_a2b_cmdlist_scan(list, NULL);
    }

    /* Calculate dn and up sizes */
    reg = OVERRIDE(SLOTFMT);
    dnSize = SLOTFMT_to_size((reg & 0x0f) >> 0);
    upSize = SLOTFMT_to_size((reg & 0xf0) >> 4);

    /* Calculate Main Node Response Offset */
    reg = OVERRIDE(I2SGCFG);
    if ((reg & 0x07) == 0) {
        RESPOFFS = (reg & 0x10) ? 238 : 245;
    } else if ((reg & 0x07) == 1) {
        RESPOFFS = (reg & 0x10) ? 245 : 248;
    } else {
        RESPOFFS = 248;
    }

    /*
     * Calculate the up and dn slot activity on the "A" side of
     * the node.  In the downstream direction, the "A" side is the "B" side
     * activity of the upstream node.  On the upstream side, the "A" side
     * activity is of the node itself.
     */
    for (i = 0; i < list->nodesScanned; i++) {

        /* Calculate node dn slot activity */
        if (i == 0) {
            /* Use the master node for the first slave */
            dnSlots = OVERRIDE(DNSLOTS);
        } else {
            /* Calculate previous node's dn slots */
            ni = &list->privNodeInfo[i-1];
            reg = sOVERRIDE(LDNSLOTS, i);
            dnSlots = (reg & 0x3f) + sOVERRIDE(DNSLOTS, i);
            dnSlots += (reg & 0x80) ? 0 : sOVERRIDE(BCDNSLOTS, i);
        }
        dnSlotActivity = (unsigned)dnSlots * (unsigned)dnSize;

        /* Calculate node up slot activity */
        ni = &list->privNodeInfo[i];
        upSlots = sOVERRIDE(UPSLOTS, i) + sOVERRIDE(LUPSLOTS, i);
        upSlotActivity = (unsigned)upSlots * (unsigned)upSize;

        /* Calculate node DN and UP respcycs */
        switch (list->respcycsFormula) {
            case RESPCYCS_FORMULA_A:
                RESPCYCS_DN[i] =
                    ceil(((64.0+(double)dnSlotActivity)/4.0)+4.0*(double)i+2.0);
                RESPCYCS_UP[i] =
                    ceil((double)RESPOFFS-(((64.0+(double)upSlotActivity)/4.0)+1.0));
                break;
            case RESPCYCS_FORMULA_B:
                /* Not supported */
                return(ADI_A2B_CMDLIST_RESPCYCS_ERROR);
                break;
            default:
                break;
        }
    }

    /* Find dn max and up min */
    maxRespCycsDn = find_max(RESPCYCS_DN, list->nodesScanned);
    minRespCycsUp = find_min(RESPCYCS_UP, list->nodesScanned);

    /* Calculate final RESPCYCS */
    oi->RESPCYCS_override = true;
    oi->RESPCYCS = ((unsigned)maxRespCycsDn + (unsigned)minRespCycsUp) / 2;

    for (i = 0; i < list->nodesScanned; i++) {
        switch (list->respcycsFormula) {
            case RESPCYCS_FORMULA_A:
                oi->sRESPCYCS_override[i] = true;
                oi->sRESPCYCS[i] = oi->RESPCYCS-4*i;
                break;
            default:
                break;
        }
    }

    return(ADI_A2B_CMDLIST_SUCCESS);
}
#endif

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_override(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_OVERRIDE_INFO *oi)
{
    if ((list == NULL) || (oi == NULL)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

#ifdef ADI_A2B_CMDLIST_RESPCYCS_OVERRIDE
    ADI_A2B_CMDLIST_RESULT result = adi_a2b_cmdlist_respcycs_override(list, oi);
    if (result != ADI_A2B_CMDLIST_SUCCESS) {
        return(result);
    }
#endif

    ADI_A2B_CMDLIST_MEMCPY(&list->oi, oi, sizeof(list->oi));

    return(ADI_A2B_CMDLIST_SUCCESS);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_get_delay(
    ADI_A2B_CMDLIST_CMD *cmd, uint32_t *delayMs)
{
    uint32_t delay;

    if (cmd->dataCount > 2) {
        return(ADI_A2B_CMDLIST_UNSUPPORTED_DATA_WIDTH);
    }

    /* Calculate the delay */
    delay = cmd->configData[0];
    if (cmd->dataCount == 2) {
        delay = (delay << 8) + cmd->configData[1];
    }

    /* Return delay */
    if (delayMs) {
        *delayMs = delay;
    }

    return(ADI_A2B_CMDLIST_SUCCESS);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_scan(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_SCAN_INFO *scan)
{
    ADI_A2B_CMDLIST_CMD cmd;
    ADI_A2B_CMDLIST_ACCESS_TYPE mode;
    ADI_A2B_CMDLIST_RESULT result;
    bool singleReg;
    uint8_t nodeAddr;
    uint8_t nodeIdx;
    uint8_t reg;
    uint8_t val;
    uint8_t irqPending;
    uint32_t delayMs;

    if (list == NULL) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

    adi_a2b_cmdlist_reset(list);

    if (scan) {
        ADI_A2B_CMDLIST_MEMSET(scan, 0, sizeof(*scan));
    }

    irqPending = ADI_A2B_CMDLIST_IRQ_NONE;
    nodeIdx = 0;
    nodeAddr = 0;

    while (1) {
        result = adi_a2b_cmdlist_next(list, &cmd, &mode, &reg, &val, &singleReg, nodeAddr);
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /* Track node discovery timeout */
        if ( (cmd.opCode == A2B_CMD_OP_DELAY) &&
             (irqPending == ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING) ) {
                result = adi_a2b_cmdlist_get_delay(&cmd, &delayMs);
                if (result != ADI_A2B_CMDLIST_SUCCESS) {
                    break;
                }
                list->privNodeInfo[nodeIdx-1].timeoutMs = delayMs;
        }
        irqPending = ADI_A2B_CMDLIST_IRQ_NONE;

        /* Track interesting register writes */
        if ( (singleReg) &&
             (cmd.opCode == A2B_CMD_OP_WRITE) ) {

            /* Track main node registers for override and partial discovery */
            if (mode == ADI_A2B_CMDLIST_MASTER_ACCESS) {
                switch (reg) {
                    case AD24xx_REG_I2SGCFG:
                        if (scan) {
                            scan->I2SGCFG = val;
                            scan->I2SGCFG_valid = true;
                        }
                        list->I2SGCFG = val;
                        break;
                    case AD24xx_REG_I2SCFG:
                        if (scan) {
                            scan->I2SCFG = val;
                            scan->I2SCFG_valid = true;
                        }
                        list->I2SCFG = val;
                        break;
                    case AD24xx_REG_DNSLOTS:
                        if (scan) {
                            scan->DNSLOTS = val;
                            scan->DNSLOTS_valid = true;
                        }
                        list->DNSLOTS = val;
                        break;
                    case AD24xx_REG_UPSLOTS:
                        if (scan) {
                            scan->UPSLOTS = val;
                            scan->UPSLOTS_valid = true;
                        }
                        list->UPSLOTS = val;
                        break;
                    case AD24xx_REG_SLOTFMT:
                        if (scan) {
                            scan->SLOTFMT = val;
                            scan->SLOTFMT_valid = true;
                        }
                        list->SLOTFMT = val;
                        break;
                    case AD24xx_REG_DATCTL:
                        if (scan) {
                            scan->DATCTL = val;
                            scan->DATCTL_valid = true;
                        }
                        list->DATCTL = val;
                        break;
                    case AD24xx_REG_I2STXOFFSET:
                        list->I2STXOFFSET = val;
                        break;
                    case AD24xx_REG_NODEADR:
                        nodeAddr = val & AD24xx_BITM_NODEADR_NODE;
                        break;
                    case AD24xx_REG_DISCVRY:
                        list->privNodeInfo[nodeIdx++].respCycs = val;
                        irqPending = ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING;
                        break;
                    case AD24xx_REG_RESPCYCS:
                        if (scan) {
                            scan->RESPCYCS = val;
                            scan->RESPCYCS_valid = true;
                        }
                        break;
                    default:
                        break;
                }
            }

            /*
             * Track subnode registers for partial rediscovery and
             * RESPCYCS calculations.
             */
            if (mode == ADI_A2B_CMDLIST_SLAVE_ACCESS) {
                switch (reg) {
                    case AD24xx_REG_UPSLOTS:
                        list->privNodeInfo[nodeAddr].UPSLOTS = val;
                        break;
                    case AD24xx_REG_LUPSLOTS:
                        list->privNodeInfo[nodeAddr].LUPSLOTS = val;
                        break;
                    case AD24xx_REG_BCDNSLOTS:
                        list->privNodeInfo[nodeAddr].BCDNSLOTS = val;
                        break;
                    case AD24xx_REG_DNSLOTS:
                        list->privNodeInfo[nodeAddr].DNSLOTS = val;
                        break;
                    case AD24xx_REG_LDNSLOTS:
                        list->privNodeInfo[nodeAddr].LDNSLOTS = val;
                        break;
                    default:
                        break;
                }

            }
        }
    }

    if (result == ADI_A2B_CMDLIST_END) {
        list->nodesScanned = nodeIdx;
        list->scanned = true;
        result = ADI_A2B_CMDLIST_SUCCESS;
    }

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_play(ADI_A2B_CMDLIST *list)
{
    ADI_A2B_CMDLIST_CMD cmd;
    ADI_A2B_CMDLIST_RESULT result;
    uint8_t readbuf[ADI_A2B_CMDLIST_READBUF_LEN];
    uint32_t delayMs;

    if (list == NULL) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

    adi_a2b_cmdlist_reset(list);

    while (1) {
        result = adi_a2b_cmdlist_next_cmd(list, &cmd);
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /* Process command */
        switch (cmd.opCode) {

            /*********************************************************************
             * Write command
             ********************************************************************/
            case A2B_CMD_OP_WRITE:
                result = adi_a2b_cmdlist_write_ctrl_reg_block(list,
                    cmd.addr, cmd.addrWidth,
                    cmd.dataCount, cmd.configData, cmd.deviceAddr
                );
                break;

            /*********************************************************************
             * Read command
             ********************************************************************/
            case A2B_CMD_OP_READ:
                if (cmd.dataCount <= ADI_A2B_CMDLIST_READBUF_LEN) {
                    result = adi_a2b_cmdlist_read_ctrl_reg_block(list,
                        cmd.addr, cmd.addrWidth, cmd.dataCount, readbuf, cmd.deviceAddr
                    );
                } else {
                    result = ADI_A2B_CMDLIST_UNSUPPORTED_READ_LENGTH;
                }
                break;

            /*********************************************************************
             * Delay command
             ********************************************************************/
            case A2B_CMD_OP_DELAY:
                result = adi_a2b_cmdlist_get_delay(&cmd, &delayMs);
                if (result != ADI_A2B_CMDLIST_SUCCESS) {
                    break;
                }
                adi_a2b_cmdlist_delay(list, delayMs);
                break;

            default:
                break;

        }

        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }
    }

    if (result == ADI_A2B_CMDLIST_END) {
        result = ADI_A2B_CMDLIST_SUCCESS;
    }

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_get_node_info(
    ADI_A2B_CMDLIST *list, uint8_t node, ADI_A2B_CMDLIST_NODE_INFO *nodeInfo)
{
    if ((list == NULL) || (nodeInfo == NULL) ||
        (node >= list->nodesDiscovered) ||
        (node >= ADI_A2B_CMDLIST_MAX_NODES)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }
    ADI_A2B_CMDLIST_MEMCPY(nodeInfo, &list->nodeInfo[node], sizeof(*nodeInfo));
    return(ADI_A2B_CMDLIST_SUCCESS);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_node_twi_transfer(
    ADI_A2B_CMDLIST *list, int8_t node,
    bool peripheral, bool broadcast, uint8_t address,
    void *out, uint16_t outLen,  void *io, uint16_t ioLen, bool ioDir)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    ADI_A2B_CMDLIST_CFG *cfg = &list->cfg;
    uint8_t masterAddr;
    uint8_t value;

    /* Check parameters */
#ifndef ADI_A2B_CMDLIST_EXPERT_MODE
    if ((list == NULL) ||
        (node >= list->nodesDiscovered) ||
        (node >= ADI_A2B_CMDLIST_MAX_NODES)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }
#endif
    if (peripheral && broadcast) {
        return(ADI_A2B_CMDLIST_ERROR);
    }
    if ((out == NULL) && (io == NULL)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

    /* Get the working master TWI address */
    masterAddr = adi_a2b_cmdlist_master_address(list);

    /* Set the remote node CHIP register if peripheral access */
    if (node >= 0) {
        if (peripheral) {
            if (result == ADI_A2B_CMDLIST_SUCCESS) {
                result = adi_a2b_cmdlist_write_ctrl_reg(
                    list, AD24xx_REG_NODEADR, node & AD24xx_BITM_NODEADR_NODE,
                    masterAddr
                );
            }
            if (result == ADI_A2B_CMDLIST_SUCCESS) {
                result = adi_a2b_cmdlist_write_ctrl_reg(
                    list, AD24xx_REG_CHIP, address,
                    masterAddr + 1
                );
            }
        }

        /* Set the master NODEADR register for this transaction */
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            value = node & AD24xx_BITM_NODEADR_NODE;
            value |= peripheral ? AD24xx_BITM_NODEADR_PERI : 0x00;
            value |= broadcast ? AD24xx_BITM_NODEADR_BRCST : 0x00;
            result = adi_a2b_cmdlist_write_ctrl_reg(
                list, AD24xx_REG_NODEADR, value, masterAddr
            );
        }

        /* Transact with the sub-node from here on */
        masterAddr += 1;
    }

    /* Perform the TWI transaction */
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        if (out && io) {
            if (ioDir) {
                result = cfg->twiWriteRead(
                    cfg->handle, masterAddr,
                    out, outLen, io, ioLen,
                    cfg->usr
                );
                adi_a2b_cmdlist_log_write_read(list, masterAddr,
                    out, outLen, io, ioLen
                );
            } else {
                result = adi_a2b_cmdlist_write_write(
                    list, masterAddr,
                    out, outLen, io, ioLen
                );
                // Logging is built into adi_a2b_cmdlist_write_write()
            }
        } else if (out) {
            result = cfg->twiWrite(
                cfg->handle, masterAddr,
                out, outLen,
                cfg->usr
            );
            adi_a2b_cmdlist_log_write(list, masterAddr, out, outLen, NULL, 0);
        } else {
            result = cfg->twiRead(
                cfg->handle, masterAddr,
                io, ioLen,
                cfg->usr
            );
            adi_a2b_cmdlist_log_read(list, masterAddr, io, ioLen);
        }
    }

    return(result);
}


ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_set(
    ADI_A2B_CMDLIST *list, uint8_t cmdListMasterAddr,
    void *cmdList, uint32_t cmdListLen, A2B_CMD_TYPE cmdListType)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;

#ifndef ADI_A2B_CMDLIST_EXPERT_MODE
    if ((list == NULL) || (cmdList == NULL) || (cmdListLen == 0)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }
    if ((cmdListType != A2B_CMD_TYPE_I2C) &&
        (cmdListType != A2B_CMD_TYPE_SPI)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }
#endif

    list->cmdList = cmdList;
    list->cmdListLen = cmdListLen;
    list->cmdListType = cmdListType;
    list->cmdListMasterAddr = cmdListMasterAddr;

    adi_a2b_cmdlist_reset(list);

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_opcode(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_CMD *cmd,
    bool singleReg, uint8_t reg, uint8_t val, uint8_t *readbuf,
    uint8_t irqPending, uint32_t *delayMs)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint32_t delay;

    /* Process command */
    switch (cmd->opCode) {

        /*********************************************************************
         * Write command
         ********************************************************************/
        case A2B_CMD_OP_WRITE:
            if (singleReg) {
                result = adi_a2b_cmdlist_write_ctrl_reg(list,
                    reg, val, cmd->deviceAddr
                );
            } else {
                result = adi_a2b_cmdlist_write_ctrl_reg_block(list,
                    cmd->addr, cmd->addrWidth,
                    cmd->dataCount, cmd->configData, cmd->deviceAddr
                );
            }
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }
            break;

        /*********************************************************************
         * Read command
         ********************************************************************/
        case A2B_CMD_OP_READ:
            if (readbuf) {
                if (cmd->dataCount <= ADI_A2B_CMDLIST_READBUF_LEN) {
                    result = adi_a2b_cmdlist_read_ctrl_reg_block(list,
                        cmd->addr, cmd->addrWidth, cmd->dataCount, readbuf, cmd->deviceAddr
                    );
                    if (result != ADI_A2B_CMDLIST_SUCCESS) {
                        break;
                    }
                } else {
                    result = ADI_A2B_CMDLIST_UNSUPPORTED_READ_LENGTH;
                    break;
                }
            }
            break;

        /*********************************************************************
         * Delay command
         ********************************************************************/
        case A2B_CMD_OP_DELAY:
            result = adi_a2b_cmdlist_get_delay(cmd, &delay);
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }

            /* Only do straight delays here. */
            if (irqPending == ADI_A2B_CMDLIST_IRQ_NONE) {
                adi_a2b_cmdlist_delay(list, delay);
            }

            /* Return delay */
            if (delayMs) {
                *delayMs = delay;
            }
            break;

        default:
            break;

    }

    return(result);
}

static void adi_a2b_cmdlist_set_results(ADI_A2B_CMDLIST *list,
    ADI_A2B_CMDLIST_EXECUTE_INFO *results, ADI_A2B_CMDLIST_RESULT result)
{
    adi_a2b_cmdlist_log(
        list, true, "Lines Processed: %d\n", list->cmdLine
    );
    adi_a2b_cmdlist_log(
        list, true, "Nodes Discovered: %d\n", list->nodesDiscovered
    );
    adi_a2b_cmdlist_log(
        list, true, "Fault Detected: %s\n",
        list->faultDetected ? "Yes" : "No"
    );
    if (list->faultDetected) {
        adi_a2b_cmdlist_log(
            list, true, "Fault Node: %d\n", list->faultNode
        );
    }
    adi_a2b_cmdlist_log(
        list, true, "Result: %s\n", adi_a2b_cmdlist_result_str(result)
    );
    if (results) {
        results->linesProcessed = list->cmdLine;
        results->nodesDiscovered = list->nodesDiscovered;
        results->faultDetected = list->faultDetected;
        results->faultNode = list->faultNode;
        results->faultType = result;
        results->resultStr = adi_a2b_cmdlist_result_str(result);
    }

    adi_a2b_cmdlist_log_error(list, result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_execute(
    ADI_A2B_CMDLIST *list, ADI_A2B_CMDLIST_EXECUTE_INFO *results)
{
    ADI_A2B_CMDLIST_CMD cmd;
    ADI_A2B_CMDLIST_RESULT result;
    ADI_A2B_CMDLIST_ACCESS_TYPE mode;
    uint8_t readbuf[ADI_A2B_CMDLIST_READBUF_LEN];
    bool singleReg;
    uint8_t nodeAddr;
    uint8_t reg;
    uint8_t val;
    uint32_t delayMs;
    uint8_t irqPending;
    bool masterMode;

    adi_a2b_cmdlist_reset(list);
    adi_a2b_cmdlist_discover_restart(list);

    irqPending = ADI_A2B_CMDLIST_IRQ_NONE;
    result = ADI_A2B_CMDLIST_ERROR;
    masterMode = false;
    delayMs = 0;
    nodeAddr = 0;

    ADI_A2B_CMDLIST_MEMSET(list->nodeInfo, 0, sizeof(list->nodeInfo));
    result = ADI_A2B_CMDLIST_SUCCESS;

    adi_a2b_cmdlist_log(list, true, "Begin discovery\n");

    while (result == ADI_A2B_CMDLIST_SUCCESS) {

        result = adi_a2b_cmdlist_next(list, &cmd, &mode, &reg, &val, &singleReg, nodeAddr);
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /* Skip anything unknown or blocked */
        if ( (mode == ADI_A2B_CMDLIST_UNKNOWN_ACCESS) ||
             (mode == ADI_A2B_CMDLIST_BLOCK_ACCESS) ) {
            continue;
        }

        /*
         * Track the state of a select set of master register writes
         * for limited error detection, or IRQ handling during the next
         * delay interval.
         */
        if ( (singleReg) &&
             (cmd.opCode == A2B_CMD_OP_WRITE) ) {

            if (mode == ADI_A2B_CMDLIST_MASTER_ACCESS) {

                irqPending = ADI_A2B_CMDLIST_IRQ_NONE;

                /* Track PLL lock IRQ following master mode set */
                if (reg == AD24xx_REG_CONTROL) {
                    if (val & AD24xx_BITM_CONTROL_MSTR) {
                        if (!masterMode) {
                            irqPending = ADI_A2B_CMDLIST_IRQ_PLL_LOCK_PENDING;
                        }
                        masterMode = true;
                    } else {
                        masterMode = false;
                    }
                /*
                 * Track discovery IRQ and unconditionally enable SW diag
                 * interrupt on slave modes prior to discovery.
                 */
                } else if (reg == AD24xx_REG_DISCVRY) {
                    result = adi_a2b_cmdlist_pwreien(list);
                    irqPending = ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING;
#ifdef ADI_A2B_CMDLIST_FORCE_FULL_LINE_DIAGNOSTICS
                /* Force full line diagnostics  */
                } else if (reg == AD24xx_REG_SWCTL) {
                    val &= ~0x30;
#endif
                /* Track AD24xx_REG_NODEADR */
                } else if (reg == AD24xx_REG_NODEADR) {
                    nodeAddr = val & AD24xx_BITM_NODEADR_NODE;
                /* Nothing special happening  */
                } else {
                    // Empty
                }

            }
        }

        /* Exit on error */
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /* Process command */
        result = adi_a2b_cmdlist_opcode(
            list, &cmd, singleReg, reg, val, readbuf, irqPending, &delayMs
        );

        /* Exit on error */
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /*
         * Perform more advanced IRQ handling during delay intervals
         */
        if ( (cmd.opCode == A2B_CMD_OP_DELAY) &&
             (irqPending != ADI_A2B_CMDLIST_IRQ_NONE) ) {
            result = adi_a2b_cmdlist_irq_poll(list, irqPending, delayMs);
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }
        }

        /*
         * Track limited slave node information
         */
        if ( (mode == ADI_A2B_CMDLIST_SLAVE_ACCESS) &&
             (cmd.opCode == A2B_CMD_OP_READ) ) {
            if (list->nodesDiscovered > 0) {
                uint8_t currentNode = list->nodesDiscovered - 1;
                for (int i = 0; i < cmd.dataCount; i++) {
                    if (currentNode < ADI_A2B_CMDLIST_MAX_NODES) {
                        switch ((cmd.addr + i) & 0xFF) {
                            case AD24xx_REG_VENDOR:
                                list->nodeInfo[currentNode].vendor  = readbuf[i];
                                break;
                            case AD24xx_REG_PRODUCT:
                                list->nodeInfo[currentNode].product  = readbuf[i];
                                break;
                            case AD24xx_REG_VERSION:
                                list->nodeInfo[currentNode].version  = readbuf[i];
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
        }
    }

    /* Reaching the end is success */
    if (result == ADI_A2B_CMDLIST_END) {
        result = ADI_A2B_CMDLIST_SUCCESS;
    }

    /* Send init events for all discovered nodes */
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        for (int i = 0; i < list->nodesDiscovered; i++) {
            if (list->cfg.event) {
                list->cfg.event(
                    list, ADI_A2B_CMDLIST_EVENT_NODE_INIT,
                    i, &list->nodeInfo[i], list->cfg.usr
                );
            }
        }
    }

    /* Pass along the results */
    adi_a2b_cmdlist_set_results(list, results, result);

    adi_a2b_cmdlist_log(
        list, true, "End discovery\n"
    );

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_delay(
    ADI_A2B_CMDLIST *list, uint32_t ms)
{
    adi_a2b_cmdlist_log(list, true, "(d) %lu\n", (long unsigned)ms);
    list->cfg.delay(ms, list->cfg.usr);
    return(ADI_A2B_CMDLIST_SUCCESS);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_init(void)
{
    ADI_A2B_CMDLIST_MEMSET(cmdList, 0, sizeof(cmdList));
    return(ADI_A2B_CMDLIST_SUCCESS);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_open(
    ADI_A2B_CMDLIST **list, ADI_A2B_CMDLIST_CFG *cfg)
{
    ADI_A2B_CMDLIST *l = NULL;
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_ERROR;
    unsigned i;

    if ((list == NULL) || (cfg == NULL)) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

    if ((cfg->twiWrite == NULL) || (cfg->twiWriteRead == NULL) ||
        (cfg->delay == NULL) || (cfg->getTime == NULL) ||
        (cfg->getBuffer == NULL) || (cfg->freeBuffer == NULL)) {
        return(ADI_A2B_CMDLIST_CFG_ERROR);
    }

    ADI_A2B_CMDLIST_ENTER_CRITICAL();
    for (i = 0; i < ADI_A2B_CMDLIST_MAX_LISTS; i++) {
        if (cmdList[i].busy == false) {
            l = &cmdList[i];
            l->busy = true;
            break;
        }
    }
    ADI_A2B_CMDLIST_EXIT_CRITICAL();

    if (l) {
        ADI_A2B_CMDLIST_MEMCPY(&l->cfg, cfg, sizeof(l->cfg));
        result = ADI_A2B_CMDLIST_SUCCESS;
        *list = l;
    } else {
        result = ADI_A2B_CMDLIST_ERROR;
        *list = NULL;
    }

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_close(
    ADI_A2B_CMDLIST **list)
{
    if (list == NULL) {
        return(ADI_A2B_CMDLIST_ERROR);
    }

    adi_a2b_cmdlist_log(*list, true, NULL);

    ADI_A2B_CMDLIST_ENTER_CRITICAL();
    ADI_A2B_CMDLIST_MEMSET(*list, 0, sizeof(**list));
    ADI_A2B_CMDLIST_EXIT_CRITICAL();
    *list = NULL;

    return(ADI_A2B_CMDLIST_SUCCESS);
}

bool adi_a2b_cmdlist_full_bus_discovered(ADI_A2B_CMDLIST *list)
{
    return (list->nodesDiscovered == list->nodesScanned);
}

/***********************************************************************
 * Partial discovery
 **********************************************************************/
static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_control(
    ADI_A2B_CMDLIST *list, uint8_t bit)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[2];
    uint8_t rBuf[1];

    wBuf[0] = AD24xx_REG_CONTROL;
    result = adi_a2b_cmdlist_node_twi_transfer(
        list, -1, false, false, 0,
        wBuf, 1, rBuf, 1, 1
    );
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        wBuf[1] = rBuf[0] | bit;
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, -1, false, false, 0,
            wBuf, 2, NULL, 0, 0
        );
    }

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_update_swctl(
    ADI_A2B_CMDLIST *list, uint8_t swctl, bool doSwctlLast, uint8_t swctlLast)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[2];
    int i;

   // Set all good nodes SWCTL registers except last good node
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        for (i = -1; i < list->nodesDiscovered - 1; i++) {
            wBuf[0] = AD24xx_REG_SWCTL;
            wBuf[1] = swctl;
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, i, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }
        }
    }

    // Set last good nodes SWCTL register
    if ((result == ADI_A2B_CMDLIST_SUCCESS) && doSwctlLast) {
        wBuf[0] = AD24xx_REG_SWCTL;
        wBuf[1] = swctlLast;
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, list->nodesDiscovered - 1, false, false, 0,
            wBuf, 2, NULL, 0, 0
        );
    }

    return(result);
}

#ifdef ADI_A2B_CMDLIST_PARTIAL_RESET_RESP_CYCLES
static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_update_respcycs(
    ADI_A2B_CMDLIST *list, int8_t node)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[2];
    int i, j;

    for (j = 1; j >= 0; j--) {
        wBuf[0] = AD24xx_REG_RESPCYCS;
        for (i = -1; i <= node; i++) {
            if (i == -1) {
                wBuf[1] = list->privNodeInfo[0].respCycs + j;
            } else {
                wBuf[1] = list->privNodeInfo[i].respCycs + j;
            }
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, i, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }
        }
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            result = adi_a2b_cmdlist_control(list,
                AD24xx_BITM_CONTROL_NEWSTRCT);
        }
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }
        adi_a2b_cmdlist_delay(list, 1);
    }

    return(result);
}
#endif

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_discover_nodes(
    ADI_A2B_CMDLIST *list, int8_t node)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t currentNode;
    uint8_t wBuf[2];
    uint8_t rBuf[3];

    // Start from the last good node
    list->nodesDiscovered = node + 1;

    do {

        // Set all good nodes SWCTL register to 0x21 except last good node
        // Set last good nodes SWCTL register to 0x01
        result = adi_a2b_cmdlist_update_swctl(list,
            AD24xx_ENUM_SWCTL_MODE2 | AD24xx_BITM_SWCTL_ENSW,
            true, AD24xx_BITM_SWCTL_ENSW
        );

        // Set AD24xx_REG_NODEADR to last good node
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_NODEADR;
            wBuf[1] = list->nodesDiscovered - 1;
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, -1, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }

        // expected response cycle of subordinate node to discover
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_DISCVRY;
            wBuf[1] = list->privNodeInfo[list->nodesDiscovered].respCycs;
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, -1, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }

        // Wait for discovery complete
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            result = adi_a2b_cmdlist_irq_poll(
                list, ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING,
                list->privNodeInfo[list->nodesDiscovered].timeoutMs
            );
            // Get the node information
            if (result == ADI_A2B_CMDLIST_SUCCESS) {
                currentNode = list->nodesDiscovered - 1;
                wBuf[0] = AD24xx_REG_VENDOR;
                result = adi_a2b_cmdlist_node_twi_transfer(
                    list, currentNode, false, false, 0,
                    wBuf, 1, rBuf, 3, 1
                );
                if (result == ADI_A2B_CMDLIST_SUCCESS) {
                    list->nodeInfo[currentNode].vendor = rBuf[0];
                    list->nodeInfo[currentNode].product = rBuf[1];
                    list->nodeInfo[currentNode].version = rBuf[2];
                }
            }
        }

    } while ( (result == ADI_A2B_CMDLIST_SUCCESS) &&
              (list->nodesDiscovered < list->nodesScanned) );

    return(result);
}


static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_reconfigure_node(
    ADI_A2B_CMDLIST *list, int8_t node)
{
    ADI_A2B_CMDLIST_CMD cmd;
    ADI_A2B_CMDLIST_ACCESS_TYPE mode;
    ADI_A2B_CMDLIST_RESULT result;
    uint8_t readbuf[ADI_A2B_CMDLIST_READBUF_LEN];
    uint8_t irqPending;
    bool singleReg;
    uint8_t nodeAddr = 0;
    uint8_t reg;
    uint8_t val;
    bool doNodeAddr;
    bool doCmd;
    bool doDelay;

    adi_a2b_cmdlist_reset(list);

    adi_a2b_cmdlist_log(
        list, true, "Begin node %d partial config\n", node
    );

    /*
     * Perform the node setup
     */
    while (1) {

        /* Fetch next command */
        result = adi_a2b_cmdlist_next(list, &cmd, &mode, &reg, &val, &singleReg, node);
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }

        /* Skip anything unknown or blocked */
        if ( (mode == ADI_A2B_CMDLIST_UNKNOWN_ACCESS) ||
             (mode == ADI_A2B_CMDLIST_BLOCK_ACCESS) ) {
            continue;
        }

        /* Process delays normally */
        irqPending = ADI_A2B_CMDLIST_IRQ_NONE;

        /* Don't set AD24xx_REG_NODEADR */
        doNodeAddr = false;

        /* Do the command by default */
        doCmd = true;

        /* Don't delay by default */
        doDelay = false;

        if (node == -1) {
            if ( (mode == ADI_A2B_CMDLIST_MASTER_ACCESS) &&
                 (cmd.opCode == A2B_CMD_OP_WRITE) ) {
                switch (reg) {
                    case AD24xx_REG_CONTROL:
                    case AD24xx_REG_NODEADR:
                    case AD24xx_REG_SWCTL:
                    case AD24xx_REG_SLOTFMT:
                    case AD24xx_REG_DATCTL:
                    case AD24xx_REG_DISCVRY:
                        doCmd = false;
                    default:
                        break;
                }
            } else {
                doCmd = false;
            }
        } else {
            /* Track master register writes */
            if ( (singleReg) &&
                 (mode == ADI_A2B_CMDLIST_MASTER_ACCESS) &&
                 (cmd.opCode == A2B_CMD_OP_WRITE) ) {
                /* Track AD24xx_REG_NODEADR */
                if (reg == AD24xx_REG_NODEADR) {
                    nodeAddr = val & AD24xx_BITM_NODEADR_NODE;
                    if (nodeAddr == node) {
                        doNodeAddr = true;
                    }
                /* Track AD24xx_REG_DISCVRY */
                } else if (reg == AD24xx_REG_DISCVRY) {
                    irqPending = ADI_A2B_CMDLIST_IRQ_DISCOVERY_PENDING;
                }
            }

            /* Filter out all SWCTL commands */
            if (reg == AD24xx_REG_SWCTL) {
                doCmd = false;
            }

            /* Do delays for this node */
            if ((cmd.opCode == A2B_CMD_OP_DELAY) && (nodeAddr == node)) {
                doDelay = true;
            }

            /* Play commands for this node */
            if ( (doNodeAddr) || (doDelay) ||
                 ((mode == ADI_A2B_CMDLIST_SLAVE_ACCESS) && (nodeAddr == node) && (doCmd)) ) {
                doCmd = true;
            } else {
                doCmd = false;
            }
        }

        /* Execute command */
        if (doCmd) {
            result = adi_a2b_cmdlist_opcode(
                list, &cmd, singleReg, reg, val, readbuf, irqPending, NULL);
        }

        /* Exit on error */
        if (result != ADI_A2B_CMDLIST_SUCCESS) {
            break;
        }
    }

    if (result == ADI_A2B_CMDLIST_END) {
        result = ADI_A2B_CMDLIST_SUCCESS;
    }

    /* Send init event for this node */
    if ((result == ADI_A2B_CMDLIST_SUCCESS) && (node >= 0)) {
        if (list->cfg.event) {
            list->cfg.event(
                list, ADI_A2B_CMDLIST_EVENT_NODE_INIT,
                node, &list->nodeInfo[node], list->cfg.usr
            );
        }
    }

    adi_a2b_cmdlist_log(
        list, true, "End node %d partial config\n", node
    );

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_reconfigure_nodes(
    ADI_A2B_CMDLIST *list, int8_t node)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    int8_t n;

    for (n = list->nodesDiscovered - 1; n > node; n--) {
        result = adi_a2b_cmdlist_reconfigure_node(list, n);
        if ((result != ADI_A2B_CMDLIST_SUCCESS) && list->abortOnError) {
            return(result);
        }
    }

    if (node == -1) {
        result = adi_a2b_cmdlist_reconfigure_node(list, n);
    }

    return(result);
}

static ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_finalize(
    ADI_A2B_CMDLIST *list)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[3];

    result = adi_a2b_cmdlist_update_swctl(list,
        AD24xx_BITM_SWCTL_ENSW,
        false, 0x00
    );

    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        wBuf[0] = AD24xx_REG_SLOTFMT;
        wBuf[1] = list->SLOTFMT;
        wBuf[2] = list->DATCTL;
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, -1, false, false, 0,
            wBuf, 3, NULL, 0, 0
        );
    }

    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        result = adi_a2b_cmdlist_control(list,
            AD24xx_BITM_CONTROL_NEWSTRCT);
        adi_a2b_cmdlist_delay(list, 1);
    }

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_remap_slots(
    ADI_A2B_CMDLIST *list, uint8_t node,
    ADI_A2B_CMDLIST_SLOT_REMAP type, bool apply)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t lostSlots;
    uint8_t offset;
    uint8_t wBuf[2];
    int8_t i;

    // Make sure the list has been scanned
    if (!list->scanned) {
        result = adi_a2b_cmdlist_scan(list, NULL);
    }

    // Confirm this node was discovered
    if (node > (list->nodesDiscovered - 1)) {
        return(ADI_A2B_CMDLIST_A2B_BAD_NODE);
    }

    adi_a2b_cmdlist_log(
        list, true, "Begin remap slots\n"
    );

    /*
     * If apply == true, compensate for lost slots downstream from 'node'
     * in upstream node(s).
     */
    if (apply) {
        lostSlots = list->privNodeInfo[node].UPSLOTS;
    } else {
        lostSlots = 0;
    }

    if (type == ADI_A2B_CMDLIST_SLOT_REMAP_FULL) {
        /* Adjust UPSLOTS on all upstream transceivers */
        wBuf[0] = AD24xx_REG_UPSLOTS;
        for (i = node; i >= -1; i--) {
            if (i == -1) {
                wBuf[1] = list->UPSLOTS - lostSlots;
            } else {
                wBuf[1] = list->privNodeInfo[i].UPSLOTS - lostSlots;
            }
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, i, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                break;
            }
        }
    } else if (type == ADI_A2B_CMDLIST_SLOT_REMAP_FAST) {
        /* Adjust UPSLOTS and LUPSLOTS on only the last node */
        wBuf[0] = AD24xx_REG_UPSLOTS;
        if (apply) {
            wBuf[1] = 0x00;
        } else {
            wBuf[1] = list->privNodeInfo[node].UPSLOTS;
        }
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, node, false, false, 0,
            wBuf, 2, NULL, 0, 0
        );
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_LUPSLOTS;
            if (apply) {
                wBuf[1] = list->privNodeInfo[node].LUPSLOTS + lostSlots;
            } else {
                wBuf[1] = list->privNodeInfo[node].LUPSLOTS;
            }
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, node, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }
    } else {
        result = ADI_A2B_CMDLIST_ERROR;
    }

    // Send NEWSTRCT
    if (result == ADI_A2B_CMDLIST_SUCCESS) {
        result = adi_a2b_cmdlist_control(list, AD24xx_BITM_CONTROL_NEWSTRCT);
        adi_a2b_cmdlist_delay(list, 1);
    }

    if (type == ADI_A2B_CMDLIST_SLOT_REMAP_FULL) {
        // Offset TDM by lost slots
        if (apply) {
            offset = list->I2STXOFFSET + lostSlots;
        } else {
            offset = list->I2STXOFFSET;
        }

        // Zero Tx portion of I2SCFG
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_I2SCFG;
            wBuf[1] = list->I2SCFG & ~(AD24xx_BITM_I2SCFG_TX);
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, -1, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }

        // Apply offset
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_I2STXOFFSET;
            wBuf[1] = offset;
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, -1, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }

        // Restore I2SCFG
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            wBuf[0] = AD24xx_REG_I2SCFG;
            wBuf[1] = list->I2SCFG;
            result = adi_a2b_cmdlist_node_twi_transfer(
                list, -1, false, false, 0,
                wBuf, 2, NULL, 0, 0
            );
        }
    }

    adi_a2b_cmdlist_log(
        list, true, "End remap slots\n"
    );

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_partial_execute(
    ADI_A2B_CMDLIST *list, int8_t node,
    ADI_A2B_CMDLIST_EXECUTE_INFO *results)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;
    uint8_t wBuf[2];
    uint8_t rBuf[1];

    // Make sure the list has been scanned
    if (!list->scanned) {
        result = adi_a2b_cmdlist_scan(list, NULL);
    }

    // Confirm this node has been discovered
    if (node > (list->nodesDiscovered - 1)) {
        return(ADI_A2B_CMDLIST_A2B_BAD_NODE);
    }

    // If it's the last node, nothing to do
    if (node == (list->nodesScanned - 1)) {
        list->cmdLine = 0;
        adi_a2b_cmdlist_set_results(list, results, result);
        return(ADI_A2B_CMDLIST_SUCCESS);
    }

    adi_a2b_cmdlist_log(
        list, true, "Begin partial discovery at node: %d\n", node
    );

    // Try a normal discovery first if starting from the main node
    if (node == -1) {
        result = adi_a2b_cmdlist_execute(list, results);
        if ( (result == ADI_A2B_CMDLIST_SUCCESS) ||
             (list->nodesDiscovered == 0) ) {
            return(result);
        }
        adi_a2b_cmdlist_control(list, AD24xx_BITM_CONTROL_ENDDSC);
        adi_a2b_cmdlist_update_swctl(list,
            AD24xx_BITM_SWCTL_ENSW,
            true, 0x00
        );
    } else {
        // Confirm last node exists and is indeed last
        wBuf[0] = AD24xx_REG_NODE;
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, node, false, false, 0,
            wBuf, 1, rBuf, 1, 1
        );
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            if ((rBuf[0] & AD24xx_BITM_NODE_LAST) == 0) {
                return(ADI_A2B_CMDLIST_A2B_NOT_LAST_NODE);
            }
        } else {
            return(ADI_A2B_CMDLIST_A2B_BAD_NODE);
        }

        // Clear previous discovery history
        adi_a2b_cmdlist_discover_restart(list);

        // Clear command line processing history
        adi_a2b_cmdlist_reset(list);

        // Reset nodesDiscovered to last good node
        list->nodesDiscovered = node + 1;

        // Open switch of last good node
        wBuf[0] = AD24xx_REG_SWCTL;
        wBuf[1] = 0x00;
        result = adi_a2b_cmdlist_node_twi_transfer(
            list, node, false, false, 0,
            wBuf, 2, NULL, 0, 0
        );

        // Delay 100mS
        adi_a2b_cmdlist_delay(list, 100);
#ifdef ADI_A2B_CMDLIST_PARTIAL_RESET_RESP_CYCLES
        // TODO: investigate further, including this can cause audio glitching
        // Reset RESP_CYC of all good nodes except the last good node
        if (result == ADI_A2B_CMDLIST_SUCCESS) {
            result = adi_a2b_cmdlist_update_respcycs(list, node);
        }
#endif
        // Re-discover dropped nodes
        if (result == ADI_A2B_CMDLIST_SUCCESS) {

            // Re-discover dropped nodes
            result = adi_a2b_cmdlist_discover_nodes(list, node);

            // End discovery on error
            if (result != ADI_A2B_CMDLIST_SUCCESS) {
                adi_a2b_cmdlist_control(list, AD24xx_BITM_CONTROL_ENDDSC);
            }

            // Set all good nodes SWCTL register to 0x01 except last good node
            // Set last good nodes SWCTL register to 0x00
            adi_a2b_cmdlist_update_swctl(list,
                AD24xx_BITM_SWCTL_ENSW,
                true, 0x00
            );
        }

        /* Pass along the results */
        list->cmdLine = 0;
        adi_a2b_cmdlist_set_results(list, results, result);
    }

    // Try to reconfigure newly discovered nodes regardless of error
    if ((list->nodesDiscovered - 1) > node) {
        adi_a2b_cmdlist_reconfigure_nodes(list, node);
        adi_a2b_cmdlist_finalize(list);
    }

    adi_a2b_cmdlist_log(
        list, true, "End partial discovery at node: %d\n", list->nodesDiscovered
    );

    return(result);
}

ADI_A2B_CMDLIST_RESULT adi_a2b_cmdlist_ioctl(ADI_A2B_CMDLIST *list, int8_t node,
    ADI_A2B_CMDLIST_IOCTL ioctl, void *param)
{
    ADI_A2B_CMDLIST_RESULT result = ADI_A2B_CMDLIST_SUCCESS;

    // Make sure the list has been scanned
    if (!list->scanned) {
        result = adi_a2b_cmdlist_scan(list, NULL);
    }

    // Confirm nodes in config
    if (node >= list->nodesScanned) {
        return(ADI_A2B_CMDLIST_A2B_BAD_NODE);
    }

    switch (ioctl) {
        case ADI_A2B_CMDLIST_IOCTL_DISABLE_CLOCKS:
            if ((node < 0 || node >= ADI_A2B_CMDLIST_MAX_NODES)) {
                result = ADI_A2B_CMDLIST_ERROR;
                break;
            }
            list->privNodeInfo[node].disableClocks = (bool)param;
            break;
        case ADI_A2B_CMDLIST_IOCTL_ENABLE_SNIFFING:
            list->enableSniffing = (bool)param;
            break;
        case ADI_A2B_CMDLIST_IOCTL_ABORT_ON_ERROR:
            list->abortOnError = (bool)param;
            break;
        case ADI_A2B_CMDLIST_IOCTL_SET_RESPCYCS_FORMULA:
            if ((ADI_A2B_RESPCYCS_FORMULA)param == RESPCYCS_FORMULA_A) {
                list->respcycsFormula = RESPCYCS_FORMULA_A;
            } else {
                result = ADI_A2B_CMDLIST_RESPCYCS_ERROR;
            }
            break;
        default:
            result = ADI_A2B_CMDLIST_ERROR;
            break;
    }

    return(result);
}
