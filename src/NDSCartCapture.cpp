/*
    Copyright 2016-2024 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#include <cstdio>
#include <cstring>
#ifdef _WIN32
#define _WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <fcntl.h>
#else
#include <termios.h>
#endif
#include <cinttypes>
#include <unistd.h>
#include "NDS.h"
#include "DSi.h"
#include "NDSCart.h"
#include "Platform.h"

static int fd_read(int fd, uint8_t *buffer, size_t len) {
    size_t bytes_left = len;
    while (bytes_left) {
        int r = read(fd, buffer, bytes_left);
        if (r > 0) {
            buffer += r;
            bytes_left -= r;
        } else if (r < 0) {
            return r;
        }
    }
    return len;
}

static int fd_write(int fd, uint8_t *buffer, size_t len) {
    size_t bytes_left = len;
    while (bytes_left) {
        int r = write(fd, buffer, bytes_left);
        if (r > 0) {
            buffer += r;
            bytes_left -= r;
        } else if (r < 0) {
            return r;
        }
    }
    return len;
}

static uint64_t get_timestamp(void) {
    return (uint64_t) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

#ifdef _WIN32
static int open_serial(const char* name) {
    DCB dcb;
    COMMTIMEOUTS timeouts;

    auto handle = CreateFileA(name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    if(handle == INVALID_HANDLE_VALUE)
        return -1;

    if(!GetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return -1;
    }

    if(!GetCommTimeouts(handle, &timeouts)) {
        CloseHandle(handle);
        return -1;
    }

    dcb.fParity = 1;
    dcb.BaudRate = 0;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fTXContinueOnXoff = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.XonChar = 0;
    dcb.XoffChar = 0;
    dcb.XonLim = 0;
    dcb.XoffLim = 0;
    dcb.EofChar = 0;
    dcb.fBinary = 1;
    dcb.EvtChar = '\n';

    if(!SetCommState(handle, &dcb)) {
        CloseHandle(handle);
        return -1;
    }

    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;

    if(!SetCommTimeouts(handle, &timeouts)) {
        CloseHandle(handle);
        return -1;
    }

    auto fd = _open_osfhandle((intptr_t)handle, O_RDWR);

    if(fd == -1) {
        CloseHandle(handle);
    }

    return fd;
}

#else

static int open_serial(const char* name) {
    struct termios tty;
    int fd = open(name, O_RDWR | O_NOCTTY);

    tcgetattr(fd, &tty);

    tty.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    tty.c_oflag &= ~(OPOST);
    tty.c_cflag |= (CS8);
    tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

    tcsetattr(fd, TCSAFLUSH, &tty);

    return fd;
}

#endif

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

namespace NDSCart
{

CartCapture::CartCapture(std::string filename, void *userdata)
    : CartCommon(std::make_unique<u8[]>(0x8000), 0, 0, false, {0,0,0}, CartType::Capture, userdata)
{
    char logFileName[128];

    snprintf(logFileName, sizeof(logFileName), "ntrcard_%" PRIu64 ".csv", get_timestamp());
    logFile = fopen(logFileName, "w");
    if (logFile != nullptr)
        fprintf(logFile, "Type,Direction,Timestamp,Flags,C[0],C[1],C[2],C[3],C[4],C[5],C[6],C[7],Data\n");

    fd = open_serial(filename.c_str());
}

CartCapture::~CartCapture()
{
#ifdef _WIN32
    auto handle = _get_osfhandle(fd);
#endif
    close(fd);
#ifdef _WIN32
    CloseHandle((HANDLE)handle);
#endif
}

void CartCapture::Reset()
{
    CartCommon::Reset();
}

// 0x10 [3:bufsize] [4:flags] [command] [data...]
#define CMD_NTR_START  0x10
// 0x11 [3:bufsize] [data...]
#define CMD_NTR_FINISH 0x11
// 0x12 [value]
#define CMD_NTR_SPI    0x12
// 0x13 [1:empty] [1:seed0h] [1:seed1h] [4:seed0l] [4:seed0h]
#define CMD_NTR_SET_SEED 0x13
// 0x14 [value] [2:flags]
#define CMD_NTR_SPICNT 0x14

void CartCapture::OnROMApplySeed(NDS& nds)
{
    u8 txData[16];
    u32 snum = (nds.ExMemCnt[0]>>8)&0x8;
    u32 romCnt = nds.NDSCartSlot.GetROMCnt() | (1 << 15);
    txData[0] = CMD_NTR_SET_SEED;
    txData[1] = 0;
    txData[2] = nds.ROMSeed0[snum+4];
    txData[3] = nds.ROMSeed1[snum+4];
    txData[4] = nds.ROMSeed0[snum];
    txData[5] = nds.ROMSeed0[snum+1];
    txData[6] = nds.ROMSeed0[snum+2];
    txData[7] = nds.ROMSeed0[snum+3];
    txData[8] = nds.ROMSeed1[snum];
    txData[9] = nds.ROMSeed1[snum+1];
    txData[10] = nds.ROMSeed1[snum+2];
    txData[11] = nds.ROMSeed1[snum+3];
    txData[12] = romCnt;
    txData[13] = romCnt >> 8;
    txData[14] = romCnt >> 16;
    txData[15] = romCnt >> 24;
    fd_write(fd, txData, sizeof(txData));
}

void CartCapture::OnROMWriteSPICnt(NDS& nds)
{
    u8 txData[4];
    u16 spiCnt = nds.NDSCartSlot.GetSPICnt() & (~0x80);
    txData[0] = CMD_NTR_SPICNT;
    txData[1] = 0;
    txData[2] = spiCnt;
    txData[3] = spiCnt >> 8;
    fd_write(fd, txData, sizeof(txData));
}

int CartCapture::ROMCommandStart(NDS& nds, NDSCart::NDSCartSlot& cartslot, const u8* cmd, u8* data, u32 len)
{
    u8 txData[16];
    u8 tmp = 0;
    u8 rxData[4] = {0};
    u32 rxLen;

    u32 romCnt = nds.NDSCartSlot.GetROMCnt();
    bool writeFlag = (romCnt & (1 << 30)) != 0;

    if (writeFlag) {
        // Writes happen in ROMCommandFinish
        return 1;
    }

    txData[0] = CMD_NTR_START;
    txData[1] = 0;
    txData[2] = 0;
    txData[3] = 0;
    txData[4] = romCnt;
    txData[5] = romCnt >> 8;
    txData[6] = romCnt >> 16;
    txData[7] = romCnt >> 24;
    for (int i = 0; i < 8; i++)
        txData[8 + i] = cmd[i];

    fd_write(fd, txData, 16);

    fd_read(fd, rxData, 4);
    rxLen = rxData[1] | (rxData[2] << 8) | (rxData[3] << 16);

    fd_read(fd, data, len < rxLen ? len : rxLen);
    for (u32 i = len; i < rxLen; i++)
        fd_read(fd, &tmp, 1);

    if (logFile != nullptr) {
        fprintf(logFile, "ROM,Read,%" PRIu64 ",%08X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,",
                get_timestamp(), romCnt, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
        for (u32 i = 0; i < len; i++) {
            fprintf(logFile, "%02X ", data[i]);
        }
        fprintf(logFile, "\n");
        fflush(logFile);
    }

    u32 copy_offset = rxLen;
    while (copy_offset < len) {
        u32 to_copy = len - copy_offset;
        if (to_copy > rxLen)
            to_copy = rxLen;
        memcpy(data + copy_offset, data, to_copy);
        copy_offset += to_copy;
    }

    return 0;
}

void CartCapture::ROMCommandFinish(const u8* cmd, u8* data, u32 len)
{
    u8 txData[16];
    u8 tmp = 0;
    u8 rxData[4] = {0};
    u32 rxLen;

    u32 romCnt = NDS::Current->NDSCartSlot.GetROMCnt();
    bool writeFlag = (romCnt & (1 << 30)) != 0;

    if (!writeFlag) {
        // Reads happen in ROMCommandStart
        return;
    }

    txData[0] = CMD_NTR_START;
    txData[1] = len;
    txData[2] = len >> 8;
    txData[3] = len >> 16;
    txData[4] = romCnt;
    txData[5] = romCnt >> 8;
    txData[6] = romCnt >> 16;
    txData[7] = romCnt >> 24;
    for (int i = 0; i < 8; i++)
        txData[8 + i] = cmd[i];

    if (fd_write(fd, txData, 16))
        if (len)
            fd_write(fd, data, len);

    fd_read(fd, rxData, 4);
    rxLen = rxData[1] | (rxData[2] << 8) | (rxData[3] << 16);

    fd_read(fd, data, len < rxLen ? len : rxLen);
    for (u32 i = len; i < rxLen; i++)
        fd_read(fd, &tmp, 1);

    if (logFile != nullptr) {
        fprintf(logFile, "ROM,Write,%" PRIu64 ",%08X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,",
                get_timestamp(), romCnt, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7]);
        for (u32 i = 0; i < len; i++) {
            fprintf(logFile, "%02X ", data[i]);
        }
        fprintf(logFile, "\n");
        fflush(logFile);
    }
}

u8 CartCapture::SPIWrite(u8 val, u32 pos, bool last)
{
    u8 data[2] = {0};
    u16 spiCnt = NDS::Current->NDSCartSlot.GetSPICnt() & ~0x80;

    data[0] = CMD_NTR_SPI;
    data[1] = val;
    fd_write(fd, data, 2);
    fd_read(fd, data, 2);

    if (logFile != nullptr) {
        fprintf(logFile, "SPI,Exchange,%" PRIu64 ",%04X,%02X,%02X,,,,,,,\n",
                get_timestamp(), spiCnt, val, data[1]);
        fflush(logFile);
    }

    return data[1];
}

}
}
