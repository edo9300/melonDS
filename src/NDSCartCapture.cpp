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
#include <termios.h>
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

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

namespace NDSCart
{

CartCapture::CartCapture(std::string filename, void *userdata)
    : CartCommon(std::make_unique<u8[]>(0x8000), 0, 0, false, {0,0,0}, CartType::Capture, userdata)
{
    struct termios tty;

    fd = open(filename.c_str(), O_RDWR | O_NOCTTY);

    tcgetattr(fd, &tty);

    tty.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    tty.c_oflag &= ~(OPOST);
    tty.c_cflag |= (CS8);
    tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

    tcsetattr(fd, TCSAFLUSH, &tty);
}

CartCapture::~CartCapture()
{
}

void CartCapture::Reset()
{
    CartCommon::Reset();
}

// 0x10 [3:bufsize] [4:flags] [command] [data...]
#define CMD_NTR_START  0x10
// 0x11 [3:bufsize] [data...]
#define CMD_NTR_FINISH 0x11
// 0x12 [value] [2:flags]
#define CMD_NTR_SPI    0x12
// 0x13 [1:empty] [1:seed0h] [1:seed1h] [4:seed0l] [4:seed0h]
#define CMD_NTR_SET_SEED 0x13

void CartCapture::ROMApplySeed(NDS& nds)
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
    fd_write(fd, txData, 16);
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
}

u8 CartCapture::SPIWrite(u8 val, u32 pos, bool last)
{
    u8 data[4] = {0};
    u16 spiCnt = NDS::Current->NDSCartSlot.GetSPICnt();

    data[0] = CMD_NTR_SPI;
    data[1] = val;
    data[2] = spiCnt;
    data[3] = spiCnt >> 8;
    fd_write(fd, data, 4);
    fd_read(fd, data, 4);

    return data[1];
}

}
}
