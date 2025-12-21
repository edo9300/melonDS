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

#include "NDS.h"
#include "NDSCart.h"
#include "Platform.h"

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

namespace NDSCart
{

CartGamesNMusic::CartGamesNMusic(std::unique_ptr<u8[]>&& rom, u32 len, u32 chipid, ROMListEntry romparams, void* userdata,
			std::optional<FATStorage>&& sdcard)
	: CartSD(std::move(rom), len, chipid, romparams, userdata, std::move(sdcard))
{
	SDCommandBufferIndex = 0;
	sdInitialized = false;
	nextIsAppCommand = false;
	sectorMultiBlockWrite = true;
	SDCommandResponseBuffer.reserve(1024);
	if(SD) {
		sdhc = SD->GetSectorCount() > 8388608;
	}
}

CartGamesNMusic::~CartGamesNMusic()
{
}

void CartGamesNMusic::Reset()
{
	CartSD::Reset();
	SDCommandBufferIndex = 0;
	sdInitialized = false;
	nextIsAppCommand = false;
	sectorMultiBlockWrite = true;
	pendingSectorWrite = std::nullopt;
	multiBlockReadSector = std::nullopt;
	SDCommandResponseBuffer.clear();
}

void CartGamesNMusic::DoSavestate(Savestate* file)
{
	CartSD::DoSavestate(file);
}

int CartGamesNMusic::ROMCommandStart(NDS& nds, NDSCart::NDSCartSlot& cartslot, const u8* cmd, u8* data, u32 len)
{
	switch(*cmd) {
	case 0x00: /* ROM read data */
	case 0xB7: /* ROM read data */
		{
			u32 addr = (cmd[1]<<24) | (cmd[2]<<16) | (cmd[3]<<8) | cmd[4];
			memcpy(data, &ROM[addr & (ROMLength-1)], len);
			return 0;
		}
	// spi
	case 0xF2: {
		auto param2 = cmd[5];
		if(param2 == 0){
			sdInitialized = SD.has_value();
		} else {
			SDCommandBufferIndex = 0;
			SDCommandResponseBuffer.clear();
			pendingSectorWrite = std::nullopt;
			multiBlockReadSector = std::nullopt;
		}
		return 0;
	}
	default:
		return CartSD::ROMCommandStart(nds, cartslot, cmd, data, len);
	}
}

void CartGamesNMusic::ParseSdCommand()
{
	SDCommandResponseBuffer.clear();
	if(nextIsAppCommand) {
		nextIsAppCommand = false;
		return ParseSdAppCommand();
	}
	auto command = *SDCommandBuffer & ~0x40;
	switch(command){
	// CMD0
	case 0: {
		SDCommandResponseBuffer = {0x01};
		break;
	}
	// CMD8
	case 8: {
		if(sdhc) {
			SDCommandResponseBuffer = {0x01, 0x00, 0x00, 0x01, 0xAA};
		}
		break;
	}
	// CMD12, stop transmission
	case 12: {
		// technically only returns 1 byte, but we add extra ones to simulate delays
		SDCommandResponseBuffer = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		multiBlockReadSector = std::nullopt;
		break;
	}
	// CMD16, set blocklen, do nothing
	case 16: {
		SDCommandResponseBuffer.resize(1);
		break;
	}
	// CMD17, read single block
	case 17: {
		auto sector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc){
			sector >>= 9;
		}

		SDCommandResponseBuffer.resize(512 + 2 + 2);
		SDCommandResponseBuffer[0] = 0x00;
		SDCommandResponseBuffer[1] = 0xfe;
		SD->ReadSectors(sector, 1, &SDCommandResponseBuffer[2]);
		break;
	}
	// CMD18, read multiple block
	case 18: {
		auto startSector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc){
			startSector >>= 9;
		}

		// technically only returns 1 byte, but we add an extra one to simulate delays
		SDCommandResponseBuffer = {0x00, 0xFF};
		multiBlockReadSector = startSector;
		break;
	}
	// CMD24, write single block
	case 24: {
		auto sector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc){
			sector >>= 9;
		}

		SDCommandResponseBuffer = {0x00};
		pendingSectorWrite = sector;
		sectorMultiBlockWrite = false;
		sectorWriteIdx = 0;
		break;
	}
	// CMD25, write multiple block
	case 25: {
		auto startSector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc){
			startSector >>= 9;
		}

		SDCommandResponseBuffer = {0x00};
		pendingSectorWrite = startSector;
		sectorMultiBlockWrite = true;
		sectorWriteIdx = 0;
		break;
	}
	// CMD55, application command
	case 55: {
		SDCommandResponseBuffer = {0x00};
		nextIsAppCommand = true;
		break;
	}
	// CMD58
	case 58: {
		SDCommandResponseBuffer.resize(5);
		SDCommandResponseBuffer[1] = sdhc ? 0x40 : 0x00;
		break;
	}
	default: {
		Log(LogLevel::Warn, "Unknown SD command: %d\n", command);
		return;
	}
	}
	Log(LogLevel::Debug, "SD command: %d\n", command);
}

void CartGamesNMusic::ParseSdAppCommand(){
	auto acmd = *SDCommandBuffer & ~0x40;
	switch(acmd) {
	// ACMD41
	case 41: {
		SDCommandResponseBuffer = {0x00};
		break;
	}
	default: {
		Log(LogLevel::Warn, "Unknown APP command: %d\n", acmd);
		return;
	}
	}
	Log(LogLevel::Debug, "APP command: %d\n", acmd);
}

u8 CartGamesNMusic::ParseWriteSectorSpi(u8 val) {
	// wait start token 0xFE
	if(sectorWriteIdx == 0) {
		if(sectorMultiBlockWrite){
			if(val == 0xFC) {
				++sectorWriteIdx;
			} else if(val == 0xFD) {
				// stop token received
				pendingSectorWrite = std::nullopt;
				sectorMultiBlockWrite = false;
				return 0xFF;
			}
		} else {
			// in single block start token is 0xFE
			if(val == 0xFE) {
				++sectorWriteIdx;
			}
		}
		return 0xFF;
	} else if (sectorWriteIdx < 512 + 1) {
		// read 512 bytes
		sectorWriteBuffer[sectorWriteIdx - 1] = val;
		++sectorWriteIdx;
	} else if(sectorWriteIdx < 512 + 1 + 2) {
		// drop 2 bytes crc
		++sectorWriteIdx;
	} else if(sectorWriteIdx < 512 + 1 + 2 + 1) {
		++sectorWriteIdx;
		if(SD->WriteSectors(*pendingSectorWrite, 1, sectorWriteBuffer) == 0)
			return 0;
		// SD ok
		return 0x05;
	} else {
		if(sectorMultiBlockWrite){
			sectorWriteIdx = 0;
			pendingSectorWrite = pendingSectorWrite.value() + 1;
		} else {
			pendingSectorWrite = std::nullopt;
		}
		// respond with 0x00 to signal write successful
		return 0x01;
	}
	return 0xFF;
}

void CartGamesNMusic::ReadSector(u32 sector) {
	SDCommandResponseBuffer.resize(1 + 512 + 2);
	SDCommandResponseBuffer[0] = 0xfe;
	SD->ReadSectors(sector, 1, &SDCommandResponseBuffer[1]);
}

u8 CartGamesNMusic::SPIWrite(u8 val, u32 pos, bool last) {
	if(!sdInitialized)
		return 0xFF;

	if(!SDCommandResponseBuffer.empty()) {
		auto ret = SDCommandResponseBuffer.front();
		SDCommandResponseBuffer.erase(SDCommandResponseBuffer.begin());
		return ret;
	}

	if(multiBlockReadSector.has_value()) {
		if(val == 0xFF) {
			auto sec = multiBlockReadSector.value();
			ReadSector(sec);
			multiBlockReadSector = sec + 1;
			return 0xFF;
		}
		multiBlockReadSector = std::nullopt;
	} else if(pendingSectorWrite) {
		return ParseWriteSectorSpi(val);
	}

	if(SDCommandBufferIndex != 0 || val != 0xFF) {
		SDCommandBuffer[SDCommandBufferIndex] = val;
		++SDCommandBufferIndex;
		if(SDCommandBufferIndex == 6){
			ParseSdCommand();
			SDCommandBufferIndex = 0;
		}
		return 0xFE;
	}
	return 0xFF;
}

}
}
