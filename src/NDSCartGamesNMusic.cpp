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
u8 CartGamesNMusic::FlashChip::HandleSpi(u8 val, u32 pos){
	if(pos == 0) {
		currentWorkFunction = nullptr;
		switch(val) {
		// set addr line
		case 0xE3: {
			currentWorkFunction = [this](u8 val, u32 pos) -> u8 {
				switch(pos){
				case 1:
					currAddrLine = (u32)val << 0x10;
					break;
				case 2:
					currAddrLine |= (u32)val << 8;
					break;
				case 3:
					currAddrLine |= (u32)val;
					[[fallthrough]];
				default:
					currentWorkFunction = nullptr;
				}
				return 0xFF;
			};
			break;
		}
		// read bytes
		case 0xE7: {
			currentWorkFunction = [this](u8 val, u32 pos) -> u8 {
				return m_card->GetROM()[(currAddrLine + (pos - 1)) % m_card->GetROMLength()];
			};
			break;
		}
		default:
			Log(LogLevel::Error, "Unhandled flash command: 0x%02X\n", (int)val);
			break;
		}
	} else if(currentWorkFunction) {
		return currentWorkFunction(val, pos);
	}
	return 0;
}

CartGamesNMusic::CartGamesNMusic(std::unique_ptr<u8[]>&& rom, u32 len, u32 chipid, ROMListEntry romparams, void* userdata,
			std::optional<FATStorage>&& sdcard)
	: CartSD(std::move(rom), len, chipid, romparams, userdata, std::move(sdcard)), flashChip(this), sdHost(this)
{
	SDMode = false;
}

CartGamesNMusic::~CartGamesNMusic()
{
}

static int cmdTot = 0;

void CartGamesNMusic::Reset()
{
	CartSD::Reset();
	sdHost.Reset();
	cmdTot = 0;
	SDMode = false;
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
		auto wasSd = SDMode;
		SDMode = param2 == 0xCC;
		if(SDMode != wasSd){
			sdHost.Reset();
		}
		return 0;
	}
	default:
		return CartSD::ROMCommandStart(nds, cartslot, cmd, data, len);
	}
}

u8 CartGamesNMusic::SPIWrite(u8 val, u32 pos, bool last) {
	if(SDMode)
		return sdHost.HandleSpi(val, pos);
	else
		return flashChip.HandleSpi(val, pos);
}

CartGamesNMusic::SDHost::SDHost(CartGamesNMusic* cart) : m_card(cart), resetted(true)
{
	nextIsAppCommand = false;
	if(m_card->SD) {
		sdhc = m_card->SD->GetSectorCount() > 8388608;
	}
}

void CartGamesNMusic::SDHost::Reset()
{
	resetted = true;
	/*currentWorkFunction = nullptr;
	nextIsAppCommand = false;*/
}

std::function<u8(u8, u32)> CartGamesNMusic::SDHost::ParseSdCommand(const std::vector<u8>& commandBuffer)
{
	const auto* SDCommandBuffer = &commandBuffer.front();
	if(nextIsAppCommand) {
		nextIsAppCommand = false;
		return ParseSdAppCommand(commandBuffer);
	}
	auto command = *SDCommandBuffer & ~0x40;
	if(command != 17)
		Log(LogLevel::Debug, "SD command: %d\n", command);
	switch(command) {
	// CMD0
	case 0: {
		return makeFunctionReturningBytes({0x01});
	}
	// CMD8
	case 8: {
		if(sdhc) {
			return makeFunctionReturningBytes({0x01, 0x00, 0x00, 0x01, 0xAA});
		} else {
			return nullptr;
		}
	}
	// CMD12, stop transmission
	case 12: {
		// A bit "off spec" (maybe), we are aborting the transmission the moment we get a non 0xFF
		// byte, so this is effectively a noop
		// technically only returns 1 byte, but we add extra ones to simulate delays
		return makeFunctionReturningBytes({0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
	}
	// CMD16, set blocklen, do nothing
	case 16: {
		++cmdTot;
		return makeFunctionReturningBytes({0x00});
	}
	// CMD17, read single block
	case 17: {
		auto sector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc)
		{
			sector >>= 9;
		}

		std::vector<u8> responseBuffer;
		responseBuffer.resize(512 + 2 + 2);
		responseBuffer[0] = 0x00;
		responseBuffer[1] = 0xfe;
		m_card->SD->ReadSectors(sector, 1, &responseBuffer[2]);
		return makeFunctionReturningBytes(std::move(responseBuffer));
	}
	// CMD18, read multiple block
	case 18: {
		auto startSector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc){
			startSector >>= 9;
		}

		// technically only returns 1 byte, but we add an extra one to simulate delays
		std::vector<u8> responseBuffer{0x00, 0xFF};
		responseBuffer.reserve(1 + 512 + 2);
		return [this, sector = startSector, buffer = responseBuffer](u8 val, u32 pos) mutable -> u8 {
			if(buffer.empty())
			{
				if(val != 0xFF)
				{
					currentWorkFunction = makeParseSdCommandFunction({val});
					return 0xFE;
				}
				ReadSector(sector, buffer);
				++sector;
			}
			auto begin = buffer.begin();
			auto res = *begin;
			buffer.erase(begin);
			return res;
		};
	}
	// CMD24, write single block
	case 24:
	// CMD25, write multiple block
	case 25: {
		auto sector = (unsigned)SDCommandBuffer[4] | (unsigned)SDCommandBuffer[3] << 8
					| (unsigned)SDCommandBuffer[2] << 16 | (unsigned)SDCommandBuffer[1] << 24;
		if(!sdhc)
		{
			sector >>= 9;
		}
		return makeWriteSectorFunction(sector, command == 25);
	}
	// CMD55, application command
	case 55: {
		nextIsAppCommand = true;
		return makeFunctionReturningBytes({0x00});
	}
	// CMD58
	case 58: {
		return makeFunctionReturningBytes({0x00, static_cast<u8>(sdhc ? 0x40 : 0x00), 0x00, 0x00, 0x00});
	}
	default: {
		Log(LogLevel::Warn, "Unknown SD command: %d\n", command);
		return nullptr;
	}
	}
}

std::function<u8(u8, u32)> CartGamesNMusic::SDHost::ParseSdAppCommand(const std::vector<u8>& commandBuffer)
{
	const auto* SDCommandBuffer = &commandBuffer.front();
	auto acmd = *SDCommandBuffer & ~0x40;
	Log(LogLevel::Debug, "APP command: %d\n", acmd);
	switch(acmd) {
	// ACMD41
	case 41: {
		return makeFunctionReturningBytes({0x00});
	}
	default: {
		Log(LogLevel::Warn, "Unknown APP command: %d\n", acmd);
		return nullptr;
	}
	}
}

std::function<u8(u8, u32)> CartGamesNMusic::SDHost::makeFunctionReturningBytes(std::vector<u8> commandBuffer)
{
	return [this, buffer=std::move(commandBuffer)](u8, u32) mutable -> u8 {
		auto begin = buffer.begin();
		auto res = *begin;
		buffer.erase(begin);
		if(buffer.empty())
		{
			currentWorkFunction = nullptr;
		}
		return res;
	};
}

std::function<u8(u8, u32)> CartGamesNMusic::SDHost::makeParseSdCommandFunction(std::vector<u8> commandBuffer)
{
	commandBuffer.reserve(6);
	return [this, buffer = std::move(commandBuffer)](u8 val, u32 pos) mutable -> u8 {
		if(!buffer.empty() || val != 0xFF)
		{
			buffer.push_back(val);
		}
		if(buffer.size() == 6)
		{
			currentWorkFunction = ParseSdCommand(buffer);
		}
		return 0xFE;
	};
}

std::function<u8(u8, u32)> CartGamesNMusic::SDHost::makeWriteSectorFunction(u32 sector, bool isMulti)
{
	static constexpr auto SINGLE_WRITE_START_TOKEN = 0xFE;
	static constexpr auto MULTI_WRITE_START_TOKEN = 0xFC;
	static constexpr auto MULTI_WRITE_STOP_TOKEN = 0xFD;

	return [=](u8 val, u32 pos) mutable -> u8 {
		std::vector<u8> writeBuffer;
		u16 sectorWriteIdx = 0;
		writeBuffer.reserve(512);
		currentWorkFunction =
				[this, buffer = std::move(writeBuffer), sectorWriteIdx, isMulti, currentWriteSector = sector](u8 val, u32 pos) mutable -> u8 {
			// wait start token
			if(sectorWriteIdx == 0)
			{
				if(isMulti)
				{
					if(val == MULTI_WRITE_START_TOKEN)
					{
						++sectorWriteIdx;
					}
					else if(val == MULTI_WRITE_STOP_TOKEN)
					{
						currentWorkFunction = nullptr;
						return 0xFF;
					}
				}
				else
				{
					// in single block start token is 0xFE
					if(val == SINGLE_WRITE_START_TOKEN)
					{
						++sectorWriteIdx;
					}
				}
				buffer.clear();
				return 0xFF;
			}
			if (sectorWriteIdx < 512 + 1)
			{
				// read 512 bytes
				buffer.push_back(val);
				++sectorWriteIdx;
				return 0xFF;
			}
			if(sectorWriteIdx < 512 + 1 + 2)
			{
				// drop 2 bytes crc
				++sectorWriteIdx;
				return 0xFF;
			}
			if(sectorWriteIdx < 512 + 1 + 2 + 1)
			{
				++sectorWriteIdx;
				if(m_card->SD->WriteSectors(currentWriteSector, 1, buffer.data()) == 0)
					return 0x00;
				// SD ok
				return 0x05;
			}
			if(isMulti)
			{
				sectorWriteIdx = 0;
				++currentWriteSector;
			}
			else
			{
				currentWorkFunction = nullptr;
			}
			// respond with 0x01 to signal write successful
			return 0x01;
		};
		return 0x00;
	};
}

void CartGamesNMusic::SDHost::ReadSector(u32 sector, std::vector<u8>& responseBuffer) {
	responseBuffer.resize(1 + 512 + 2);
	responseBuffer[0] = 0xfe;
	m_card->SD->ReadSectors(sector, 1, &responseBuffer[1]);
}

u8 CartGamesNMusic::SDHost::HandleSpi(u8 val, u32 pos)
{
	if(!m_card->SD)
		return 0xFF;

	/*if(cmdTot == 2)
		Log(LogLevel::Error, "Handling spi: 0x%02X, %d\n", (int)val, (int)pos);*/
	if(pos == 0 || resetted || !currentWorkFunction)
	{
		resetted = false;
		std::vector<u8> buff;
		if(val != 0xFF)
		{
			buff.push_back(val);
		}
		currentWorkFunction = makeParseSdCommandFunction(std::move(buff));
		return 0xFE;
	}
	else if(currentWorkFunction)
	{
		return currentWorkFunction(val, pos);
	}
	else
	{
		return 0xFF;
	}
}

}
}
