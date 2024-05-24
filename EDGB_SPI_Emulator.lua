-- EverDrive GB SPI emulator for Mesen2
-- Copyright (c) Adrian "asie" Siekierka, 2024
-- All rights reserved.

-- TODO:
-- - Emulate the bits in SPI_CTRL

-- Configuration
local IMAGE_PATH = "/path/to/filesystem.img"
local STORAGE_HIGH_CAPACITY = true

----
---- STORAGE EMULATION
----

local TF_SPI_WRITING_SINGLE = 1
local TF_SPI_WRITING_MULTIPLE = 2
local TF_ILLEGAL_COMMAND = 0x04
local TF_PARAMETER_ERROR = 0x40

local spi_tf = (function()
	local phy = io.open(IMAGE_PATH, "r+b")
	if phy == nil then
		error("Could not open '" .. IMAGE_PATH .. "'!")
	end

	local m = {}
	local tx_buffer = ""
	local rx_buffer = ""
	
	local status = 0
	
	local app_cmd = false
	local reading = false
	local writing = 0
	
	local function push(data)
		tx_buffer = tx_buffer .. data
	end
	
	local function pop(length)
		local result = rx_buffer:sub(1,length)
		rx_buffer = rx_buffer:sub(length+1)
		
		if #result < length then
			result = result .. string.rep(string.char(0xFF), length - #result)
		end
		
		return result
	end
	
	m.xmit = function(tx)
		if #rx_buffer > 0 or writing > 0 or tx < 0x80 then
			rx_buffer = rx_buffer .. string.char(tx)
		end
		
		local rx = 0xFF
		if #tx_buffer > 0 then
			rx = string.byte(tx_buffer)
			tx_buffer = tx_buffer:sub(2)
		end
		
		if writing > 0 then
			-- handle card output
			if rx ~= 0xFF then
				return rx
			end
			
			-- remove stall bytes
			while string.byte(rx_buffer) == 0xFF do
				pop(1)
			end
			
			-- data token present?
			if #rx_buffer == 0 then
				return 0xFF
			end
			
			if writing == TF_SPI_WRITING_SINGLE then
				if string.byte(rx_buffer) ~= 0xFE then
					emu.log(string.format("TF: unexpected data block start %02x", string.byte(rx)))
					writing = 0
					pop(1)
					return 0xFF
				end
				
				-- write data block
				if #rx_buffer < 515 then
					return 0xFF
				end
				
				phy:write(rx_buffer:sub(2, 2 + 511))
				writing = 0
				pop(515)
				
				push(string.char(0xE5))
			end
			
			if writing == TF_SPI_WRITING_MULTIPLE then
				if string.byte(rx_buffer) ~= 0xFC then
					if string.byte(rx_buffer) ~= 0xFD then
						emu.log(string.format("TF: unexpected data block start %02x", string.byte(rx)))
					end
					
					writing = 0
					pop(1)
					return 0xFF
				end
				
				-- write data block
				if #rx_buffer < 515 then
					return 0xFF
				end
				
				phy:write(rx_buffer:sub(2, 2 + 511))
				pop(515)
				
				push(string.char(0xE5))
			end
		end -- if writing
			
		if reading and #rx_buffer == 0 then
			push(string.char(0xFE))
			push(phy:read(512))
			-- TODO: CRC calculation
			push(string.char(0x00, 0x00))
		end
		
		while #rx_buffer >= 6 do
			while string.byte(rx_buffer) >= 0x80 do
				pop(1)
			end
			if #rx_buffer < 6 then break end
			local req = pop(6)
			local cmd = string.byte(req,1)
			local arg = (string.byte(req,2) << 24) | (string.byte(req,3) << 16) | (string.byte(req,4) << 8) | string.byte(req,5)
			local response = string.char(0x00)
			
			local curr_app_cmd = app_cmd
			app_cmd = false
			
			if (cmd & 0x3F) == 0 then
				emu.log("TF: reset")
				status = 0x01
			elseif (cmd & 0x3F) == 1 then
				emu.log("TF: init (CMD1)")
				status = 0x00
			elseif (cmd & 0x3F) == 8 then
				emu.log("TF: read interface configuration")
				response = string.char(0x00, 0x00, 0x00, 0x01, arg & 0xFF)
			elseif (cmd & 0x3F) == 12 then
				emu.log("TF: stop reading")
				reading = false
				tx_buffer = ""
				response = string.char(0xFF, 0xFF, 0x00, 0xFF)
			elseif (cmd & 0x3F) == 16 then
				emu.log(string.format("TF: set block length = %d", arg))
				if arg ~= 512 then
					response = string.char(TF_PARAMETER_ERROR)
				end
			elseif ((cmd & 0x3F) == 17) or ((cmd & 0x3F) == 18) then
				if STORAGE_HIGH_CAPACITY then arg = arg << 9 end
				phy:seek("set", arg)
				if (cmd & 0x3F) == 17 then emu.log(string.format("TF: reading single sector @ %08X", phy:seek())) end
				if (cmd & 0x3F) == 18 then emu.log(string.format("TF: reading multiple sectors @ %08X", phy:seek())) end
				response = string.char(0x00, 0xFF, 0xFE) .. phy:read(512) .. string.char(0x00, 0x00)
				-- TODO: CRC calculation
				if (cmd & 0x3F) == 18 then reading = true end
			elseif ((cmd & 0x3F) == 24) or ((cmd & 0x3F) == 25) then
				if STORAGE_HIGH_CAPACITY then arg = arg << 9 end
				phy:seek("set", arg)
				if (cmd & 0x3F) == 24 then
					emu.log(string.format("TF: writing single sector @ %08X", phy:seek()))
					writing = TF_SPI_WRITING_SINGLE
				end
				if (cmd & 0x3F) == 25 then
					emu.log(string.format("TF: writing multiple sectors @ %08X", phy:seek()))
					writing = TF_SPI_WRITING_MULTIPLE
				end
			elseif (cmd & 0x3F) == 55 then
				app_cmd = true
			elseif (cmd & 0x3F) == 58 then
				emu.log("TF: read OCR")
				local ocr = 0x00000000
				if STORAGE_HIGH_CAPACITY then
					ocr = ocr | (1 << 30)
				end
				response = string.char(0x00, (ocr >> 24) & 0xFF, (ocr >> 16) & 0xFF, (ocr >> 8) & 0xFF, ocr & 0xFF)
			elseif curr_app_cmd and (cmd & 0x3F) == 41 then
				emu.log("TF: init (ACMD41)")
				status = 0x00
			else
				emu.log(string.format("TF: unknown command %d", cmd & 0x3F))
				response = string.char(TF_ILLEGAL_COMMAND)
			end
			
			push(string.char(string.byte(response, 1) | status) .. response:sub(2))
		end
		
		return rx
	end

	return m
end)()
-- Constants

----
---- CARTRIDGE EMULATION
----

-- Constants
local EDGB_REG_SPI_DATA = 0xBD00
local EDGB_REG_SPI_CTRL = 0xBD01
local EDGB_REG_KEY = 0xBD0A
local EDGB_KEY_UNLOCK = 0xA5

-- State
local spi_data = 0xFF
local spi_ctrl = 0
local unlocked = false

-- Handlers
local unlockedReadCb = nil
local unlockedWriteCb = nil

local function onRead(address)
	if unlocked then
		if address == EDGB_REG_SPI_DATA then
			return spi_data
		elseif address == EDGB_REG_SPI_CTRL then
			return spi_ctrl
		elseif address >= 0xA000 then
			emu.log(string.format("unknown read: %04X", address))
		end
	end
end

local function onWrite(address, value)
	if unlocked then
		if address == EDGB_REG_SPI_DATA then
			spi_data = spi_tf.xmit(value)
		elseif address == EDGB_REG_SPI_CTRL then
			spi_ctrl = value
		elseif address >= 0xA000 then
			emu.log(string.format("unknown write: %04X = %02X", address, value))
		end
	end
end

local function onUnlock(address, value)
	if value == EDGB_KEY_UNLOCK then
		unlocked = true
		emu.log("EDGB: Unlocking")
	else
		unlocked = false
		-- emu.removeMemoryCallback(onRead, emu.callbackType.read, 0xA000, 0xBFFF)
		-- emu.removeMemoryCallback(onWrite, emu.callbackType.write, 0xA000, 0xBFFF)
		emu.log("EDGB: Locking")
	end
end

-- TODO: Fix removeMemoryCallback
emu.addMemoryCallback(onRead, emu.callbackType.read, 0xA000, 0xBFFF)
emu.addMemoryCallback(onWrite, emu.callbackType.write, 0xA000, 0xBFFF)		
emu.addMemoryCallback(onUnlock, emu.callbackType.write, 0xBD0A)
