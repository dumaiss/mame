-- license:BSD-3-Clause
-- copyright-holders:Kitamura
-- KonaOS developer tooling for the zephyr80 machine.
--
--   konaos.load_os(map)             the nucleus symbols (konaos.map)
--   konaos.load_program(map)        a program: its map, placement.json, EXE
--                                   manifest and EXE beside it
--   konaos.find(spec)               every symbol a name matches
--   konaos.where(spec)              one symbol: image, zone, logical and
--                                   physical addresses
--   konaos.bp(spec[, action])       a breakpoint that stops only in the
--                                   symbol's own physical bank (-debug)
--   konaos.here()                   the symbol at the stopped PC
--   konaos.sram(phys[, n])          SRAM bytes by physical address, any bank
--   konaos.vram(addr[, n])          VRAM bytes (the V9958's 128 KiB)
--   konaos.console                  the GRAPHIC6 console: read, find, capture
--   konaos.keys                     typing on the USB keyboard
--
-- The functions raise an error with a plain message when an input is
-- missing or a name is ambiguous; konaos.diagnostics lists the warnings.
-- `-plugin konaos` loads it; MAME's Lua console (-console) reaches it as
-- `konaos`, scripts as emu.plugin.konaos.  KONAOS_SESSION names a session
-- file (the SDK's `make run`): stage, prompt, command, capture.
local exports = {
	name = "konaos",
	version = "1.0",
	description = "KonaOS on zephyr80: symbols, bank-aware breakpoints, physical memory, console capture, SDK runs",
	license = "BSD-3-Clause",
	author = { name = "Kitamura" } }

local plugin = exports

function plugin.set_folder(path)
	plugin.folder = path
end

function plugin.startplugin()
	local api = require("konaos/api")
	api.console = require("konaos/console")
	api.keys = api.console.keys
	emu.plugin.konaos = api
	_G.konaos = api
	local session = os.getenv("KONAOS_SESSION")
	if session and session ~= "" then
		require("konaos/session").start(api, session)
	end
end

return exports
