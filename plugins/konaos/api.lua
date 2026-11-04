-- license:BSD-3-Clause
-- copyright-holders:Kitamura
-- KonaOS symbols, physical memory and bank-aware breakpoints.
--
-- Images.  The OS image is konaos.map: the nucleus at 2000h-DFFFh lives in
-- bank 7 (the fixed core and the mode-11 window at P = 0) and common memory
-- E000h-FFFFh in bank 0 (KonaOS architecture 2.3, 2.4); 0000h-1FFFh is the
-- caller's bank in mode 11, so no physical bank owns an OS symbol there.  A
-- program is its link map with, beside it, place/placement.json (the home
-- zone's areas), the EXE manifest <name>.json (the segment rows and their
-- areas) and ../<name>.exe.  Its symbols' zones follow their areas: the home
-- areas are `home`, a segment's areas `far<n>` (FAR_CODE row n) or
-- `data<n>` (a mappable DATA row), every other area `resident`.
--
-- Banks.  The loader chooses a program's banks at run time, so they are
-- found in memory: at every launch (the nucleus's kos_process_launch, whose
-- request names the child's bank) the launched bank is compared with each
-- program's resident code from its EXE, and the aperture slice (8000h) of
-- banks 0-5 with its far and mappable segments' bytes.  `konaos.locate`
-- does the same at once.  A copy left in a freed bank can match too; it is
-- harmless, because no code runs there until a launch replaces it.
--
-- Breakpoints.  A symbol's breakpoint is set at its logical address with
-- the condition `phys(pc) == <physical>` (one term per bank its image was
-- found in).  phys() is the machine's debugger function: it applies the
-- board's decoder to the current latch, so the stop happens only when that
-- physical byte is the one executing - never for another bank's code at the
-- same logical address, such as a far segment and the home zone at 8000h.
-- Until its image is found a breakpoint is pending; it is set at the
-- launch that loads it.
--
-- Names.  `image:name` searches that image only; a bare name searches every
-- image.  An exact name is preferred; without one anywhere in the scope,
-- `_name` (the C symbol) is tried.  More than one match (two images, or two
-- values in one image) is an error that lists them: nothing is chosen.
-- Absolute symbols (sizes, bank ids) are found but take no breakpoint.
local json = require("json")

local M = {
	images = {},       -- name -> image
	order = {},        -- image names in load order
	breakpoints = {},  -- records from M.bp
	diagnostics = {},  -- warnings, newest last
}

local PROCESS_BANKS = 6  -- banks 0-5 hold processes and their segments
local APERTURE = 0x8000

local function fail(fmt, ...)
	error("konaos: " .. string.format(fmt, ...), 0)
end

local function warn(fmt, ...)
	local text = "konaos: " .. string.format(fmt, ...)
	M.diagnostics[#M.diagnostics + 1] = text
	emu.print_info(text)
end

local function read_file(path, what)
	local f = io.open(path, "rb")
	if not f then fail("%s %s is missing", what, path) end
	local data = f:read("a")
	f:close()
	return data
end

local function exists(path)
	local f = io.open(path, "rb")
	if f then f:close() end
	return f ~= nil
end

local function dirname(path)
	return path:match("^(.*)/[^/]*$") or "."
end

local function basename(path)
	return path:match("([^/]*)$")
end

local function share()
	local s = manager.machine.memory.shares[":sram"]
	if not s then fail("this machine has no :sram share (not zephyr80?)") end
	return s
end

-- Link maps -------------------------------------------------------------------

-- An sdld map: areas (address, size, attributes) and the global symbols
-- listed under each.
local function parse_map(path)
	local areas, symbols = {}, {}
	local area
	for line in read_file(path, "link map"):gmatch("[^\r\n]+") do
		if line:find("^Files Linked") then break end
		local name, addr, size, attrs = line:match("^(%S+)%s+(%x%x%x%x%x%x%x%x)%s+(%x%x%x%x%x%x%x%x)%s+=%s+%d+%.%s+bytes%s+%(([^)]*)%)")
		if not name and line:find("^%.%s+%.ABS%.") then
			name, addr, size, attrs = ".ABS.", "0", "0", "ABS"
		end
		if name then
			area = areas[name]
			if not area then
				area = { name = name, address = tonumber(addr, 16), size = tonumber(size, 16), abs = attrs:find("ABS") ~= nil }
				areas[name] = area
			end
		elseif area then
			local value, sym, module = line:match("^%s+(%x%x%x%x%x%x%x%x)%s+(%S+)%s*(%S*)%s*$")
			if value and not sym:find("^%-") then
				symbols[#symbols + 1] = { name = sym, value = tonumber(value, 16), area = area.name, module = module ~= "" and module or nil }
			end
		end
	end
	if not next(areas) then fail("%s is not an sdld link map", path) end
	return areas, symbols
end

-- Absolute values that are not addresses: area sizes and starts, bank ids.
local function constant(sym)
	return sym.area == ".ABS." or sym.name:find("^[ls]__") or sym.name:find("^b_") or sym.name:find("^%.__")
end

local function add_symbols(image, symbols)
	image.symbols, image.by_name = {}, {}
	local seen = {}
	for _, s in ipairs(symbols) do
		local key = s.name .. "@" .. s.value .. "@" .. s.area
		if not seen[key] then
			seen[key] = true
			s.image = image.name
			s.abs = constant(s) and true or false
			s.zone, s.segment = image.zone_of(s)
			image.symbols[#image.symbols + 1] = s
			local list = image.by_name[s.name] or {}
			list[#list + 1] = s
			image.by_name[s.name] = list
		end
	end
end

local function register(image)
	if not M.images[image.name] then M.order[#M.order + 1] = image.name end
	M.images[image.name] = image
	return image
end

-- The OS: konaos.map.  Zones by address (the nucleus's own placement).
function M.load_os(path)
	local areas, symbols = parse_map(path)
	if not areas["_CODE"] or not areas["_COMMON"] then
		fail("%s has no _CODE and _COMMON areas: not a KonaOS nucleus map", path)
	end
	local image = { name = "konaos", kind = "os", map = path, areas = areas }
	function image.zone_of(s)
		if s.value < 0x2000 then return "caller" end
		if s.value < 0xe000 then return "nucleus" end
		return "common"
	end
	add_symbols(image, symbols)
	register(image)
	M.install_launch_hook()
	return image
end

-- EXE v2 container (docs/exe-format.md): the primary payload and the rows.
local function u16(data, at) return data:byte(at + 1) | (data:byte(at + 2) << 8) end
local function u32(data, at) return u16(data, at) | (u16(data, at + 2) << 16) end

local function parse_exe(path)
	local data = read_file(path, "EXE")
	if #data < 36 or data:sub(1, 4) ~= "KEX2" then fail("%s is not an EXE v2 container", path) end
	local exe = { image_size = u16(data, 0x0c), entry = 0x100 + u16(data, 0x10), segments = {} }
	local primary = u32(data, 0x18)
	exe.primary = data:sub(primary + 1, primary + exe.image_size)
	local table_at = u32(data, 0x14)
	for i = 0, data:byte(0x0a + 1) - 1 do
		local row = table_at + 16 * i
		local id, kind, size, flags, payload = data:byte(row + 1), data:byte(row + 2), u16(data, row + 2), u16(data, row + 6), u32(data, row + 8)
		exe.segments[id] = { id = id, kind = kind == 1 and "FAR_CODE" or "DATA", mappable = kind == 1 or (flags & 1) ~= 0,
			bytes = data:sub(payload + 1, payload + size) }
	end
	return exe
end

-- A program, from its map path (build/user/<name>/<name>.map).
function M.load_program(path, name)
	local dir = dirname(path)
	name = name or basename(path):gsub("%.map$", "")
	local areas, symbols = parse_map(path)
	local placement_path = dir .. "/place/placement.json"
	local manifest_path = dir .. "/" .. name .. ".json"
	local exe_path = dirname(dir) .. "/" .. name .. ".exe"
	local placement = json.decode(read_file(placement_path, "placement"))
	local manifest = json.decode(read_file(manifest_path, "EXE manifest"))
	local image = { name = name, kind = "program", map = path, exe_path = exe_path, areas = areas,
		zones = {}, banks = {}, segment_banks = {} }
	if type(placement) == "table" and placement.home then
		-- The home area holds the home modules; its BSS area exists only
		-- when they have statics.
		local area = placement.home_area
		if not area or not areas[area] then
			fail("%s names the home area %s, which %s does not have", placement_path, tostring(area), path)
		end
		image.zones[area] = { zone = "home" }
		if placement.home_bss_area then image.zones[placement.home_bss_area] = { zone = "home" } end
	end
	image.rows = {}
	for _, row in ipairs(type(manifest) == "table" and manifest.segments or {}) do
		local zone = (row.kind == "FAR_CODE" and "far" or "data") .. row.id
		image.rows[row.id] = row
		for _, key in ipairs({ "area", "bss_area" }) do
			if row[key] then image.zones[row[key]] = { zone = zone, segment = row.id } end
		end
	end
	function image.zone_of(s)
		local z = image.zones[s.area]
		if z then return z.zone, z.segment end
		return "resident"
	end
	add_symbols(image, symbols)
	if exists(exe_path) then
		image.exe = parse_exe(exe_path)
	else
		warn("%s is missing: %s's banks cannot be found", exe_path, name)
	end
	register(image)
	M.locate(image)
	return image
end

-- Physical memory ----------------------------------------------------------------

-- SRAM bytes at physical address `phys` (bank << 16 | address), any bank,
-- whatever the latch maps; one byte as a number, or `n` as a string.
function M.sram(phys, n)
	local s = share()
	if not n then return s:read_u8(phys) end
	local out = {}
	for i = 0, n - 1 do out[i + 1] = string.char(s:read_u8(phys + i)) end
	return table.concat(out)
end

-- VRAM bytes as the V9958 stores them (MAME keeps GRAPHIC6/7 interleaved).
function M.vram(address, n)
	local space = manager.machine.devices[":vdp"].spaces["vram"]
	if not n then return space:read_u8(address) end
	local out = {}
	for i = 0, n - 1 do out[i + 1] = string.char(space:read_u8(address + i)) end
	return table.concat(out)
end

local function matches(s, phys, bytes, first, last)
	for i = first, last do
		if s:read_u8(phys + i - 1) ~= bytes:byte(i) then return false end
	end
	return true
end

-- The banks holding a program's resident code (from its EXE), optionally
-- only `only`; and the banks holding each far or mappable segment.
-- The resident code's bytes as indexes into the primary payload (0100h is 1).
local function code_range(image)
	local area = image.areas["_CODE"]
	local first = area.address - 0x100 + 1
	return first, math.min(first + area.size - 1, #image.exe.primary)
end

function M.locate(image, only)
	if type(image) == "string" then image = M.images[image] end
	if not image or image.kind ~= "program" or not image.exe or not image.areas["_CODE"] then return false end
	local s = share()
	local first, last = code_range(image)
	local found = {}
	for bank = 0, PROCESS_BANKS - 1 do
		if (only == nil or bank == only) and matches(s, (bank << 16) + 0x100, image.exe.primary, first, last) then
			found[#found + 1] = bank
		end
	end
	if only == nil or #found > 0 then
		image.banks = found
		image.segment_banks = {}
		for id, seg in pairs(image.exe.segments) do
			local banks = {}
			if seg.mappable and #seg.bytes > 0 then
				for bank = 0, PROCESS_BANKS - 1 do
					if matches(s, (bank << 16) + APERTURE, seg.bytes, 1, #seg.bytes) then banks[#banks + 1] = bank end
				end
			end
			image.segment_banks[id] = banks
		end
	end
	return #found > 0
end

-- Names -------------------------------------------------------------------------

local function describe(s)
	return string.format("%s:%s (%s, %04X%s)", s.image, s.name, s.abs and "absolute" or s.zone, s.value,
		s.module and (" in " .. s.module) or "")
end

-- Every symbol a spec names, by the rules above.
function M.find(spec)
	local scope, name = spec:match("^([^:]+):(.+)$")
	local images = {}
	if scope then
		if not M.images[scope] then
			fail("no image %s is loaded (loaded: %s)", scope, #M.order > 0 and table.concat(M.order, ", ") or "none")
		end
		images = { M.images[scope] }
	else
		name = spec
		for _, n in ipairs(M.order) do images[#images + 1] = M.images[n] end
	end
	local function collect(want)
		local out = {}
		for _, image in ipairs(images) do
			for _, s in ipairs(image.by_name[want] or {}) do out[#out + 1] = s end
		end
		return out
	end
	local out = collect(name)
	if #out == 0 and not name:find("^_") then out = collect("_" .. name) end
	return out
end

-- The one symbol a spec names, or an error saying why not.
function M.resolve(spec)
	if #M.order == 0 then fail("no symbols loaded: konaos.load_os(map) or konaos.load_program(map)") end
	local found = M.find(spec)
	if #found == 0 then fail("no symbol %s in %s", spec, spec:find(":") and "that image" or table.concat(M.order, ", ")) end
	if #found > 1 then
		local list = {}
		for i, s in ipairs(found) do list[i] = describe(s) end
		fail("%s is ambiguous: %s; name it as image:symbol", spec, table.concat(list, "; "))
	end
	return found[1]
end

-- Where a symbol is: logical address, zone and the physical addresses its
-- image occupies now (empty while it is not loaded), or why it has none.
function M.where(spec)
	local s = type(spec) == "table" and spec or M.resolve(spec)
	local info = { symbol = s, logical = s.value, zone = s.zone, phys = {} }
	if s.abs then
		info.why = "an absolute value, not an address"
	elseif s.image == "konaos" then
		if s.zone == "caller" then
			info.why = "0000h-1FFFh is the calling process's bank in mode 11: no one physical bank"
		else
			info.phys = { ((s.zone == "common" and 0 or 7) << 16) | s.value }
		end
	else
		local image = M.images[s.image]
		local banks = s.segment and image.segment_banks[s.segment] or image.banks
		if not image.exe then
			info.why = image.exe_path .. " is missing"
		elseif s.segment and not (image.exe.segments[s.segment] or {}).mappable then
			info.why = "segment " .. s.segment .. " is not mappable at 8000h"
		elseif #banks == 0 then
			info.why = "not in memory yet"
		end
		for _, b in ipairs(banks) do info.phys[#info.phys + 1] = (b << 16) | s.value end
	end
	local parts = {}
	for i, p in ipairs(info.phys) do parts[i] = string.format("%05X", p) end
	info.text = string.format("%s at %04X, physical %s", describe(s), s.value,
		#parts > 0 and table.concat(parts, " or ") or ("none: " .. info.why))
	return info
end

-- Breakpoints ---------------------------------------------------------------------

local function cpu_debug()
	local cpu = manager.machine.devices[":maincpu"]
	if not cpu.debug then fail("breakpoints need the debugger: start MAME with -debug") end
	return cpu.debug
end

-- (Re)set a record's breakpoint for the banks its image occupies now.
local function arm(record)
	local debug = cpu_debug()
	if record.index then
		debug:bpclear(record.index)
		record.index = nil
	end
	local info = M.where(record.symbol)
	record.phys = info.phys
	record.text = info.text
	if #info.phys == 0 then
		record.pending = info.why
		return record
	end
	local terms = {}
	for i, p in ipairs(info.phys) do terms[i] = string.format("phys(pc)==%X", p) end
	record.condition = table.concat(terms, " || ")
	record.index = debug:bpset(record.symbol.value, record.condition, record.action or "")
	record.pending = nil
	return record
end

-- A breakpoint at a symbol, stopping only in its physical bank.  `action`
-- is a debugger command run at the stop.  Returns the record: index,
-- condition, phys (the physical addresses), text, or pending (why not yet).
function M.bp(spec, action)
	local s = M.resolve(spec)
	if s.abs then fail("%s is an absolute value, not an address", describe(s)) end
	if s.image == "konaos" and s.zone == "caller" then fail("%s: %s", describe(s), M.where(s).why) end
	local record = { spec = spec, symbol = s, action = action }
	M.breakpoints[#M.breakpoints + 1] = record
	return arm(record)
end

function M.bpclear(record)
	for i = #M.breakpoints, 1, -1 do
		local r = M.breakpoints[i]
		if record == nil or r == record then
			if r.index then cpu_debug():bpclear(r.index) end
			table.remove(M.breakpoints, i)
		end
	end
end

-- Re-arm the breakpoints of an image (after its banks were found again).
local function rearm(image)
	if not manager.machine.devices[":maincpu"].debug then return end
	for _, r in ipairs(M.breakpoints) do
		if r.symbol.image == image.name then arm(r) end
	end
end

-- A launch: the nucleus's kos_process_launch receives HL = the request
-- (+0 bank, +1 entry; kernel/include/process.h).  The child is loaded by
-- then, so its bank and segments are found before its first instruction.
function M.install_launch_hook()
	local os_image = M.images["konaos"]
	local launch = os_image and (os_image.by_name["_kos_process_launch"] or {})[1]
	if not launch then
		warn("%s has no _kos_process_launch: program banks are found only by konaos.locate", os_image and os_image.map or "no OS map")
		return
	end
	local program = manager.machine.devices[":maincpu"].spaces["program"]
	if M.launch_tap then M.launch_tap:remove() end
	M.launches = 0
	M.launch_tap = program:install_read_tap(launch.value, launch.value, "konaos-launch", function(offset, data)
		local hl = manager.machine.devices[":maincpu"].state["HL"].value
		local bank, entry = program:read_u8(hl), program:read_u8(hl + 1) | (program:read_u8(hl + 2) << 8)
		if bank >= PROCESS_BANKS then return end
		M.launches = M.launches + 1
		for _, n in ipairs(M.order) do
			local image = M.images[n]
			if image.kind == "program" and image.exe and image.exe.entry == entry and M.locate(image, bank) then
				rearm(image)
			end
		end
	end)
end

-- What runs at the stopped PC: phys(pc) from the debugger, then the image
-- and the nearest symbol at or below it in that bank.
function M.phys(expression)
	cpu_debug()
	local log = manager.machine.debugger.consolelog
	local before = #log
	manager.machine.debugger:command("print phys(" .. (expression or "pc") .. ")")
	local value = tonumber((log[#log] or ""):match("^(%x+)$") or "", 16)
	if #log == before or not value then fail("phys() is not available: %s", log[#log] or "no reply") end
	return value
end

function M.here()
	local phys = M.phys("pc")
	local bank, address = phys >> 16, phys & 0xffff
	local best
	for _, n in ipairs(M.order) do
		local image = M.images[n]
		for _, s in ipairs(image.symbols) do
			if not s.abs and s.value <= address then
				local info = M.where(s)
				for _, p in ipairs(info.phys) do
					if p >> 16 == bank and (not best or s.value > best.value) then best = s end
				end
			end
		end
	end
	if not best then return string.format("%05X: no loaded symbol", phys) end
	return string.format("%05X: %s+%X", phys, describe(best), address - best.value)
end

return M
