-- license:BSD-3-Clause
-- copyright-holders:Kitamura
-- The KonaOS V9958 console as text, and typing on the USB keyboard.
--
-- The console draws 6x8 glyphs into a GRAPHIC6 bitmap (85x26): page 0 is a
-- circular 256-line surface, text row r starts at scanline R#23 + 4 + 8r
-- (mod 256), column c covers the bytes 3c..3c+2 of a line; paper is colour
-- 4, and a reverse-video cell swaps ink and paper.  Cells are matched against
-- the font the image ships: its ROMFS v9958.mod, the KMOD `mapped` segment
-- (eight 256-byte pages, page s holding row s of every character), read
-- from the -flash file.  VRAM is read through the chip's own address space
-- (logical byte L at ((L & 1) << 16) | (L >> 1)) and the registers from its
-- saved state, so reading touches no port and changes nothing the guest
-- sees.  Sprites (the cursor, the pointer) are not part of the bitmap.
local C = { columns = 85, rows = 26, UNKNOWN = "\1" }

local function fail(fmt, ...)
	error("konaos: " .. string.format(fmt, ...), 0)
end

local function u16(data, at) return data:byte(at + 1) | (data:byte(at + 2) << 8) end
local function u32(data, at) return u16(data, at) | (u16(data, at + 2) << 16) end

-- The ROMFS: "KRFS" at flash page 2, logical bytes spread over the 0000h-
-- BFFFh window of pages 2-7.
local function romfs_read(image, logical, length)
	local out = {}
	while length > 0 do
		local page, offset = 2 + logical // 0xc000, logical % 0xc000
		local take = math.min(length, 0xc000 - offset)
		out[#out + 1] = image:sub(page * 0x10000 + offset + 1, page * 0x10000 + offset + take)
		logical, length = logical + take, length - take
	end
	return table.concat(out)
end

-- The console font, glyph-major (code * 8 + row), from a KonaOS flash image.
function C.font_from_image(path)
	local f = io.open(path, "rb")
	if not f then fail("flash image %s cannot be read for the console font", path) end
	local image = f:read("a")
	f:close()
	local header = romfs_read(image, 0, 16)
	if header:sub(1, 4) ~= "KRFS" then fail("%s has no KonaOS ROMFS: the console font is unknown", path) end
	local entry_size, count = header:byte(6), u16(header, 6)
	local directory = romfs_read(image, 16, entry_size * count)
	for i = 0, count - 1 do
		local entry = directory:sub(i * entry_size + 1, (i + 1) * entry_size)
		if entry:sub(1, 24):match("^[^%z]*") == "v9958.mod" then
			local mod = romfs_read(image, u32(entry, 24), u32(entry, 28))
			if mod:sub(1, 4) ~= "KMOD" then fail("v9958.mod is not a KMOD container") end
			local payload = u16(mod, 0x0a)
			for s = 0, mod:byte(5) - 1 do
				local row = 16 + 12 * s
				if mod:byte(row + 1) == 5 and u16(mod, row + 4) == 2048 then
					local pages = mod:sub(payload + u16(mod, row + 10) + 1, payload + u16(mod, row + 10) + 2048)
					local glyphs = {}
					for code = 0, 255 do
						for r = 0, 7 do glyphs[#glyphs + 1] = pages:sub(r * 256 + code + 1, r * 256 + code + 1) end
					end
					return table.concat(glyphs)
				end
			end
			fail("v9958.mod has no 2048-byte mapped font segment")
		end
	end
	fail("the ROMFS of %s holds no v9958.mod", path)
end

-- Load the font (default: the -flash image's).  Codes sharing a bitmap read
-- as the first printable ASCII one, then the lowest.
function C.load_font(path)
	if not path then
		local flash = manager.machine.images[":flash"]
		path = flash and flash.filename
		if not path or path == "" then fail("no -flash image to read the console font from") end
	end
	local data = C.font_from_image(path)
	local order = {}
	for code = 0x20, 0x7e do order[#order + 1] = code end
	for code = 0, 255 do
		if code < 0x20 or code > 0x7e then order[#order + 1] = code end
	end
	C.glyphs = {}
	for _, code in ipairs(order) do
		local rows = {}
		for r = 1, 8 do rows[r] = string.char(data:byte(code * 8 + r) & 0xfc) end
		local key = table.concat(rows)
		if C.glyphs[key] == nil then C.glyphs[key] = code end
	end
	return path
end

local function vdp()
	local device = manager.machine.devices[":vdp"]
	return device.spaces["vram"], emu.item(device.items["0/m_cont_reg"])
end

local FULL = string.rep("\252", 8)

-- The screen: rows (strings of character codes, C.UNKNOWN for a cell that
-- matches no glyph), or nil and why the VDP is not showing the console.
function C.read()
	if not C.glyphs then C.load_font() end
	local vram, regs = vdp()
	local mode = ((regs:read(0) & 0x0e) << 1) | ((regs:read(1) & 0x18) >> 3)
	if mode ~= 0x14 then return nil, "the VDP is not in GRAPHIC6" end
	if (regs:read(2) & 0x20) ~= 0 then return nil, "R#2 shows page 1, not the console page" end
	local linemask = ((regs:read(2) & 0x1f) << 3) | 7
	local origin = (regs:read(23) + 4) & 0xff
	local pair = {}
	for b = 0, 255 do pair[b] = (((b >> 4) ~= 4) and 2 or 0) | (((b & 15) ~= 4) and 1 or 0) end
	local rows = {}
	for row = 0, C.rows - 1 do
		local cells = {}
		for c = 1, C.columns do cells[c] = {} end
		for k = 0, 7 do
			local y = (origin + row * 8 + k) & linemask
			local even = vram:read_range(y * 128, y * 128 + 127, 8)
			local odd = vram:read_range(0x10000 + y * 128, 0x10000 + y * 128 + 127, 8)
			for c = 0, C.columns - 1 do
				local i = 3 * c -- logical bytes i, i+1, i+2 of the line
				local function byte(n) return (n & 1) == 0 and even:byte(n // 2 + 1) or odd:byte(n // 2 + 1) end
				cells[c + 1][k + 1] = (pair[byte(i)] << 6) | (pair[byte(i + 1)] << 4) | (pair[byte(i + 2)] << 2)
			end
		end
		local codes = {}
		for c = 1, C.columns do
			local m = cells[c]
			local key = string.char(m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8])
			local code = C.glyphs[key]
			if key == FULL then
				code = 0x20
			elseif code == nil then
				key = string.char(m[1] ~ 0xfc, m[2] ~ 0xfc, m[3] ~ 0xfc, m[4] ~ 0xfc, m[5] ~ 0xfc, m[6] ~ 0xfc, m[7] ~ 0xfc, m[8] ~ 0xfc)
				code = C.glyphs[key] or 1
			end
			codes[c] = string.char(code)
		end
		rows[row + 1] = table.concat(codes)
	end
	return rows
end

-- A row as printable text: ASCII as is, unknown cells '?', other codes '.'.
function C.printable(line)
	return (line:gsub("[^\32-\126]", function(c) return c == C.UNKNOWN and "?" or "." end))
end

function C.text(rows)
	rows = rows or C.read()
	if not rows then return nil end
	local out = {}
	for i, r in ipairs(rows) do out[i] = C.printable(r):gsub("%s+$", "") end
	return table.concat(out, "\n")
end

-- The first row and column holding `text`, or nil.
function C.find(text, rows)
	rows = rows or C.read()
	for i, line in ipairs(rows or {}) do
		local column = line:find(text, 1, true)
		if column then return i, column end
	end
	return nil
end

-- Capture: every `every` frames (default 6), a changed screen is appended
-- to `path` with its emulated time.  C.stop() ends it.
function C.capture(path, every)
	C.stop()
	local f = io.open(path, "w")
	if not f then fail("cannot write the console capture %s", path) end
	local frames, last = 0, nil
	C.capture_file = f
	C.capture_handle = emu.add_machine_frame_notifier(function()
		frames = frames + 1
		if frames % (every or 6) ~= 0 then return end
		local ok, text = pcall(C.text)
		if ok and text and text ~= last then
			last = text
			f:write(string.format("--- %.3f s\n%s\n", manager.machine.time:as_double(), text))
			f:flush()
		end
	end)
	return path
end

function C.stop()
	if C.capture_handle then
		C.capture_handle:unsubscribe()
		C.capture_handle = nil
		C.capture_file:close()
		C.capture_file = nil
	end
end

-- The USB keyboard -----------------------------------------------------------------

-- Keys go through MAME's input path: the keyboard device's fields
-- (:ioc:keyboard, one per HID usage), which the next frame's input update
-- applies like a host key.  US legends (the firmware's translation table).
local K = { usage = {}, hold = 2, gap = 2 }
C.keys = K

local US = {}
do
	local lower = "abcdefghijklmnopqrstuvwxyz1234567890"
	local upper = "ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()"
	for i = 1, #lower do
		US[lower:sub(i, i)] = { 0x03 + i, false }
		US[upper:sub(i, i)] = { 0x03 + i, true }
	end
	local plain, shifted = " -=[]\\;'`,./", " _+{}|:\"~<>?"
	local usages = { 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 }
	for i = 1, #plain do
		US[plain:sub(i, i)] = { usages[i], false }
		if i > 1 then US[shifted:sub(i, i)] = { usages[i], true } end
	end
	US["\r"] = { 0x28, false }
	US["\t"] = { 0x2b, false }
	US["\b"] = { 0x2a, false }
end

local function fields()
	if next(K.usage) then return K.usage end
	for tag, port in pairs(manager.machine.ioport.ports) do
		local index = tag:match("^:ioc:keyboard:KEYS(%d)$")
		local base = index and tonumber(index) * 16 or (tag == ":ioc:keyboard:MODIFIERS" and 0xe0)
		if base then
			for _, field in pairs(port.fields) do
				local bit = 0
				while (field.mask >> bit) & 1 == 0 do bit = bit + 1 end
				K.usage[base + bit] = field
			end
		end
	end
	if not next(K.usage) then fail("no :ioc:keyboard fields: not the zephyr80 machine") end
	return K.usage
end

-- The key presses that type `text`, as a queue K.pump drains one frame at a
-- time: { usage, down } steps and frame gaps.
function K.queue(text)
	local f = fields()
	local steps = K.steps or {}
	for i = 1, #text do
		local map = US[text:sub(i, i)]
		if not map then fail("no key types %q", text:sub(i, i)) end
		local chord = map[2] and { 0xe1, map[1] } or { map[1] }
		for _, u in ipairs(chord) do steps[#steps + 1] = { f[u], 1 } end
		for _ = 1, K.hold do steps[#steps + 1] = false end
		for j = #chord, 1, -1 do steps[#steps + 1] = { f[chord[j]], 0 } end
		for _ = 1, K.gap do steps[#steps + 1] = false end
	end
	K.steps = steps
	if not K.handle then
		K.handle = emu.add_machine_frame_notifier(function()
			local step = table.remove(K.steps, 1)
			if step then step[1]:set_value(step[2]) end
		end)
	end
end

-- True once every queued key has been pressed and released.
function K.idle()
	return not K.steps or #K.steps == 0
end

return C
