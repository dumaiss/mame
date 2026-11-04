-- license:BSD-3-Clause
-- copyright-holders:Kitamura
-- A run session (the SDK's `make run`): wait for the shell prompt on the
-- console, type the program's command, wait for the prompt to return and
-- report.  The session file holds one `key<TAB>value` per line:
--
--   command     the line to type (/fs/HELLO.EXE args)
--   prompt      the shell prompt (default "/> ", the card root)
--   osmap       konaos.map, for symbols and launches (optional)
--   program     a program's link map (optional, repeatable)
--   break       a symbol to break at (repeatable; needs -debug)
--   capture     the file every changed screen is appended to
--   result      the file the outcome is written to
--   exit        1: end MAME once the program has returned
--   timeout     emulated seconds for the prompt and for the program (60)
--
-- The result file's first line is `ran`, `no-prompt`, `no-return` or
-- `error`, then the detail; the program's output (the screen rows between
-- the command and the next prompt) follows.
local S = {}


local function parse(path)
	local f = io.open(path, "r")
	if not f then error("konaos: session file " .. path .. " is missing", 0) end
	local conf = { program = {}, ["break"] = {} }
	for line in f:lines() do
		local key, value = line:match("^([%w_]+)\t(.*)$")
		if key then
			if type(conf[key]) == "table" then
				table.insert(conf[key], value)
			else
				conf[key] = value
			end
		end
	end
	f:close()
	return conf
end

function S.start(api, path)
	local conf = parse(path)
	local console = api.console
	local keys = api.keys
	local prompt = conf.prompt or "/> "
	local timeout = tonumber(conf.timeout or "60")
	local done = false

	local function finish(status, detail, output)
		if done then return end
		done = true
		local f = conf.result and io.open(conf.result, "w")
		if f then
			f:write(status, "\n", detail or "", "\n")
			if output then f:write(output, "\n") end
			f:close()
		end
		emu.print_info(string.format("konaos: %s: %s", status, detail or ""))
		if output then print(output) end
		if conf.exit == "1" then manager.machine:exit() end
	end

	local function wait(predicate, seconds, what, status)
		local limit = manager.machine.time:as_double() + seconds
		while true do
			local value = predicate()
			if value then return value end
			if manager.machine.time:as_double() > limit then
				error({ session = status, detail = string.format("no %s within %d emulated s", what, seconds) }, 0)
			end
			coroutine.yield()
		end
	end

	-- The row index of the last row starting with the prompt, and the rest.
	local function last_prompt(rows)
		for i = #rows, 1, -1 do
			if rows[i]:sub(1, #prompt) == prompt then return i, rows[i]:sub(#prompt + 1):gsub("%s+$", "") end
		end
	end

	local body = coroutine.create(function()
		if conf.osmap and conf.osmap ~= "" then api.load_os(conf.osmap) end
		for _, map in ipairs(conf.program) do api.load_program(map) end
		console.load_font()
		if conf.capture then console.capture(conf.capture) end
		wait(function()
			local rows = console.read()
			if not rows then return nil end
			local i, rest = last_prompt(rows)
			return i and rest == ""
		end, timeout, "shell prompt " .. string.format("%q", prompt), "no-prompt")
		for _, spec in ipairs(conf["break"]) do
			local r = api.bp(spec)
			emu.print_info("konaos: breakpoint " .. (r.pending and (spec .. " pending (" .. r.pending .. ")") or r.text))
		end
		local started = manager.machine.time:as_double()
		keys.queue(conf.command .. "\r")
		wait(keys.idle, 10, "keyboard", "error")
		-- The command line, then a fresh prompt below it: the program returned.
		local command_row, output
		wait(function()
			local rows = console.read()
			if not rows then return nil end
			command_row = nil
			for i = #rows, 1, -1 do
				if rows[i]:sub(1, #prompt + #conf.command) == prompt .. conf.command then command_row = i break end
			end
			local i, rest = last_prompt(rows)
			if command_row and i and i > command_row and rest == "" then
				local out = {}
				for r = command_row + 1, i - 1 do out[#out + 1] = console.printable(rows[r]):gsub("%s+$", "") end
				output = table.concat(out, "\n")
				return true
			end
		end, timeout, "return to the prompt after " .. conf.command, "no-return")
		finish("ran", string.format("%s returned to the prompt %.3f emulated s after the command", conf.command,
			manager.machine.time:as_double() - started), output)
	end)

	-- A stop at one of the session's breakpoints is reported once (the
	-- debugger's periodic callback runs while it is stopped).
	local reported, started
	emu.register_periodic(function()
		if not started then return end
		local debugger = manager.machine.debugger
		if not debugger or debugger.execution_state ~= "stop" then
			reported = nil
			return
		end
		local cpu = manager.machine.devices[":maincpu"]
		local pc = cpu.state["PC"].value
		if reported == pc then return end
		for _, r in ipairs(api.breakpoints) do
			if r.index and r.symbol.value == pc then
				reported = pc
				emu.print_info(string.format("konaos: stopped at %s:%s (%s)", r.symbol.image, r.symbol.name, api.here()))
				break
			end
		end
	end)

	S.handle = emu.add_machine_frame_notifier(function()
		started = true
		if done or coroutine.status(body) ~= "suspended" then return end
		local ok, err = coroutine.resume(body)
		if not ok then
			if type(err) == "table" then
				finish(err.session, err.detail, console.text and select(2, pcall(console.text)) or nil)
			else
				finish("error", tostring(err))
			end
		end
	end)
end

return S
