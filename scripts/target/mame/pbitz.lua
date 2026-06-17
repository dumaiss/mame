-- license:BSD-3-Clause

---------------------------------------------------------------------------
--
--   pbitz.lua
--
--   pBITz driver-specific makefile.
--   Use make SUBTARGET=pbitz to build.
--
---------------------------------------------------------------------------

CPUS["Z80"] = true
MACHINES["Z80DAISY"] = true

function createProjects_mame_pbitz(_target, _subtarget)
	project ("mame_pbitz")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-pbitz"))
	addprojectflags()

	defines {
		"PBITZ_RS232_MINIMAL",
	}

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "src/lib/netlist",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
		ext_includedir("asio"),
		ext_includedir("flac"),
		ext_includedir("glm"),
		ext_includedir("jpeg"),
		ext_includedir("rapidjson"),
		ext_includedir("zlib"),
	}

	files {
		MAME_DIR .. "src/devices/bus/rs232/loopback.cpp",
		MAME_DIR .. "src/devices/bus/rs232/loopback.h",
		MAME_DIR .. "src/devices/bus/rs232/pty.cpp",
		MAME_DIR .. "src/devices/bus/rs232/pty.h",
		MAME_DIR .. "src/devices/bus/rs232/rs232.cpp",
		MAME_DIR .. "src/devices/bus/rs232/rs232.h",
		MAME_DIR .. "src/devices/machine/z80sio.cpp",
		MAME_DIR .. "src/devices/machine/z80sio.h",
		MAME_DIR .. "src/mame/pbitz/coffeez80.cpp",
		MAME_DIR .. "src/mame/pbitz/pbitz_coffeeio.cpp",
		MAME_DIR .. "src/mame/pbitz/pbitz_coffeeio.h",
		MAME_DIR .. "src/mame/pbitz/pbitz_memctl.cpp",
		MAME_DIR .. "src/mame/pbitz/pbitz_memctl.h",
		MAME_DIR .. "src/mame/pbitz/pbitz_serial.cpp",
		MAME_DIR .. "src/mame/pbitz/pbitz_serial.h",
		MAME_DIR .. "src/mame/pbitz/pbitz_zsio.cpp",
		MAME_DIR .. "src/mame/pbitz/pbitz_zsio.h",
		MAME_DIR .. "src/mame/pbitz/zephyr80.cpp",
		MAME_DIR .. "src/mame/pbitz/zephyr80_map.h",
		MAME_DIR .. "src/mame/pbitz/zephyr_banktst.cpp",
		MAME_DIR .. "src/mame/pbitz/zephyr_siotop.cpp",
	}
end

function linkProjects_mame_pbitz(_target, _subtarget)
	links {
		"mame_pbitz",
	}
end
