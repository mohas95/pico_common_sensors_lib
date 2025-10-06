# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-src/enc_bootloader"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/tmp"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/src"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build/enc_bootloader/src/enc_bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
