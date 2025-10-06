# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-src"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-build"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/tmp"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/src/picotool-populate-stamp"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/src"
  "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/src/picotool-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/src/picotool-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mohas95/repo/pico_common_sensors_lib/.pico-deps/picotool-subbuild/picotool-populate-prefix/src/picotool-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
