cmake_minimum_required(VERSION 3.20)
zephyr_library()
zephyr_library_sources(trackpoint_i2c.c)
zephyr_library_sources_ifdef(CONFIG_TRACKPOINT_SPI trackpoint_spi.c)
