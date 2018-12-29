zephyr_include_directories(.)
zephyr_sources_ifndef(
  CONFIG_SOC_SERIES_SAM4L4
  soc_pmc.c
  soc_gpio.c
  )

zephyr_sources_ifdef(
  CONFIG_ARM_MPU
  arm_mpu_regions.c
  )
