if(CONFIG_LD2410S_SENSOR)
      set(sensor_src "ld2410s.c")
endif()

if(CONFIG_LD2420_SENSOR)
      set(sensor_src "ld2402.c")
endif()



idf_component_register(
    SRCS
    "main.c"
    "clock.c"
    "${sensor_src}"
    "light_driver.c"
    "ota.c"
    INCLUDE_DIRS "."
)
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS "-save-temps")
