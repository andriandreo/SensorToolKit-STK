idf_component_register(SRCS
                            # "Applications/Amperometry/AD5940_AMPmain.c" "Applications/Amperometry/Amperometric.c"
                            # "Applications/Open Circuit Potentiometry/AD5940_ADCSeqMUX.c" 
                            "Applications/Open Circuit Potentiometry/AD5940_ADCSeqMUX_MQTT.c"
                            # "Applications/Open Circuit Potentiometry/AD5940_OCP-ADCPolling.c"
                            # "Applications/Open Circuit Potentiometry/AD5940_OCP-ADCPolling_MQTT.c"
                            
                       INCLUDE_DIRS "."
                       REQUIRES ad5940lib ESP32 driver freertos
)
