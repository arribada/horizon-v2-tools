# Arribada Horizon tag V3 quickstart

1. Reset flash:

   ``sudo tracker_config --reset FLASH``
   
2. Reset CPU:

   ``sudo tracker_config --reset CPU``
    
3. Erase existing config:

    ``sudo tracker_config --erase``

4. Update almanac:

    ``sudo gps_almanac --file mgaoffline.ubx``

5. Program the GPS:

    ``sudo gps_ascii_config --file ublox_gnss_configuration.dat``

6. Apply configuration file:

    ``sudo tracker_config --write yourThingName.json --setdatetimeutc``

7. Apply cellular configuration:

    ``sudo cellular_config --root_ca VeriSign-Class-3-Public-Primary-Certification-Authority-G5.pem --cert yourThingName.cert --key yourThingName.key``

8. Create new log file (previous log was deleted when flash was reset):
    
    ``sudo tracker_config --create_log LINEAR``

9. Test cellular connection:

    ``sudo tracker_config --test_mode CELLULAR``

10. Use LEDs to confirm test success:

| LED action      | Meaning                                                                                          |
|-----------------|--------------------------------------------------------------------------------------------------|
|Flashing White   |   GPS test fix                                                                                   |
|Solid White      |   GPS test fix locked and fixed for minimum required time period. LED will stay on for 5 seconds |
|Flashing Yellow  |   Test cellular data connection in progress                                                      |
|Solid Yellow     |   Test cellular data connection made and IoT message success LED will stay on for 5 seconds      |
|Flashing Blue    |   Test message to satellite sending in progress                                                  |
|Solid Blue       |   Test satellite message sent LED will stay on for 5 seconds                                     |

First time cellular tests can take a while.
