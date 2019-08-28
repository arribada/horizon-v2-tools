# Arribada Horizon tag V3 cellular setup

1. Create AWS keys for a specific device, replacing "namespace" with the AWS namespace provided and "yourThingName" with a unique tag identifier:

   ``aws_config --namespace namespace --register_thing yourThingName``

   This will create three files:
      * ``yourThingName.cert``
      * ``yourThingName.key``
      * ``yourThingName.pubkey``

2. Apply certificate and keys to device (using VeriSign-Class-3-Public-Primary-Certification-Authority-G5.pem file provided):

   ``sudo cellular_config --root_ca VeriSign-Class-3-Public-Primary-Certification-Authority-G5.pem --cert yourThingName.cert --key yourThingName.key``

3. Get the AWS endpoint:

   ``aws_config --get_iot_endpoint``

4. Create unique config file for a tag which includes the following values, replacing xxxxxxxx with the credentials given by the endpoint:

   ```
    {
        "version": 1, # Recommended to be set to 1 for first file version
        "iot": {
            "cellular": {
                "aws": {
                    "arn": "xxxxxxxxx.iot.us-west-2.amazonaws.com",
                    "thingName": ""
                }
            }
            ...
        }
        ...
    }
   ```
    
   If thingName is kept blank then the deviceName will be used instead.

5. In the config file, include the network information (name, password and username), found online:
   ```
   {
        "version": 1, # Recommended to be set to 1 for first file version
        "iot": {
           ...
           "apn": {
              "name": "internetd.gdsp", 
              "password": "", 
              "username": ""
           }
        }
        ...
    }
    ```

6. Verify battery level is at 100:

   ``sudo tracker_config --battery``

7. Reset flash:

   ``sudo tracker_config --reset FLASH``
   
8. Reset CPU:

   ``sudo tracker_config --reset CPU``
    
9. Erase existing config:

    ``sudo tracker_config --erase``

10. Update almanac:

    ``sudo gps_almanac --file mgaoffline.ubx``

11. Program the GPS:

    ``sudo gps_ascii_config --file ublox_gnss_configuration.dat``

12. Apply configuration file:

    ``sudo tracker_config --write yourThingName.json --setdatetimeutc``

12. Apply cellular configuration:

    ``sudo cellular_config --root_ca VeriSign-Class-3-Public-Primary-Certification-Authority-G5.pem --cert yourThingName.cert --key yourThingName.key``

13. Test cellular connection:

    ``sudo tracker_config --test_mode CELLULAR``

14. Use LEDs to confirm test success:

| LED action      | Meaning                                                                                          |
|-----------------|--------------------------------------------------------------------------------------------------|
|Flashing White   |   GPS test fix                                                                                   |
|Solid White      |   GPS test fix locked and fixed for minimum required time period. LED will stay on for 5 seconds |
|Flashing Yellow  |   Test cellular data connection in progress                                                      |
|Solid Yellow     |   Test cellular data connection made and IoT message success LED will stay on for 5 seconds      |
|Flashing Blue    |   Test message to satellite sending in progress                                                  |
|Solid Blue       |   Test satellite message sent LED will stay on for 5 seconds                                     |

15. Test GPS connection:

    ``sudo tracker_config --test_mode GPS``

16. Create new log file (previous log was deleted when flash was reset):
    
    ``sudo tracker_config --create_log LINEAR``
