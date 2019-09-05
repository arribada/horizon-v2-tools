import struct
import logging
import sys
import json
import inspect
import binascii
import datetime
import dateutil.parser

__version__ = 4

logger = logging.getLogger(__name__)


class ExceptionConfigInvalidValue(Exception):
    pass


class ExceptionConfigInvalidJSONField(Exception):
    pass


def _flatdict(base, olddict, newdict):
    """Convert a JSON dict to a flat dict i.e., nested dictionaries
    are assigned using dotted notation to represent hierarchy e.g.,
    bluetooth.advertising"""
    for i in olddict.keys():
        if isinstance(olddict[i], dict):
            _flatdict(base + ('.' if base else '') + i, olddict[i], newdict)
        else:
            newdict[base + ('.' if base else '') + i] = olddict[i]
    return newdict


def _pathsplit(fullpath):
    """Splits out the fullpath into a tuple comprising the variable name
    (i.e., given x.y.z, this would be z) and the root path (i.e., would
    be x.y given the previous example)"""
    items = fullpath.split('.')
    return ('.'.join(items[:-1]), items[-1])


def _findclass(fullpath):
    """Give a path name we identify to which configuration class it
    belongs.  The path uniquely identifies every configuration value
    and the root path denotes which class it belongs i.e., this code 
    performs a reverse search across all classes."""
    (path, param) = _pathsplit(fullpath)
    for i in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        cls = i[1]
        if issubclass(cls, ConfigItem) and cls != ConfigItem and path == cls.path and \
            param in cls.json_params:
            return (path, param, cls)
    return (None, None, None)


def decode(data):
    """Attempt to decode a single configuration item from an input data buffer.
    A tuple is returned containing an instance of the configuration object plus
    the input data buffer, less the amount of data consumed."""
    item = TaggedItem()
    if (len(data) < item.header_length):
        return (None, data)

    # Unpack header tag at current position
    item.unpack(data)

    # Find the correct configuration class based on the configuration tag
    cfg = None
    for i in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        cls = i[1]
        if issubclass(cls, ConfigItem) and cls != ConfigItem and \
            item.tag == cls.tag:
            cfg = cls()
            break
    if (cfg):
        try:
            cfg.unpack(data)
        except:
            # Likely insufficient bytes to unpack
            return (None, data)
        # Advance buffer past this configuration item
        data = data[cfg.length:]

    return (cfg, data)


def decode_all(data):
    """Iteratively decode an input data buffer to a list of configuration
    objects.
    """
    objects = []
    while data:
        (cfg, data) = decode(data)
        if cfg:
            objects += [ cfg ]
        else:
            break
    return objects


def encode_all(objects):
    """Encode a list of configuration objects, in order, to a serial byte
    stream.
    """
    data = b''
    for i in objects:
        data += i.pack()
    return data


def json_dumps(objects):
    """Convert a list of configuration objects representing a configuration
    set to JSON format."""
    obj_hash = {}
    for i in objects:
        h = obj_hash
        p = i.path.split('.')
        for j in p:
            if j not in h:
                h[j] = {}
            h = h[j]
        for j in i.json_params:
            h[j] = getattr(i, j)
    return json.dumps(obj_hash, indent=4, sort_keys=True)


def json_loads(text):
    """Convert JSON text representing a configuration set to a list
    of configuration objects."""
    obj = {}
    flat = _flatdict('', json.loads(text), {})
    for i in flat:
        (path, param, cls) = _findclass(i)
        if cls:
            if len(cls.params) == 1:
                path = i
            if path not in obj:
                obj[path] = cls()
            setattr(obj[path], param, flat[i])
        else:
            raise ExceptionConfigInvalidJSONField('Could not find %s' % i)
    return [obj[i] for i in obj]


class _Blob(object):
    """Blob object is a container for arbitrary message fields which
    can be packed / unpacked using python struct"""
    _fmt = ''
    _args = []

    def __init__(self, fmt, args):
        self._fmt = fmt
        self._args = args

    def extend(self, fmt, args):
        self._fmt += fmt
        self._args += args

    def pack(self):
        packer = struct.Struct(self._fmt)
        args = tuple([getattr(self, k) for k in self._args])
        return packer.pack(*args)

    def unpack(self, data):
        unpacker = struct.Struct(self._fmt)
        unpacked = unpacker.unpack_from(data)
        i = 0
        for k in self._args:
            setattr(self, k, unpacked[i])
            i += 1

    def __repr__(self):
        s = self.__class__.__name__ + ' contents:\n'
        for i in self._args:
            s += i + ' = ' + str(getattr(self, i, 'undefined')) + '\n'
        return s


class TaggedItem(_Blob):
    """Configuration item bare (without any value)"""
    def __init__(self, bytes_to_follow=0):
        _Blob.__init__(self, b'<H', ['tag'])
        self.header_length = struct.calcsize(self._fmt)
        self.length = self.header_length + bytes_to_follow


class ConfigItem(TaggedItem):
    """A configuration item which should be subclassed"""
    def __init__(self, fmt=b'', args=[], **kwargs):
        TaggedItem.__init__(self, struct.calcsize(b'<' + fmt))
        self.extend(fmt, args)
        for k in kwargs.keys():
            setattr(self, k, kwargs[k])


class ConfigItem_GPS_LogPositionEnable(ConfigItem):
    tag = 0x0000
    path = 'gps'
    params = ['logPositionEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_GPS_LogTTFFEnable(ConfigItem):
    tag = 0x0001
    path = 'gps'
    params = ['logTTFFEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_GPS_Mode(ConfigItem):
    tag = 0x0002
    path = 'gps'
    params = ['mode']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        mode = self.mode
        if self.mode == 'SWITCH_TRIGGERED':
            self.mode = 0
        elif self.mode == 'SCHEDULED':
            self.mode = 1
        elif self.mode == 'HYBRID':
            self.mode = 2
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.mode = mode
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.mode == 0):
            self.mode = 'SWITCH_TRIGGERED'
        elif (self.mode == 1):
            self.mode = 'SCHEDULED'
        elif (self.mode == 2):
            self.mode = 'HYBRID'
        else:
            self.mode = 'UNKNOWN'


class ConfigItem_GPS_ScheduledAquisitionInterval(ConfigItem):
    tag = 0x0003
    path = 'gps'
    params = ['scheduledAquisitionInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_GPS_MaximumAquisitionTime(ConfigItem):
    tag = 0x0004
    path = 'gps'
    params = ['maximumAquisitionTime']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_GPS_ScheduledAquisitionNoFixTimeout(ConfigItem):
    tag = 0x0005
    path = 'gps'
    params = ['scheduledAquisitionNoFixTimeout']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_GPS_LastKnownPosition(ConfigItem):
    tag = 0x0006
    path = 'gps.lastKnownPosition'
    params = ['iTOW', 'longitude', 'latitude', 'height', 'accuracyHorizontal', 'accuracyVertical', 'year', 'month', 'day', 'hours', 'minutes', 'seconds']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'IlllIIH5B', self.params, **kwargs)

    def pack(self):
        longitude = self.longitude
        latitude = self.latitude
        height = self.height
        accuracyHorizontal = self.accuracyHorizontal
        accuracyVertical = self.accuracyVertical
        self.longitude = int(self.longitude / 1E-7)
        self.latitude = int(self.latitude / 1E-7)
        self.height = int(self.height * 1000.0)
        self.accuracyHorizontal = int(self.accuracyHorizontal * 1000.0)
        self.accuracyVertical = int(self.accuracyVertical * 1000.0)
        data = ConfigItem.pack(self)
        self.longitude = longitude
        self.latitude = latitude
        self.height = height
        self.accuracyHorizontal = accuracyHorizontal
        self.accuracyVertical = accuracyVertical
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        self.longitude = 1E-7 * self.longitude
        self.latitude = 1E-7 * self.latitude
        self.height = self.height / 1000.0
        self.accuracyHorizontal = self.accuracyHorizontal / 1000.0
        self.accuracyVertical = self.accuracyVertical / 1000.0


class ConfigItem_GPS_VeryFirstFixHoldTime(ConfigItem):
    tag = 0x0007
    path = 'gps'
    params = ['veryFirstFixHoldTime']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_GPS_LogDebugEnable(ConfigItem):
    tag = 0x0008
    path = 'gps'
    params = ['logDebugEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_saltwaterSwitch_LogEnable(ConfigItem):
    tag = 0x0800
    path = 'saltwaterSwitch'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_saltwaterSwitch_HysteresisPeriod(ConfigItem):
    tag = 0x0801
    path = 'saltwaterSwitch'
    params = ['hysteresisPeriod']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_RTC_SyncToGPSEnable(ConfigItem):
    tag = 0x0600
    path = 'rtc'
    params = ['syncToGPSEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_RTC_CurrentDateTime(ConfigItem):
    tag = 0x0601
    path = 'rtc'
    params = ['day', 'month', 'year', 'hours', 'minutes', 'seconds']
    json_params = ['dateTime']

    def __init__(self, **kwargs):
        if 'dateTime' in kwargs:
            self.dateTime = kwargs['dateTime']
            del kwargs['dateTime']
        else:
            self.dateTime = None
        ConfigItem.__init__(self, b'BBHBBB', self.params, **kwargs)

    def pack(self):
        if self.dateTime:
            d = dateutil.parser.parse(self.dateTime)
            self.day = d.day
            self.month = d.month
            self.year = d.year
            self.hours = d.hour
            self.minutes = d.minute
            self.seconds = d.second
        return ConfigItem.pack(self)

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        d = datetime.datetime(day=self.day,
                     month=self.month,
                     year=self.year,
                     hour=self.hours,
                     minute=self.minutes,
                     second=self.seconds)
        self.dateTime = d.ctime()


class ConfigItem_Logging_Enable(ConfigItem):
    tag = 0x0100
    path = 'logging'
    params = ['enable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Logging_FileSize(ConfigItem):
    tag = 0x0101
    path = 'logging'
    params = ['fileSize']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'L', self.params, **kwargs)


class ConfigItem_Logging_FileType(ConfigItem):
    tag = 0x0102
    path = 'logging'
    params = ['fileType']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        fileType = self.fileType
        if self.fileType == 'LINEAR':
            self.fileType = 0
        elif self.fileType == 'CIRCULAR':
            self.fileType = 1
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.fileType = fileType
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.fileType == 0):
            self.fileType = 'LINEAR'
        elif (self.fileType == 1):
            self.fileType = 'CIRCULAR'
        else:
            self.fileType = 'UNKNOWN'


class ConfigItem_Logging_GroupSensorReadingsEnable(ConfigItem):
    tag = 0x0103
    path = 'logging'
    params = ['groupSensorReadingsEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Logging_StartEndSyncEnable(ConfigItem):
    tag = 0x0104
    path = 'logging'
    params = ['startEndSyncEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Logging_DateTimeStampEnable(ConfigItem):
    tag = 0x0105
    path = 'logging'
    params = ['dateTimeStampEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Logging_HighResolutionTimerEnable(ConfigItem):
    tag = 0x0106
    path = 'logging'
    params = ['highResolutionTimerEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_AXL_LogEnable(ConfigItem):
    tag = 0x0200
    path = 'accelerometer'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_AXL_Config(ConfigItem):
    tag = 0x0201
    path = 'accelerometer'
    params = ['config']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)


class ConfigItem_AXL_HighThreshold(ConfigItem):
    tag = 0x0202
    path = 'accelerometer'
    params = ['highThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_AXL_SampleRate(ConfigItem):
    tag = 0x0203
    path = 'accelerometer'
    params = ['sampleRate']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_AXL_Mode(ConfigItem):
    tag = 0x0204
    path = 'accelerometer'
    params = ['mode']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        mode = self.mode
        if self.mode == 'PERIODIC':
            self.mode = 0
        elif self.mode == 'TRIGGER_ABOVE':
            self.mode = 3
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.mode = mode
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.mode == 0):
            self.mode = 'PERIODIC'
        elif (self.mode == 3):
            self.mode = 'TRIGGER_ABOVE'
        else:
            self.mode = 'UNKNOWN'


class ConfigItem_AXL_ScheduledAquisitionInterval(ConfigItem):
    tag = 0x0205
    path = 'accelerometer'
    params = ['scheduledAquisitionInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_AXL_MaximumAquisitionTime(ConfigItem):
    tag = 0x0206
    path = 'accelerometer'
    params = ['maximumAquisitionTime']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_PressureSensor_LogEnable(ConfigItem):
    tag = 0x0300
    path = 'pressureSensor'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_PressureSensor_SampleRate(ConfigItem):
    tag = 0x0301
    path = 'pressureSensor'
    params = ['sampleRate']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_PressureSensor_LowThreshold(ConfigItem):
    tag = 0x0302
    path = 'pressureSensor'
    params = ['lowThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_PressureSensor_HighThreshold(ConfigItem):
    tag = 0x0303
    path = 'pressureSensor'
    params = ['highThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_PressureSensor_Mode(ConfigItem):
    tag = 0x0304
    path = 'pressureSensor'
    params = ['mode']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        mode = self.mode
        if self.mode == 'PERIODIC':
            self.mode = 0
        elif self.mode == 'TRIGGER_BELOW':
            self.mode = 1
        elif self.mode == 'TRIGGER_BETWEEN':
            self.mode = 2
        elif self.mode == 'TRIGGER_ABOVE':
            self.mode = 3
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.mode = mode
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.mode == 0):
            self.mode = 'PERIODIC'
        elif (self.mode == 1):
            self.mode = 'TRIGGER_BELOW'
        elif (self.mode == 2):
            self.mode = 'TRIGGER_BETWEEN'
        elif (self.mode == 3):
            self.mode = 'TRIGGER_ABOVE'
        else:
            self.mode = 'UNKNOWN'


class ConfigItem_PressureSensor_ScheduledAquisitionInterval(ConfigItem):
    tag = 0x0305
    path = 'pressureSensor'
    params = ['scheduledAquisitionInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_PressureSensor_MaximumAquisitionTime(ConfigItem):
    tag = 0x0306
    path = 'pressureSensor'
    params = ['maximumAquisitionTime']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_TempSensor_LogEnable(ConfigItem):
    tag = 0x0700
    path = 'temperateSensor'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_TempSensor_SampleRate(ConfigItem):
    tag = 0x0701
    path = 'temperateSensor'
    params = ['sampleRate']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_TempSensor_LowThreshold(ConfigItem):
    tag = 0x0702
    path = 'temperateSensor'
    params = ['lowThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_TempSensor_HighThreshold(ConfigItem):
    tag = 0x0703
    path = 'temperateSensor'
    params = ['highThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_TempSensor_Mode(ConfigItem):
    tag = 0x0704
    path = 'temperateSensor'
    params = ['mode']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        mode = self.mode
        if self.mode == 'PERIODIC':
            self.mode = 0
        elif self.mode == 'TRIGGER_BELOW':
            self.mode = 1
        elif self.mode == 'TRIGGER_BETWEEN':
            self.mode = 2
        elif self.mode == 'TRIGGER_ABOVE':
            self.mode = 3
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.mode = mode
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.mode == 0):
            self.mode = 'PERIODIC'
        elif (self.mode == 1):
            self.mode = 'TRIGGER_BELOW'
        elif (self.mode == 2):
            self.mode = 'TRIGGER_BETWEEN'
        elif (self.mode == 3):
            self.mode = 'TRIGGER_ABOVE'
        else:
            self.mode = 'UNKNOWN'


class ConfigItem_BLE_DeviceAddress(ConfigItem):
    tag = 0x0500
    path = 'bluetooth'
    params = ['deviceAddress']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'6s', self.params, **kwargs)

    def pack(self):
        old = self.deviceAddress
        self.deviceAddress = binascii.unhexlify(self.deviceAddress.replace(':', ''))[::-1]
        if (ord(self.deviceAddress[5]) & 0b11000000) != 0b11000000:
            raise ExceptionConfigBLEDeviceAddressTopTwoBitsNotSet # Enforce top 2 bits being set
        data = ConfigItem.pack(self)
        self.deviceAddress = old
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        device_id = binascii.hexlify(self.deviceAddress[::-1])
        new_id = b''
        for i in range(len(device_id)):
            new_id = new_id + device_id[i]
            if i & 1 and i != (len(device_id)-1):
                new_id = new_id + ':'
        self.deviceAddress = new_id


class ConfigItem_BLE_TriggerControl(ConfigItem):
    tag = 0x0501
    path = 'bluetooth'
    params = ['triggerControl']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)


class ConfigItem_BLE_ScheduledInterval(ConfigItem):
    tag = 0x0502
    path = 'bluetooth'
    params = ['scheduledInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_BLE_ScheduledDuration(ConfigItem):
    tag = 0x0503
    path = 'bluetooth'
    params = ['scheduledDuration']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_BLE_AdvertisingInterval(ConfigItem):
    tag = 0x0504
    path = 'bluetooth'
    params = ['advertisingInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_BLE_ConnectionInterval(ConfigItem):
    tag = 0x0505
    path = 'bluetooth'
    params = ['connectionInterval']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_BLE_InactivityTimeout(ConfigItem):
    tag = 0x0506
    path = 'bluetooth'
    params = ['inactivityTimeout']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'H', self.params, **kwargs)


class ConfigItem_BLE_PHYMode(ConfigItem):
    tag = 0x0507
    path = 'bluetooth'
    params = ['phyMode']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)

    def pack(self):
        phyMode = self.phyMode
        if self.phyMode == '1_MBPS':
            self.phyMode = 0
        elif self.phyMode == '2_MBPS':
            self.phyMode = 1
        else:
            raise ExceptionConfigInvalidValue
        data = ConfigItem.pack(self)
        self.phyMode = phyMode
        return data

    def unpack(self, data):
        ConfigItem.unpack(self, data)
        if (self.phyMode == 0):
            self.phyMode = '1_MBPS'
        elif (self.phyMode == 1):
            self.phyMode = '2_MBPS'
        else:
            self.phyMode = 'UNKNOWN'


class ConfigItem_BLE_LogEnable(ConfigItem):
    tag = 0x0508
    path = 'bluetooth'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Battery_LogEnable(ConfigItem):
    tag = 0x0900
    path = 'battery'
    params = ['logEnable']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'?', self.params, **kwargs)


class ConfigItem_Battery_LowThreshold(ConfigItem):
    tag = 0x0901
    path = 'battery'
    params = ['lowThreshold']
    json_params = params

    def __init__(self, **kwargs):
        ConfigItem.__init__(self, b'B', self.params, **kwargs)
