import struct
import logging
import inspect
import sys

logger = logging.getLogger(__name__)


class ExceptionMessageInvalidValue(Exception):
    pass


def str_error(error_code):
    errors = [ 'CMD_NO_ERROR',
      'CMD_ERROR_FILE_NOT_FOUND',
      'CMD_ERROR_FILE_ALREADY_EXISTS',
      'CMD_ERROR_INVALID_CONFIG_TAG',
      'CMD_ERROR_GPS_COMMS',
      'CMD_ERROR_TIMEOUT',
      'CMD_ERROR_CONFIG_PROTECTED',
      'CMD_ERROR_CONFIG_TAG_NOT_SET',
      'CMD_ERROR_BRIDGING_DISABLED',
      'CMD_ERROR_DATA_OVERSIZE',
      'CMD_ERROR_INVALID_PARAMETER',
      'CMD_ERROR_INVALID_FW_IMAGE_TYPE',
      'CMD_ERROR_IMAGE_CRC_MISMATCH',
      'CMD_ERROR_FILE_INCOMPATIBLE' ]
    if error_code >= len(errors):
        return 'ERROR_UNKNOWN'
    return errors[error_code]


def decode(data):

    """Attempt to decode a message from an input data buffer"""
    hdr = ConfigMessageHeader()
    if (len(data) < hdr.header_length):
        return (None, data)

    # If there's no sync byte return
    if ConfigMessageHeader.sync not in data:
        return (None, data)

    # Find location of first sync byte and decode from that position
    pos = data.index(ConfigMessageHeader.sync)

    # Unpack message header at SYNC position
    hdr.unpack(data[pos:])

    msg = None
    unused_cls = [ ConfigMessage, ConfigMessageHeader ]
    for i in inspect.getmembers(sys.modules[__name__], inspect.isclass):
        cls = i[1]
        if issubclass(cls, ConfigMessageHeader) and cls not in unused_cls and \
            hdr.cmd == cls.cmd:
            msg = cls()
            break

    if (msg):
        msg.unpack(data)
        # Advance buffer past this message
        data = data[(pos + msg.length):]

    return (msg, data)


def convert_to_dict(obj):
    d = {}
    ignore = ['error_code', 'cmd', 'sync']
    for i in obj._args:
        if i not in ignore:
            d[i] = getattr(obj, i)
    return d


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


class ConfigMessageHeader(_Blob):
    """Configuration message header"""
    sync = 0x7E

    def __init__(self, bytes_to_follow=0):
        _Blob.__init__(self, b'<BB', ['sync', 'cmd'])
        self.header_length = struct.calcsize(self._fmt)
        self.length = self.header_length + bytes_to_follow


class ConfigMessage(ConfigMessageHeader):
    """A configuration message which should be subclassed"""
    def __init__(self, fmt=b'', args=[], **kwargs):
        ConfigMessageHeader.__init__(self, struct.calcsize(fmt))
        self.extend(fmt, args)
        for k in kwargs.keys():
            setattr(self, k, kwargs[k])


class GenericResponse(ConfigMessage):
    """A generic response message which should be subclassed"""
    cmd = 0
    name = 'GENERIC_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B', ['error_code'], **kwargs)


class ConfigMessage_CFG_READ_REQ(ConfigMessage):

    cmd = 1
    name = 'CFG_READ_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'H', ['cfg_tag'], **kwargs)


class ConfigMessage_CFG_READ_RESP(ConfigMessage):

    cmd = 8
    name = 'CFG_READ_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BI', ['error_code', 'length'], **kwargs)


class ConfigMessage_CFG_WRITE_CNF(ConfigMessage):

    cmd = 9
    name = 'CFG_WRITE_CNF'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B', ['error_code'], **kwargs)


class ConfigMessage_CFG_WRITE_REQ(ConfigMessage):

    cmd = 2
    name = 'CFG_WRITE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'I', ['length'], **kwargs)


class ConfigMessage_CFG_SAVE_REQ(ConfigMessageHeader):

    cmd = 3
    name = 'CFG_SAVE_REQ'


class ConfigMessage_CFG_RESTORE_REQ(ConfigMessageHeader):

    cmd = 4
    name = 'CFG_RESTORE_REQ'


class ConfigMessage_CFG_ERASE_REQ(ConfigMessage):

    cmd = 5
    name = 'CFG_ERASE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'H', ['cfg_tag'], **kwargs)


class ConfigMessage_CFG_PROTECT_REQ(ConfigMessageHeader):

    cmd = 6
    name = 'CFG_PROTECT_REQ'


class ConfigMessage_CFG_UNPROTECT_REQ(ConfigMessageHeader):

    cmd = 7
    name = 'CFG_UNPROTECT_REQ'


class ConfigMessage_GPS_WRITE_REQ(ConfigMessage):

    cmd = 10
    name = 'GPS_WRITE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'I', ['length'], **kwargs)


class ConfigMessage_GPS_READ_REQ(ConfigMessage):

    cmd = 11
    name = 'GPS_READ_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'I', ['length'], **kwargs)


class ConfigMessage_GPS_READ_RESP(ConfigMessage):

    cmd = 12
    name = 'GPS_READ_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BI', ['error_code', 'length'], **kwargs)


class ConfigMessage_GPS_CONFIG_REQ(ConfigMessage):

    cmd = 13
    name = 'GPS_CONFIG_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'?', ['enable'], **kwargs)


class ConfigMessage_BLE_CONFIG_REQ(ConfigMessage):

    cmd = 14
    name = 'BLE_CONFIG_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'?', ['enable'], **kwargs)


class ConfigMessage_BLE_WRITE_REQ(ConfigMessage):

    cmd = 15
    name = 'BLE_WRITE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BH', ['address', 'length'], **kwargs)


class ConfigMessage_BLE_READ_REQ(ConfigMessage):

    cmd = 16
    name = 'BLE_READ_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BH', ['address', 'length'], **kwargs)


class ConfigMessage_STATUS_REQ(ConfigMessageHeader):

    cmd = 17
    name = 'STATUS_REQ'


class ConfigMessage_STATUS_RESP(ConfigMessage):

    cmd = 18
    name = 'STATUS_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BIIIQBI???', ['error_code', 'fw_version', 'ble_fw_version', 'cfg_version', 'device_id', 'charge_level', 'log_file_size', 'pressure_enabled', 'temp_enabled', 'accel_enabled'], **kwargs)


class ConfigMessage_FW_SEND_IMAGE_REQ(ConfigMessage):

    cmd = 19
    name = 'FW_SEND_IMAGE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BII', ['image_type', 'length', 'crc',], **kwargs)

    def pack(self):
        image_type = self.image_type
        if self.image_type == 'STM32':
            self.image_type = 1
        elif self.image_type == 'BLE':
            self.image_type = 2
        else:
            raise ExceptionMessageInvalidValue
        data = ConfigMessage.pack(self)
        self.image_type = image_type
        return data

    def unpack(self, data):
        ConfigMessage.unpack(self, data)
        if (self.image_type == 1):
            self.image_type = 'STM32'
        elif (self.image_type == 2):
            self.image_type = 'BLE'
        else:
            self.image_type = 'UNKNOWN'


class ConfigMessage_FW_SEND_IMAGE_COMPLETE_CNF(GenericResponse):

    cmd = 20
    name = 'FW_SEND_IMAGE_COMPLETE_CNF'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B', ['error_code'], **kwargs)


class ConfigMessage_FW_APPLY_IMAGE_REQ(ConfigMessage):

    cmd = 21
    name = 'FW_APPLY_IMAGE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B', ['image_type'], **kwargs)

    def pack(self):
        image_type = self.image_type
        if self.image_type == 'STM32':
            self.image_type = 1
        elif self.image_type == 'BLE':
            self.image_type = 2
        elif self.image_type == 'SOFTDEVICE':
            self.image_type = 3
        else:
            raise ExceptionMessageInvalidValue
        data = ConfigMessage.pack(self)
        self.image_type = image_type
        return data

    def unpack(self, data):
        ConfigMessage.unpack(self, data)
        if (self.image_type == 1):
            self.image_type = 'STM32'
        elif (self.image_type == 2):
            self.image_type = 'BLE'
        elif (self.image_type == 3):
            self.image_type = 'SOFTDEVICE'
        else:
            self.image_type = 'UNKNOWN'


class ConfigMessage_RESET_REQ(ConfigMessage):

    cmd = 22
    name = 'RESET_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B', ['reset_type'], **kwargs)

    def pack(self):
        reset_type = self.reset_type
        if self.reset_type == 'STM32':
            self.reset_type = 0
        elif self.reset_type == 'FLASH':
            self.reset_type = 1
        else:
            raise ExceptionMessageInvalidValue
        data = ConfigMessage.pack(self)
        self.reset_type = reset_type
        return data

    def unpack(self, data):
        ConfigMessage.unpack(self, data)
        if (self.reset_type == 0):
            self.reset_type = 'STM32'
        elif (self.reset_type == 1):
            self.reset_type = 'FLASH'
        else:
            self.reset_type = 'UNKNOWN'


class ConfigMessage_BATTERY_STATUS_REQ(ConfigMessageHeader):

    cmd = 23
    name = 'BATTERY_STATUS_REQ'


class ConfigMessage_BATTERY_STATUS_RESP(ConfigMessage):

    cmd = 24
    name = 'BATTERY_STATUS_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B?B', ['error_code', 'charging_ind', 'charging_level'], **kwargs)


class ConfigMessage_LOG_CREATE_REQ(ConfigMessage):

    cmd = 25
    name = 'LOG_CREATE_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'B?', ['mode', 'sync_enable'], **kwargs)

    def pack(self):
        mode = self.mode
        if self.mode == 'LINEAR':
            self.mode = 0
        elif self.mode == 'CIRCULAR':
            self.mode = 1
        else:
            raise ExceptionMessageInvalidValue
        data = ConfigMessage.pack(self)
        self.mode = mode
        return data

    def unpack(self, data):
        ConfigMessage.unpack(self, data)
        if (self.mode == 0):
            self.mode = 'LINEAR'
        elif (self.mode == 1):
            self.mode = 'CIRCULAR'
        else:
            self.mode = 'UNKNOWN'


class ConfigMessage_LOG_ERASE_REQ(ConfigMessageHeader):

    cmd = 26
    name = 'LOG_ERASE_REQ'


class ConfigMessage_LOG_READ_REQ(ConfigMessage):

    cmd = 27
    name = 'LOG_READ_REQ'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'II', ['start_offset', 'length'], **kwargs)


class ConfigMessage_LOG_READ_RESP(ConfigMessage):

    cmd = 28
    name = 'LOG_READ_RESP'

    def __init__(self, **kwargs):
        ConfigMessage.__init__(self, b'BI', ['error_code', 'length'], **kwargs)
