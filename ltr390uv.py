"""Модуль управления LTR-390UV-01, это датчик внешней освещенности и датчик ультрафиолетового света (UVS)"""
# micropython
# mail: goctaprog@gmail.com
# MIT license
# import struct

from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import BaseSensorEx, Iterator, get_error_str, check_value
# from machine import Pin  # I2C, SPI,
# import micropython
from sensor_pack_2.bitfield import bit_field_info
# from micropython import const
from collections import namedtuple
from sensor_pack_2.bitfield import BitFields
from sensor_pack_2.regmod import RegistryRO, RegistryRW

# power_on - (1) Power on event and all interrupt threshold settings in the registers
#               have been reset to power on default states and should be
#               examined if necessary.
# int_status - (1) Interrupt is triggered and will be cleared after read
# data_status - (1) UVS/ALS data is new data (Data has not been read and will be
#               cleared after read)
sensor_status = namedtuple("sensor_status", "power_on int_status data_status")

'''
Address R/W     Register Name       Description                                                 Reset Value
0x00    R/W     MAIN_CTRL           ALS/UVS operation mode control, SW reset                    0x00
0x04    R/W     ALS_UVS_MEAS_RATE   ALS/UVS measurement rate and resolution in Active Mode      0x22
0x05    R/W     ALS_UVS_GAIN        ALS/UVS analog Gain range                                   0x01
0x06    R       PART_ID             Part number ID and revision ID                              0xB2
0x07    R       MAIN_STATUS         Power-On status, Interrupt status, Data status              0x20
0x0D    R       ALS_DATA_0          ALS ADC measurement data, LSB                               0x0
0x0E    R       ALS_DATA_1          ALS ADC measurement data                                    0x0
0x0F    R       ALS_DATA_2          ALS ADC measurement data, MSB                               0x0
0x10    R       UVS_DATA_0          UVS ADC measurement data, LSB                               0x0
0x11    R       UVS_DATA_1          UVS ADC measurement data                                    0x0
0x12    R       UVS_DATA_2          UVS ADC measurement data, MSB                               0x0
'''

# регистр режимов работы датчика
_main_control_reg = (bit_field_info(name='soft_reset', position=range(4, 5), valid_values=None),
                     bit_field_info(name='UVS_mode', position=range(3, 4), valid_values=None),
                     bit_field_info(name='ALS_UVS_enable', position=range(1, 2), valid_values=None))

# регистр периода/частоты измерений
_meas_rate_reg = (bit_field_info(name='resolution', position=range(4, 7), valid_values=range(6)),
                  bit_field_info(name='meas_rate', position=range(3), valid_values=range(6)))

# регистр усиления
_gain_reg = (bit_field_info(name='gain', position=range(3), valid_values=range(5)),)

# Part ID and Revision ID
_id_reg = (bit_field_info(name='part_id', position=range(4, 8), valid_values=None),
           bit_field_info(name='rev_id', position=range(4), valid_values=None))

# регистр состояния
_main_status_reg = (bit_field_info(name='power_on', position=range(5, 6), valid_values=None),
                    bit_field_info(name='int_status', position=range(4, 5), valid_values=None),
                    bit_field_info(name='data_status', position=range(3, 4), valid_values=None))


class LTR390UV(BaseSensorEx, Iterator):
    """Класс, управляющий ALS&UV датчиком LTR-390UV"""

    def __init__(self, adapter: bus_service.BusAdapter, address=0x53):
        """i2c - объект класса I2C; address - адрес датчика на шине"""
        super().__init__(adapter, address, False)
        self._id_reg = RegistryRO(device=self, address=0x06, fields=BitFields(_id_reg), byte_len=None)
        self._status_reg = RegistryRO(device=self, address=0x07, fields=BitFields(_main_status_reg), byte_len=None)
        self._meas_rate_reg = RegistryRW(device=self, address=0x04, fields=BitFields(_meas_rate_reg), byte_len=None)
        self._gain_reg = RegistryRW(device=self, address=0x05, fields=BitFields(_gain_reg), byte_len=None)
        self.ctrl_reg = RegistryRW(device=self, address=0x00, fields=BitFields(_main_control_reg), byte_len=None)
        # буфер на 3 байта
        self._buf_3 = bytearray((0 for _ in range(3)))
        # настройки, обновляются методом xxx!
        self._uv_sens = 2300    # чувствительность датчика в UV диапазоне для разрешения отсчета в 18..20 бит!
        #
        self._uv_mode = None
        self._resolution = None
        self._meas_rate = None
        self._gain = None
        self._enabled = None

    def get_status(self) -> sensor_status:
        """Возвращает состояние датчика"""
        _reg = self._status_reg
        _reg.read()
        return sensor_status(power_on=_reg['power_on'], int_status=_reg['int_status'], data_status=_reg['data_status'])

    def get_conversion_cycle_time(self) -> int:
        """возвращает время преобразования датчика в [мc]. Первым должен быть вызван метод start_measurement!"""
        return LTR390UV._meas_rate_to_ms(self.meas_rate)

    @staticmethod
    def _meas_rate_to_resolution(meas_rate: int) -> int:
        """возвращает разрешение в условных единицах, соответствующее meas_rate.
        Значение разрешения 13 Bit не используется!!!
        meas_rate/ms       resolution/bit in sample/conversion time ms
        0/25               4/16/25
        1/50               3/17/50
        2/100              2/18/100
        3/200              1/19/200
        4/500              0/20/400
        5/1000             0/20/400
        """
        vr = range(6)
        check_value(meas_rate, vr, get_error_str("meas_rate", meas_rate, vr))
        # 0 - 20 bit
        _resol = 4, 3, 2, 1, 0, 0
        return _resol[meas_rate]

    @staticmethod
    def _meas_rate_to_ms(meas_rate: int) -> int:
        """возвращает частоту преобразования(условные единицы) во время преобразования(conversion time) в мс"""
        vr = range(6)
        check_value(meas_rate, vr, get_error_str("meas_rate", meas_rate, vr))
        _conv_time = 25, 50, 100, 200, 500, 1000
        return _conv_time[meas_rate]

    def get_id(self) -> tuple:
        """Возвращает Part_ID, Revision_ID"""
        _reg = self._id_reg
        _reg.read()
        return _reg['part_id'], _reg['rev_id']

    def soft_reset(self):
        """Производит програмный сброс датчика"""
        _reg = self.ctrl_reg
        _reg['soft_reset'] = 1
        _reg.write()

    @property
    def gain(self) -> [int, None]:
        """Возвращает усиление в условных единицах от 0 до 4 включительно"""
        return self._gain

    @property
    def meas_rate(self) -> [int, None]:
        """Возвращает период обновления данных датчиком в условных единицах от 0 до 6 включительно"""
        return self._meas_rate

    @property
    def resolution(self) -> [int, None]:
        """Возвращает кол-во бит в отсчете в условных единицах от 0 до 5 включительно.
        resolution вычисляется автоматически по значению поля meas_rate метода start_measurement"""
        return self._resolution

    @property
    def uv_mode(self) -> [bool, None]:
        return self._uv_mode

    @property
    def in_standby(self) -> bool:
        """Если возвратит Истина, то устройство находится в режиме standby"""
        return not self._enabled

    @property
    def uv_sensitivity(self) -> int:
        """Возвращает чувствительность датчика в UV диапазоне в условных единицах"""
        return self._uv_sens

    @uv_sensitivity.setter
    def uv_sensitivity(self, value: int):
        """Устанавливает чувствительность датчика в UV диапазоне в условных единицах"""
        rng = range(1, 10_000)
        check_value(value, rng, get_error_str("UV sensitivity", value, rng))
        self._uv_sens = value

    def set_active(self, value: bool = True):
        """Если value в Истина, то датчик в режиме работа, иначе в режиме ожидания/standby"""
        _reg = self.ctrl_reg
        _reg.read()
        _reg['ALS_UVS_enable'] = value
        _reg.write()
        #
        _reg.read()
        self._enabled = _reg['ALS_UVS_enable']

    def start_measurement(self, uv_mode: bool, meas_rate: int = 1, gain: int = 3, enable: bool = True):
        """Производит настройку параметров измерения.
        Если uv_mode в Истина, то датчик измеряет освещенность в УФ диапазоне, иначе в видимом диапазоне!
        meas_rate - период обновления датчиком информации, от 0 до 5 включительно. 0 - 25 мс, 5 - 2000 мс.
        resolution - вычисляется по meas_rate! Значение разрешения 13 Bit не используется!!!
        gain - коэффициент усиления. от 0 до 4 включительно. 0 - 1, 4 - 18"""
        _reg = self._meas_rate_reg
        _reg.read()
        _reg['meas_rate'] = meas_rate
        _reg['resolution'] = LTR390UV._meas_rate_to_resolution(meas_rate)
        _reg.write()    # запись в регистр meas_rate
        #
        _reg = self._gain_reg
        _reg.read()
        _reg['gain'] = gain
        _reg.write()    # запись в регистр gain
        #
        _reg = self.ctrl_reg
        _reg.read()
        _reg['ALS_UVS_enable'] = enable
        _reg['UVS_mode'] = uv_mode
        _reg.write()    # запись в регистр управления
        # читаю настройки из датчика
        _reg = self._meas_rate_reg
        _reg.read()     # чтение регистра meas_rate
        self._meas_rate = _reg['meas_rate']
        self._resolution = _reg['resolution']
        #
        _reg = self._gain_reg
        _reg.read()     # чтение регистра gain
        self._gain = _reg['gain']
        #
        _reg = self.ctrl_reg
        _reg.read()     # чтение регистра управления
        self._uv_mode = _reg['UVS_mode']
        self._enabled = _reg['ALS_UVS_enable']

    def get_illumination(self, raw: bool = True, w_fac: float = 1.0) -> [int, float]:
        """Возвращает освещенность в 'сырых' единицах если raw в Истина, или в люксах, если raw в Ложь"""
        addr = 0x10 if self.uv_mode else 0x0D
        buf = self._buf_3
        self.read_buf_from_mem(addr, buf, 1)
        val = buf[0] + 256 * buf[1] + 65536 * buf[2]
        # для режима UV значения в UVI недоступны. Производитель привел формулу, которая, по моему мнению, неверна!
        # поэтому, при переключении в uv_mode вы будете получать от датчика только 'сырые' значения!
        if raw or self.uv_mode:
            return val
        _gain = 1, 3, 6, 9, 18  # для пересчета gain
        x = 0.25 * 2 ** self.resolution
        _tmp = _gain[self.gain] * x
        # lux. als
        return 0.6 * w_fac * val / _tmp   # lux

    # Iterator
    def __next__(self) -> [float, int, None]:
        """Часть протокола итератора"""
        if self.uv_mode is None:
            return None
        return self.get_illumination(raw=False)
