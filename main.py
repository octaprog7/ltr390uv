import sys
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
import ltr390uv
import time


def show_header(caption: str, symbol: str = "*", count: int = 40):
    print(count * symbol[0])
    print(caption)
    print(count * symbol[0])


if __name__ == '__main__':
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # on Raspberry Pi Pico
    adapter = I2cAdapter(i2c)
    als = ltr390uv.LTR390UV(adapter=adapter)
    _id = als.get_id()
    print(f"Part number id: {_id[0]}; Revision id: {_id[1]};")
    als.soft_reset()
    print("Software reset successfully!")
    #
    als.start_measurement(uv_mode=False)
    cct_ms = als.get_conversion_cycle_time()
    # настройки
    print(f"uv_mode: {als.uv_mode}")
    print(f"meas_rate: {als.meas_rate}")
    print(f"resolution: {als.resolution}")
    print(f"gain: {als.gain}")
    # состояние
    status = als.get_status()
    print(status)

    show_header(f"ALS mode. LUX out! uv_mode: {als.uv_mode}")

    for i in range(1000):
        time.sleep_ms(cct_ms)
        print(f"lux: {als.get_illumination(raw=False)}")
        
    als.start_measurement(uv_mode=True)
    cct_ms = als.get_conversion_cycle_time()

    show_header(f"UV mode. RAW only out! uv_mode: {als.uv_mode}")

    cnt = 0
    for raw in als:
        time.sleep_ms(cct_ms)
        print(f"raw: {raw}")
        cnt += 1
        if cnt > 3_000:
            sys.exit(0)
