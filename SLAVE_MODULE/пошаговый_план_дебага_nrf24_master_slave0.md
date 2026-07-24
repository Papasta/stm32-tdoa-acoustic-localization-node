# Пошаговый план дебага радиоканала nRF24L01+ master ↔ slave0

## 1. Цель

Нужно постепенно уменьшить неизвестность и определить, где теряется timestamp-пакет:

1. slave0 не может управлять своим `NRF3` по SPI;
2. `NRF3_IRQ` slave0 удерживается в нуле из-за неочищенного `STATUS`;
3. slave0 не передаёт пакет после события;
4. master `NRF3` не находится в PRX или слушает не тот pipe;
5. адрес/канал/скорость/CRC не совпадают;
6. пакет приходит на master, но не считывается из RX FIFO;
7. группировка событий работает неправильно.

## 2. Проверка железа slave0: питание и линии nRF3

Проверить мультиметром/логическим анализатором:

| Сигнал | Ожидаемое состояние после старта |
|---|---|
| VCC nRF24L01+ | 3.3 В без просадок |
| GND | общий с STM32 |
| CSN | 1 в покое |
| CE | 0 в покое для PTX, короткий импульс при передаче |
| SCK/MOSI | активность при инициализации и передаче |
| MISO | отвечает, не висит постоянно 0 или 1 |
| IRQ | 1 после очистки `STATUS`; 0 только при `TX_DS` или `MAX_RT` |

Обязательно поставить рядом с nRF24L01+ конденсатор 10–47 мкФ и 0.1 мкФ. Если питание проседает при TX, модуль может зависать, держать IRQ в нуле или не получать ACK.

## 3. Проверка SPI slave0 → NRF3

В STM32CubeMonitor после старта смотреть:

```text
radio_nrf3_present
radio_nrf3_rf_ch_read
radio_nrf3_setup_aw_read
radio_nrf3_config
radio_nrf3_last_status
radio_nrf3_status_after_clear
radio_nrf3_fifo_status
radio_nrf3_irq_pin_level
```

Ожидаемые значения для slave0 `NRF3`:

```text
radio_nrf3_present = 1
radio_nrf3_rf_ch_read = NRF3_FREQ, сейчас 76
radio_nrf3_setup_aw_read = 3
radio_nrf3_config = 0x4E
radio_nrf3_status_after_clear & 0x70 = 0
radio_nrf3_irq_pin_level = 1
```

Если `radio_nrf3_present = 0`, проблема до радиопротокола: SPI3, CSN, питание, MISO/MOSI/SCK, неправильный модуль или конфликт пинов.

Если `radio_nrf3_irq_pin_level = 0`, а `radio_nrf3_status_after_clear & 0x70 = 0`, значит линия IRQ физически притянута к земле, перепутан пин, неверная подтяжка, пайка или конфликт GPIO.

Если `radio_nrf3_irq_pin_level = 0`, а `radio_nrf3_last_status` содержит `0x10`, `0x20` или `0x40`, значит флаг прерывания не очищается. В версии v5 добавлена принудительная очистка `STATUS`, `FLUSH_RX`, `FLUSH_TX` при инициализации.

## 4. Проверка генерации события на slave0

Вызвать 1–3 импульсных события и смотреть:

```text
recognized_event_id
radio_event_ready
radio_event_packet.timestamp_sample
radio_event_packet.event_id
radio_nrf3_tx_busy
radio_nrf3_tx_ok_count
radio_nrf3_max_rt_count
radio_nrf3_last_status
radio_nrf3_observe_tx
```

Ожидаемый сценарий при успешной передаче:

1. `recognized_event_id` увеличился;
2. `radio_event_packet.event_id` совпал с новым номером события;
3. `radio_nrf3_tx_busy` кратковременно стал 1;
4. после IRQ `radio_nrf3_tx_ok_count` увеличился;
5. `radio_nrf3_tx_busy` вернулся в 0;
6. `radio_nrf3_max_rt_count` не растёт.

Если `recognized_event_id` растёт, но `radio_nrf3_tx_busy` не меняется — проблема в вызове `Radio_ProcessMainLoop()` или `Radio_PublishEventTimestamp()`.

Если `radio_nrf3_max_rt_count` растёт — slave0 передаёт, но не получает ACK от master. Тогда проверять master `NRF3`: адрес pipe0, канал, CRC, скорость, CE, PRX-режим.

## 5. Проверка master NRF3 как приёмника timestamp

На мастере смотреть:

```text
g_radio_nrf3_present
g_radio_nrf3_last_status
g_radio_nrf3_last_fifo
g_radio_nrf3_irq_count
g_radio_nrf3_rx_dr_seen
g_radio_nrf3_poll_rx_dr_count
g_radio_timestamp_rx_count
g_radio_last_pipe
g_radio_last_event_id
g_radio_grouped_event_count
```

Ожидаемо после события slave0:

```text
g_radio_nrf3_present = 1
g_radio_timestamp_rx_count увеличился
g_radio_last_pipe = 0
g_radio_last_event_id = radio_event_packet.event_id на slave0
g_radio_grouped_event_count увеличился сразу при всех 4 метках или после timeout группы
```

Если на slave0 растёт `radio_nrf3_max_rt_count`, а на master `g_radio_timestamp_rx_count = 0`, значит master не слушает правильный адрес/канал или не находится в PRX.

## 6. Проверка совпадения адресов и каналов

Для slave0 должно быть:

### Slave0 NRF3 TX

```c
nrf3_master_pipe0_addr = {0x00, 0xCC, 0xBB, 0xAA, 0xE1};
NRF3_FREQ = 76;
```

### Master NRF3 PRX pipe0

```c
pipe0 = {0x00, 0xCC, 0xBB, 0xAA, 0xE1};
NRF3_FREQ = 76;
```

Проверить не только текст файла, но и то, что именно эти файлы реально собраны в CubeIDE: `Clean Project → Build Project`.

## 7. Проверка IRQ slave0 NRF3, если линия низкая сразу после включения

1. Отключить master, прошить slave0 v5.
2. После старта проверить `radio_nrf3_irq_pin_level`.
3. Если стало 1 — проблема была в неочищенном `STATUS`/FIFO, исправление сработало.
4. Если осталось 0:
   - прочитать `radio_nrf3_last_status`;
   - если `status & 0x70 != 0`, значит STATUS не очищается: проблема SPI-записи или CSN;
   - если `status & 0x70 == 0`, значит проблема железа IRQ-линии или неправильного GPIO.

## 8. Проверка SPI-записи в NRF3

Если регистры читаются, но не меняются:

1. Временно записать другой `RF_CH`, например 75, и прочитать назад.
2. Вернуть 76.
3. Если чтение не меняется, запись не проходит: CSN, MOSI, SCK, питание, режим SPI.

SPI nRF24L01+ должен быть Mode 0: CPOL=0, CPHA=0.

## 9. Проверка минимальной радиосвязи без детектора

Для изоляции детектора от радио можно временно вызвать отправку тестового пакета раз в 2 секунды после инициализации:

```c
radio_event_packet.timestamp_sample = get_time_us();
radio_event_packet.event_id++;
radio_event_ready = 1;
```

Если тестовые пакеты приходят на master, а события нет — проблема в детекторе или вызове публикации события.

Если тестовые пакеты не приходят — проблема в радиосвязи.

## 10. Проверка master → slave0 через NRF2

Параллельно проверить второй радиомодуль:

На slave0:

```text
radio_nrf2_present = 1
radio_nrf2_rf_ch_read = NRF2_FREQ, сейчас 40
radio_nrf2_irq_pin_level обычно 1
radio_nrf2_reset_count растёт после reset от master
radio_nrf2_sync_count растёт после sync от master
radio_nrf2_health_count растёт при health-check
```

Если `NRF2` работает, но `NRF3` нет, значит питание/пины/SPI3/адреса/режим именно первого радиомодуля.

## 11. Интерпретация типовых симптомов

| Симптом | Наиболее вероятная причина |
|---|---|
| `radio_nrf3_present = 0` | Нет SPI-связи с NRF3 |
| `radio_nrf3_irq_pin_level = 0`, `status & 0x70 != 0` | Не очищен/не очищается `STATUS` |
| `radio_nrf3_irq_pin_level = 0`, `status & 0x70 == 0` | IRQ физически замкнут/не тот GPIO/конфликт пина |
| `radio_nrf3_max_rt_count` растёт | Master не ACK-ает timestamp-пакет |
| `radio_nrf3_tx_ok_count` растёт, master не видит пакет | Master читает не тот radio/pipe, ошибка группировки или мониторинг не тех переменных |
| На master `last_pipe = 255` всегда | `RX_DR` ни разу не был обработан на NRF3 |
| На master `last_pipe = 7` | STATUS сообщает RX FIFO empty/ошибка чтения pipe |
| `g_radio_timestamp_rx_count` растёт, но сервер не получает | Проблема уже в группировке/W5500, а не в радиоканале slave → master |

## 12. Минимальная последовательность дебага

1. Проверить питание и CE/CSN/IRQ slave0 NRF3.
2. Проверить `radio_nrf3_present` и чтение регистров.
3. Проверить, что после инициализации `radio_nrf3_irq_pin_level = 1`.
4. Сгенерировать событие и проверить `radio_nrf3_tx_busy`.
5. Проверить `TX_DS` или `MAX_RT` на slave0.
6. Если `MAX_RT`, перейти к master NRF3: CE, CONFIG, RF_CH, pipe0 address.
7. Если `TX_DS`, но master не группирует событие, смотреть master RX FIFO/pipe/event_id.
8. Только после успешного `g_radio_timestamp_rx_count` переходить к W5500 и серверу.
