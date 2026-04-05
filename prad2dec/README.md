# prad2dec

Static library for reading CODA EVIO data and decoding detector electronics.

## Components

- **EvChannel** — EVIO file reader with bank tree scanning and multi-format decoding
- **EtChannel** — Subclass for live ET system reading (`-DWITH_ET=ON`)
- **Fadc250Decoder** — FADC250 composite waveform decoding
- **Fadc250RawDecoder** — FADC250 raw hardware format decoding
- **Adc1881mDecoder** — ADC1881M decoding (original PRad)
- **SspDecoder** — SSP/MPD fiber data for GEM readout
- **WaveAnalyzer** — Pedestal subtraction, peak search, integration

## Usage

```cpp
evc::EvChannel ch;
ch.Open("data.evio");
fdec::EventData event;

while (ch.Read() == evc::status::success) {
    if (!ch.Scan()) continue;
    for (int i = 0; i < ch.GetNEvents(); ++i) {
        ch.DecodeEvent(i, event);
        // event.info  — timestamp, trigger type/bits, run number
        // event.rocs[r].slots[s].channels[c].samples[]
    }
}
```

## Dependencies

- [evio](https://github.com/JeffersonLab/evio) (evio-6.0) — required
- [et](https://github.com/JeffersonLab/et) — optional, for EtChannel

Both are resolved from the Hall-B CODA installation by default; if not found, CMake fetches from GitHub. Override with `-DEVIO_SOURCE=fetch` / `-DET_SOURCE=fetch`.
