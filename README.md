# Portable Charger — USB-C Power Delivery PCB

A custom portable USB-C Power Delivery (PD) charger with a dedicated PD trigger circuit for negotiating output voltage. Powered from a single USB-C input and designed for compact, portable use.

## Features

- **USB-C Power Delivery** input with PD voltage negotiation trigger
- **Custom PD trigger circuit** to request a specific voltage from the source
- Compact KiCad PCB design optimized for portability
- Designed with **JLCPCB assembly** constraints in mind
- CAD enclosure files included

## Repository Structure

```
Portable_Charger/
├── Charger_Kicad/         # Main charger PCB KiCad project
├── USBC_PD_Trigger_Kicad/ # USB-C PD trigger board KiCad project
└── CAD/                   # Enclosure / mechanical CAD files
```

## How It Works

1. The **PD trigger board** negotiates a target voltage with a USB-C PD source using the CC lines
2. The negotiated voltage is passed to the **charger board** for regulation and output
3. The design runs entirely from a single USB-C cable — no barrel jacks or separate power supplies required

## Tools Used

- **KiCad** — Schematic capture and PCB layout
- **JLCPCB** — PCB fabrication and assembly
- **Fusion360 / SolidWorks** — Enclosure design

## Author

**Jackson Barber** — [github.com/jacksterb1234](https://github.com/jacksterb1234)
