---
layout: default
title: Hardware Setup Jetson
nav_order: 5
---

{% include gallery-styles.html %}
{% include gallery-scripts.html %}

# Hardware
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Wire harness

| Component                | Description                          | Quantity |
|--------------------------|--------------------------------------|----------|
| Yellow wire AWG 24 45 cm | CAN L wire base to D131              | 1        |
| Green wire AWG 24 45 cm  | CAN H wire base to D131              | 1        |
| Black wire AWG 20 30 cm  | GND wire base to D131                | 1        |
| Red wire AWG 20 30 cm    | 12V wire base to power switch        | 1        |
| Black wire AWG 24 15 cm  | GND wire D131 Jack to D131 CAN       | 1        |
| Red wire AWG 24 15 cm    | 12 wire power switch to D131 jack    | 1        |
| Quick connect            | Quick connect crimp for power switch | 2        |
| 2x2 ATX crimp housing    | D131 power jack crimp housing        | 1        |
| ATX crimp contacts       | D131 power jack crimp contacts       | 2        |
| AWG 24 Ferrules          | CAN Ferrules                         | 3        |
| Harwin 2x2 crimp housing | For CAN base                         | 1        |
| Harwin crimp contacts    | For CAN base                         | 4        |

| Tool                        | Description                 |
|-----------------------------|-----------------------------|
| Diagonal pliers             | To cut the cable            |
| Wire stripper               | To isolate the wire strands |
| Crimping tool Harwin        | Harwin wire crimper         |
| Crimping tool ATX           | ATX-style wire crimper      |
| Crimping tool Ferrules      | Ferrule crimper             |
| Crimping tool quick connect | Quick connect crimper       |

{% include gallery.html gallery_id="gallery_d131_wiring" gallery_data="gallery_d131_wiring" %}

## Robot

| Component              | Description                       | Quantity |
|------------------------|-----------------------------------|----------|
| DJI RoboMaster S1      | Fully assembled and tested robot  | 1        |
| M4 distance bolts 10mm | Distance bolts for base plate     | 1        |
| Wire harness           | Previously assembled wire harness | 1        |

| Tool             | Description           |
|------------------|-----------------------|
| Torx screwdriver | For the turret screws |

{% include gallery.html gallery_id="gallery_common_base" gallery_data="gallery_common_base" %}

### UI Unit Cable

First, we assemble the I2C connection cable. Then, we assemble the rest of the UI unit.

| Component             | Description                            | Quantity |
|-----------------------|----------------------------------------|----------|
| OLED display Cable    | I2C cable coming with the display unit | 1        |
| Harwin 2x1            | Crimp housing                          | 1        |
| Harwin 1x1            | Crimp housing                          | 2        |
| Harwin crimp contacts | Crimp contacts                         | 4        |

| Tool                 | Description                       |
|----------------------|-----------------------------------|
| Diagonal pliers      | To cut the cable                  |
| Wire stripper        | To isolate the wire strands       |
| Crimping tool        | Harwin wire crimper               |

{% include gallery.html gallery_id="gallery_common_ui_wiring" gallery_data="gallery_common_ui_wiring" %}

### UI Unit

| Component             | Description                       | Quantity |
|-----------------------|-----------------------------------|----------|
| UI front plate        | Lasercut front plate              | 1        |
| Power switch          | Power switch                      | 1        |
| UI Button             | Button to interact with UI        | 1        |
| OLED display          | UI display                        | 1        |
| Red wire 20cm         | Signal wire for UI button         | 1        |
| Red wire 4cm          | Wire for pullup                   | 1        |
| Black wire 20cm       | Gnd wire for UI button            | 1        |
| Shrink tubing 2cm     | Schrink tubing for wires          | 3        |
| Resistor 1k5          | Pullup resistor for UI button     | 1        |
| Harwin 2x1            | Crimp housing for UI button       | 1        |
| Harwin crimp contacts | Crimp contacts                    | 2        |
| M2 distance bolt      | Distance bolt to mount UI display | 2        |
| M2 screws 10mm        | screws to mount display on bolts  | 4        |

| Tool                 | Description                       |
|----------------------|-----------------------------------|
| Diagonal pliers      | To cut the cable                  |
| Wire stripper        | To isolate the wire strands       |
| Crimping tool        | Harwin wire crimper               |
| Phillips screwdriver | For small M2 screws               |
| Soldering Iron       | To solder wires                   |
| Heat gun             | To apply shrink tubing to wires   |

{% include gallery.html gallery_id="gallery_common_ui_plate" gallery_data="gallery_common_ui_plate" %}

### Computer

| Component              | Description                                            | Quantity |
|------------------------|--------------------------------------------------------|----------|
| D131 Carrier board box | Box containing the carrier board and mounting material | 1        |
| Jetson heat sink       | Heat sink and fan unit for the Jetson Orin             | 1        |
| Jetson Orin NX         | Jetson Orin NX computer                                | 1        |
| WiFi module            | Intel M2 WiFi module                                   | 1        |
| WiFi Antenna           | WiFi antenna for Intel module                          | 2        |
| SSD                    | SSD M2 256 GB                                          | 1        |

| Tool           | Description                            |
|----------------|----------------------------------------|
| Torx T6        | To screw in the Jetson mounting screws |
| Small Phillips | To unscrew fan                         |

{% include gallery.html gallery_id="gallery_d131_jetson" gallery_data="gallery_d131_jetson" %}

{% include gallery.html gallery_id="gallery_d131_computer" gallery_data="gallery_d131_computer" %}

### Robot

| Component                | Description                                     | Quantity |
|--------------------------|-------------------------------------------------|----------|
| Compute unit             | Assembled D131 carrier and Jetson               | 1        |
| Base plate               | Lasercut base plate                             | 1        |
| M3x17 F/F distance bolts | To mount the compute unit on the base plate     | 4        |
| M3x35 M/F distance bolts | To mount the UI unit on the compute unit        | 4        |
| M3x10 screws             | Screws to mount distance bolts                  | 4        |
| M4 screws                | Screws that originally held the turret in place | 4        |

| Tool                   | Description                                                          |
|------------------------|----------------------------------------------------------------------|
| Small slot screwdriver | To screw in the CAN wires into the CAN header                        |
| Torx screwdriver       | Screwdriver coming with the RoboMaster, to mount the M4 screws       |
| Pliers                 | To hold distance bolts in palce or mount the power switch connectors |

{% include gallery.html gallery_id="gallery_d131_base" gallery_data="gallery_d131_base" %}
