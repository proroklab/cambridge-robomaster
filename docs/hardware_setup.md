---
layout: default
title: Hardware Setup
nav_order: 3
---

# Hardware
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

{% include gallery-styles.html %}

## RoboMaster Assembly
Follow the instructions of the manual to fully assemble the robot, including registration at DJI and setup/calibration using the DJI RoboMaster software.

## Preparation
Laser cutting/TBD, detailed bill of materials. TBD.

## Preliminaries
Information on crimping
{% include gallery.html gallery_id="gallery_common_crimping" gallery_data="gallery_common_crimping" %}

## Wire harness

Common, then for Raspi and Jetson separately
{% include gallery.html gallery_id="gallery_common_wiring" gallery_data="gallery_common_wiring" %}

{% include gallery.html gallery_id="gallery_d131_wiring" gallery_data="gallery_d131_wiring" %}

{% include gallery.html gallery_id="gallery_raspi_wiring" gallery_data="gallery_raspi_wiring" %}

## Robot

### Common
Robot base common, UI
{% include gallery.html gallery_id="gallery_common_base" gallery_data="gallery_common_base" %}

{% include gallery.html gallery_id="gallery_common_ui_wiring" gallery_data="gallery_common_ui_wiring" %}

{% include gallery.html gallery_id="gallery_common_ui_plate" gallery_data="gallery_common_ui_plate" %}

### Raspberry Pi
{% include gallery.html gallery_id="gallery_raspi_base" gallery_data="gallery_raspi_base" %}

### Jetson
{% include gallery.html gallery_id="gallery_d131_computer" gallery_data="gallery_d131_computer" %}

{% include gallery.html gallery_id="gallery_d131_base" gallery_data="gallery_d131_base" %}

{% include gallery-scripts.html %}
