# OpenVario Protocol

This document describes the OpenVario communication protocol. Communication between `sensord` and `variod` and XCSoar is based on NMEA sentences. NMEA is the abbreviation for "National Marine Electronics Association".

**Version:** 1.4  
**Date:** 14.03.2016 - battery voltage sentence added  
**Date:** 20.01.2026 - sentences added for Acceleration and Gyro

The original definition of these sentences can be found here.

---

## Specification

### Generic Definition

```
$POV,<type>,<value>,<type>,<value>, .....*<Checksum>
```

The NMEA sentence always starts with `$POV` and ends up with a checksum and `*`. Each `$POV` sentence can contain multiple datapoints defined by type and value. The type part is a uppercase letter describing the type and meaning of the following value according to the overview of supported types below. The value is a floating point number. The length of the sentence is not fixed, but terminated with the usual asterisk and a two character checksum.

**Example:**
```
$POV,P,+949.30,Q,-24.57*7D
```

---

## Value Pairs

The following value pairs are defined at the moment:

### Airspeed
- **S:** true airspeed in km/h
  - Example: `$POV,S,123.45*05`

### Pressure
- **P:** static pressure in hPa
  - Example: `$POV,P,1018.35*39`
  
- **Q:** dynamic pressure in Pa
  - Example: `$POV,Q,23.3*04`
  
- **R:** total pressure in hPa
  - Example: `$POV,R,1025.17*35`

### Temperature
- **T:** temperature in deg C
  - Example: `$POV,T,23.52*35`

### Voltage
- **V:** battery voltage in V
  - Example: `$POV,V,11.99*31`

### Vario
- **E:** TE vario in m/s
  - Example: `$POV,E,2.15*14` 

### Acceleration
- **A:** acceleration in m/s²
  - Example: `$POV,A,-1.5099,-.0292,13.7134*18`
  - Note: <value> consists of 3 components: X,Y,Z
  - Values are presented in body frame axis (X forward, Y right, Z down)

### Angular Rate (aka Gyroscope)
- **G:** Angular Rate in °/s
  - Example: `$POV,G,4.165,-8.709,-10.479*1B`
  - Note: <value> consists of 3 components: X,Y,Z
  - Values are presented in body frame axis (X forward, Y right, Z down)
  - roll, left wing up is positive
  - pitch, nose up is positive
  - yaw, right turn is positive

---

## Command

**C:** Command followed by type of command and parameter if necessary
```
C,<type>,<parameter>
```

Commands are grouped in "Types":

### Vario
- **VU:** Volume of external vario up by 10%
- **VD:** Volume of external vario down by 10%
- **VM:** Mute external Vario
- Example: `$POV,C,VU*09` → Set up volume by 10%

### McCready
- **MC,\<value\>**
  - Example: `$POV,C,MC,+0.5*28` → Set McCready to +0.5

### Wing Load
- **WL,\<value\>**
  - `<value>` is a factor of the additional weight added by water ballast in relation to the reference weight of the glider
  - Example: 
    - `$POV,C,WL,1.0*12` → No water ballast
    - `$POV,C,WL,1.1*13` → 10% more weight than reference weight

### Bugs
- **BU,\<value\>**
  - `<value>` reflects the value of bugs
  - Example: 
    - `$POV,C,BU,1.0*1E` → No Bugs
    - `$POV,C,WL,0.5*16` → 50% Bugs

### Real Polar
- **RPO,\<coff a\>, \<coff b\>, \<coff c\>**
  - `<coff x>` are reflecting the coefficients of the polar including bugs and ballast
  - Example: _(to be provided)_

### Ideal Polar
- **IPO,\<coff a\>, \<coff b\>, \<coff c\>**
  - `<coff x>` are reflecting the coefficients of the ideal polar (just glider, without bugs and ballast)
  - Example: _(to be provided)_

---

## Checksum

The NMEA Checksum is calculated over the NMEA string between `$` and `*`. The checksum is a hexadecimal number representing the XOR of all bytes.
