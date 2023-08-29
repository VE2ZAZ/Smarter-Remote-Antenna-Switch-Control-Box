# A Smarter Remote Antenna Switch Control Box 
### Add automatic antenna selection to an Ameritron or similar remote antenna switch when paired with an ICOM transceiver such as the IC-7300.

![](/images/Radio_Box.png)

Note: This repository stores the Arduino IDE code that runs on the Raspberry Pi Pico platform which controls the project described here. The full documentation for this project is available on [VE2ZAZ's website](https://ve2zaz.net/Antenna_Switch_Controller/Antenna_Sw_Control.htm).

The project consists of a smarter control box for a remote antenna switch. It manages antenna selection as a function of frequency, and does much more. It:

- Controls an off-the-shelf multi-port remote antenna switch, in my case the Ameritron RCS-10,
- Selects the proper antenna based on the operating frequency,
- Allows to select a different antenna for reception, and,
- Manages the two antennas between transmit and receive,
- Restores the previously selected antenna(s) at power on,
- Allows to override automatic antenna selection,
- Automatically switches to a dummy load on port 1 to protect the radio when trying to transmit into a receive-only antenna, and produces an audible alarm,
- Offers a convenient tactile color LCD screen interface, and provides audible feedback of touch actions,
- Has a configurable antenna list, with each antenna having its own description, type and operational range. The description of each antenna gets displayed on-screen.
- Uses Solid State Relays (SSR) to operate the remote antenna switch,
- Accomodates remote antenna switches equipped with up to 8 ports. Both binary port encoding and individual relay control lines are supported. Active-high or active-low control signals can be configured on the board,
- Uses ICOM's CI-V Remote physical port to read the current operating frequency,
- Senses the physical PTT line (active low) provided by the radio for fast antenna selection between transmit and receive,
- When using separate Rx and Tx antennas, offers a Delayed Transmit SSR output which can be used to inhibit transmitted output power for a short duration via the radio ALC or TX Inhibit signal. This allows the antenna switch relays to complete antenna transfer before RF is applied to the switch. The duration can be set during configuration.
- Offers a USB (serial) command-driven text console to configure the controller.
- Can be put together for less than $100, and for much less with a well-stuffed junk box and a bit of imagination and labor, especially when it comes to the enclosure.

Potential candidate remote antenna switch families are Ameritron RCS-4/8/10/12,  DX Engineering RR8B, WA4MCMkits RAAS-4/8 and Hamplus AS-61/81. But there are others, and you can even make your own remote switch using regular power relays.

Looks attractive? Visit [VE2ZAZ's website](https://ve2zaz.net/Antenna_Switch_Controller/Antenna_Sw_Control.htm).
