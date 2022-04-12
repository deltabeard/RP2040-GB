# RP2040_GB

Game Boy (DMG) emulator [Peanut-GB](https://github.com/deltabeard/Peanut-GB) on the Raspberry Pi RP2040 microcontroller, using an ILI9225 screen.

Runs at more than 70 fps without audio emulation. WIth frame skip and interlacing, can run at up to 120 fps.

![1](https://user-images.githubusercontent.com/3747104/162945331-605747fb-e48e-4b29-8007-9947afa29597.jpg)

## Future work

Further work is required to improve Peanut-GB for this microcontroller environment. This includes:

- Using an APU that is optimised for space and speed. No, or very few, floating point operations.
